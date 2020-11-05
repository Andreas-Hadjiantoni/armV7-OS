/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of
 * which can be found via http://creativecommons.org (and should be included as
 * LICENSE.txt within the associated archive or repository).
 */

#include "hilevel.h"
#include "queueAPI/queue.h"

#define BLOCK_SIZE 24
#define ENTRY_SIZE 3
#define DISK_SIZE  (1024 * 1024)

extern void     main_console();
extern uint32_t tos_console;

pcb_t pcb[ 100 ];
int executing = 0;

fileDescriptor fileTable[150];
int openFiles; //number of open files

// queues[0]:  highest Priority
// queues[4]:  lowest  Priority
queue queues[5];
//highest priority non empty queue
char hpneq;

// Queue in which the currently executing process was
char cepq; // "currently executing process queue"

const int timerLimits[5] = {0x0000147A, 0x000028F4, 0x000051E8, 0x00007ADC, 0x0000CCC4};
//                            ~5ms         ~10ms       ~20ms      ~30ms       ~50ms

// processes in higher priority queue levels will have to
const int ageRequirements[5] = {0x0, 0x0000CCC4, 0x00011EAC, 0x00019988, 0x0002664C};
//                                     ~50ms       ~70ms       ~100ms      ~150ms
int P_number = 1;

typedef enum {
  FROM_TIMER,
  FROM_IO_REQ,
  FROM_YIELD,
} schedulerSource;

// Returns first non empty queue
short getNextQ() {
  char counter = 0;

  while (empty(&queues[counter]) && counter < 5)
    counter++;

  if (counter == 5)
    return -1;
  else
    return counter;
}

void write_(char*x, int n) {
  for( int i = 0; i < n; i++ ) {
    PL011_putc( UART0, *x++, true );
  }
}

//Returns first non filled queue after queue indexed at "after":
short getNextNonFilledQ( int after ) {
  char counter = after;

  while (full(&queues[ counter ]) && counter < 5)
    counter++;

  if (counter == 5)
    return -1;
  else
    return counter;
}

void advanceOldProcesses() {
  queue temp;
  pcb_t *item;
  initialiseQueue(&temp, sizeof(pcb_t*));

  for (int i = 1; i < 5; i++) {
    while (!empty(&queues[i]) && ((pcb_t*)peek_front(&queues[i])) -> age >= ageRequirements[i]) {
      pop(&queues[i], (void*)&item);
      push(&temp, (void*)&item);
    }

    while (!empty(&temp)) {
      pop(&temp, (char*)&item);
      item -> age = 0;
      if (!full(&queues[ i -1 ]))
        push(&queues[ i - 1 ], (void*)&item);
      else  {
        // make room in queues[ i - 1 ] for the aged item
        pcb_t *item2;
        pop(&queues[ i - 1 ], (void*)&item2);
        push(&queues[ i - 1 ], (void*)&item);
        push(&queues[ i ], (void*)&item2);
      }
    }
  }
}

//move the process one level down if this invocation is due to the timer
//move the process one level up if this invocation is because the process made a system call (i/o bound)
void myScheduler(ctx_t* ctx, schedulerSource source) {

  pcb_t *nextProc;
  pcb_t *current = &pcb[ executing ];

  int qToPush;

  pcb[ executing ].ctx = *ctx;
  pcb[ executing ].status = STATUS_READY;

  if (source == FROM_TIMER || source == FROM_YIELD) {
    if (hpneq < 4) {
      qToPush = cepq + 1;
    } else {
      qToPush = cepq;
    }
  } else {
    if (hpneq > 0) {
      qToPush = cepq - 1;
    } else {
      qToPush = cepq;
    }
  }

  if ( !full( &queues[ qToPush ] ) )
    push(&queues[ qToPush ], (void*)&current);
  else {
    int available = getNextNonFilledQ( qToPush );
    if ( available != -1 ) {
      push(&queues[ available ], (void*)&current); // put in next available if qToPush is not
    } else {
      available = getNextNonFilledQ( 0 );
      if ( available != -1 ) {
        push(&queues[ available ], (void*)&current); // put in any available
      } else {
        write_("Out of Memory. Operation is aborted!", 36);
        return;
      }
    }
  }

  hpneq = getNextQ();
  //After the first invocation of this function, cepq and hpneq
  // are numerically equivalent
  cepq = hpneq;

  if (hpneq == -1) {
    //no processes left
  }
  pop(&queues[ hpneq ], (void*)&nextProc);
  executing = nextProc - pcb;

  nextProc -> status = STATUS_EXECUTING;
  *ctx = nextProc -> ctx;
}

void loadMasterFile() {
  fileTable[0].startAddress = 0;
  fileTable[0].mode = READ_WRITE;
  fileTable[0].pointer = 0;
  openFiles++;
}

void hilevel_handler_rst(ctx_t* ctx) {

  for (int i = 0; i < 5; i++)
    initialiseQueue(&queues[i], sizeof(pcb_t*));

  pcb_t console;
  memset(&console, 0, sizeof(pcb_t));

  console.pid = 0;
  console.ctx.cpsr = 0x50;
  console.status = STATUS_EXECUTING;
  console.main = main_console;
  console.ctx.pc = ( uint32_t )( main_console );
  console.tos = &tos_console;
  console.ctx.sp   = ( uint32_t )( &tos_console  );

  *ctx = console.ctx;
  pcb[ 0 ] = console;
  executing = 0;

  hpneq = 0;
  cepq = 0;

  TIMER0 -> Timer1Load  = timerLimits[ 0 ];     // select period for top level queue ~ 400 ns == 2684160
  TIMER0 -> Timer1Ctrl  = 0x00000002;           // select 32-bit   timer
  TIMER0 -> Timer1Ctrl |= 0x00000040;           // select periodic timer
  TIMER0 -> Timer1Ctrl |= 0x00000020;           // enable          timer interrupt
  TIMER0 -> Timer1Ctrl |= 0x00000080;           // enable          timer

  GICC0->PMR          = 0x000000F0;             // unmask all            interrupts
  GICD0->ISENABLER1  |= 0x00000010;             // enable timer          interrupt
  GICC0->CTLR         = 0x00000001;             // enable GIC interface
  GICD0->CTLR         = 0x00000001;             // enable GIC distributor

  openFiles = 0;
  loadMasterFile();
  int_enable_irq();

  return;
}

void ageProcesses() {
  for (int i = 0; i < P_number; i++)
    if (pcb[i].status == STATUS_READY)
      pcb[i].age += TIMER0->Timer1Load;
}

void hilevel_handler_irq(ctx_t* ctx) {

  int_unable_irq();

  // Step 2: read  the interrupt identifier so we know the source.

  uint32_t id = GICC0->IAR;

  // Step 4: handle the interrupt, then clear (or reset) the source.

  switch ( id ) {
    case GIC_SOURCE_TIMER0: {
      if (getNextQ() != -1 ) { //There are no processes if otherwise
        ageProcesses();
        advanceOldProcesses();
        myScheduler(ctx, FROM_TIMER);
      }

      TIMER0->Timer1Load  = timerLimits[ hpneq ];
      TIMER0->Timer1IntClr = 0x01;
      break;
    }
  }

  // Step 5: write the interrupt identifier to signal we're done.
  GICC0->EOIR = id;

  int_enable_irq();
  return;
}

bool tosIsUsed(uint32_t *tos) {
  for (pcb_t *proc = &pcb[ 0 ]; proc < &pcb[ P_number ]; proc++)
    if ((*proc).tos == tos)
      return true;

  return false;
}

uint32_t *malloc_() {
  // loop through all potential stack tops
  for (uint32_t *tos = (&tos_console) + 0x00001000 / sizeof(uint32_t); *tos <= 0xFFFFFFFF; tos += 0x00001000 ) {
    if (!tosIsUsed( tos )) {
      return tos;
    }
  }

  return 0;
}

void moveProcessesDown( int index, int limit ) {
  for (int i = index + 1; i < limit; i++)
    pcb[ i - 1 ] = pcb[ i ];
}

int getIndex( int pid, int limit ) {
  for (int i = 0; i < limit; i++)
    if ( pcb[ i ].pid == pid )
      return i;
  return -1;
}

void descheduleAndUpdateQueues( pcb_t *entry ) {
  queue temp;
  pcb_t* eachEntry;
  initialiseQueue(&temp, sizeof(pcb_t*));

  for (int i = 0; i < 5; i++) {
    while (!empty(&queues[i])) {
      pop(&queues[i], (void*)&eachEntry);
      if (eachEntry != entry) {
        if (eachEntry > entry)
          eachEntry--;
        push( &temp, (void*)&eachEntry);
      }
    }

    while (!empty(&temp)) {
      pop(&temp, (void*)&eachEntry);
      push(&queues[ i ], (void*)&eachEntry);
    }
  }
}

int getStrLen(const char* fname) {
  int counter = 0;

  while (fname[counter] != '\0')
    counter++;

  return counter;
}

uint32_t toUint32(char c[ 3 ]) {
  uint32_t x = 0;
  x |= ( c[ 0 ] << 16 ) | ( c[ 1 ] << 8 ) | ( c[ 2 ] );
  return x;
}

char* fromUint32(uint32_t x, char* c) {
  c[ 0 ] = x & 0x00FF0000;
  c[ 1 ] = x & 0x0000FF00;
  c[ 2 ] = x & 0x000000FF;
  return c;
}

bool AddressIsInBlock(uint32_t blockAddress, uint32_t address) {
  return address >= blockAddress && address < blockAddress + BLOCK_SIZE;
}

uint32_t searchInBlock(uint32_t blockAddress, char entry[ 3 ], uint32_t endAddress) {
  uint8_t* block;
  uint8_t* limit;

  if (AddressIsInBlock(blockAddress, endAddress) &&
      (uint8_t*)endAddress < block + BLOCK_SIZE)
    limit = (uint8_t*)endAddress;
  else
    limit = block + BLOCK_SIZE - ENTRY_SIZE;

  disk_rd( blockAddress, block, BLOCK_SIZE );

  for (uint8_t* i = block; i < limit; i += ENTRY_SIZE)
    if (memcmp(i, entry, ENTRY_SIZE) == 0)
      return (uint32_t)*i;

  return -1;

}

char *getFileName(uint32_t startAddress, char *fname) {
  uint32_t eachBlockAddress = startAddress;
  char counter = 0;
  char* nextBlockAddress;
  uint8_t *block, *i;

  do {
    disk_rd(eachBlockAddress, block, BLOCK_SIZE);

    nextBlockAddress = block + BLOCK_SIZE - ENTRY_SIZE;
    eachBlockAddress = toUint32(nextBlockAddress);

    for(i = block; i < block + BLOCK_SIZE - ENTRY_SIZE && (char)*i != '\0'; i+= ENTRY_SIZE) {
      fname[ counter ] = (char)*i;
      counter++;
    }

    if ((char)*i == '\0') {
      fname[counter] = '\0';
      break;
    }

  } while (1);

  return fname;
}

uint32_t getEndAddress(uint32_t startAddress) {
  uint8_t* endAddress;
  disk_rd(startAddress, endAddress, ENTRY_SIZE);
  return toUint32((char*)endAddress);
}

uint32_t getNextAddress(uint32_t blockAddress) {
  uint8_t* nextAddress;
  disk_rd(blockAddress + BLOCK_SIZE - ENTRY_SIZE, nextAddress, ENTRY_SIZE);
  return toUint32((char*)nextAddress);
}

//ok
uint32_t searchForFile(const char* fname) {
  uint32_t eachBlockAddress = 0;
  uint8_t *block;
  uint8_t *limit;
  char fileNameTemp[ 50 ];
  uint32_t endAddress = getEndAddress(0);

  do {
    disk_rd( eachBlockAddress, block, BLOCK_SIZE );

    for (uint8_t* entry = block; entry < block + BLOCK_SIZE - ENTRY_SIZE; entry += ENTRY_SIZE)
      if (memcmp(getFileName( toUint32( (char*)entry ), fileNameTemp ), fname, getStrLen(fname)) == 0)
        return toUint32( (char*)entry );

    eachBlockAddress = getNextAddress( eachBlockAddress );
  } while (!AddressIsInBlock(eachBlockAddress, endAddress));

  for (uint8_t* entry = block; entry <= (uint8_t*)endAddress; entry += ENTRY_SIZE)
    if (memcmp(getFileName( toUint32( (char*)entry ), fileNameTemp ), fname, getStrLen(fname)) == 0)
      return toUint32( (char*)entry );

  return -1;
}

//OK
uint32_t open_( const char* fname, uint32_t address, fmode mode ) {
  fileTable[ openFiles ].fd = fileTable[ openFiles ].fd + 1;
  fileTable[ openFiles ].startAddress = address;
  fileTable[ openFiles ].mode = mode;
  fileTable[ openFiles ].pointer = address;
  fileTable[ openFiles ].pid = pcb[ executing ].pid;
  openFiles++;
}

//ok
bool fileUsesBlock( char fStartAddress[ 3 ], char fendAddress[ 3 ], uint32_t blockAddress ) {
  uint32_t fileStartAddress = toUint32( fStartAddress );
  uint32_t eachBlockAddress = fileStartAddress;

  while ( eachBlockAddress != blockAddress && !AddressIsInBlock( eachBlockAddress, toUint32(fendAddress) ) ) {
    disk_rd( eachBlockAddress + BLOCK_SIZE - ENTRY_SIZE, (uint8_t*)&eachBlockAddress, ENTRY_SIZE);
  }

  if (eachBlockAddress == blockAddress)
    return true;
  else
   return false;

}


bool incrementPointer( fileDescriptor *fdesc ){
  char entry[ 3 ];
  char eof[ 3 ] = { 0x00, 0x00, 0x00 };


  if ( fdesc -> pointer % BLOCK_SIZE == BLOCK_SIZE - ENTRY_SIZE ) {
    disk_rd( fdesc -> pointer, (uint8_t*)&entry, ENTRY_SIZE);
    if ( memcmp( entry, eof, ENTRY_SIZE ) == 0)
      return false;
    else {
      disk_rd( fdesc -> pointer, (uint8_t*)fdesc -> pointer, ENTRY_SIZE);
      return true;
    }
  } else {
    fdesc -> pointer += ENTRY_SIZE;
    return true;
  }
}

// Scan the file system
bool BlockIsFree(uint32_t blockAddress) {
    uint8_t* eachMasterBlock;
    uint32_t eachMasterBlockAddress = 0;
    char *entry, MasterEndAddress[ 3 ];
    uint8_t* limit;
    bool hasAnotherBlock = true;
    char temp[ 3 ];

    disk_rd(0, MasterEndAddress, ENTRY_SIZE);

    do {
      disk_rd( eachMasterBlockAddress, eachMasterBlock, BLOCK_SIZE - ENTRY_SIZE );

      if (AddressIsInBlock(eachMasterBlockAddress, toUint32(MasterEndAddress)) &&
          (uint8_t*)MasterEndAddress < eachMasterBlock + BLOCK_SIZE)  {
        limit = (uint8_t*)MasterEndAddress;
        hasAnotherBlock = false;
      }
      else
        limit = eachMasterBlock + BLOCK_SIZE - ENTRY_SIZE;

      for (entry = (char*)eachMasterBlock; entry < (char*)limit; entry += ENTRY_SIZE )
        if (fileUsesBlock( entry, fromUint32(getEndAddress(toUint32(entry)), temp), blockAddress ))
          return false;

      eachMasterBlockAddress = getNextAddress(eachMasterBlockAddress);

    } while ( hasAnotherBlock );

    return true;
}

void setFName(uint32_t startAddress, const char* fname) {

}

void setFEndAddress(uint32_t startAddress, int size) {

}

void updateMasterFile(uint32_t blockAddress) {
  uint32_t endAddress = getEndAddress(0);
  uint32_t newEndAddress = endAddress + ENTRY_SIZE;
  disk_wr(endAddress, (uint8_t*)&blockAddress, ENTRY_SIZE);
  disk_wr(0, (uint8_t*)&newEndAddress, ENTRY_SIZE);
}

uint32_t creat(const char* fname) {
  uint32_t blockAddress = BLOCK_SIZE; //first block belongs to master file, so directly jump to the second

  while ( !BlockIsFree( blockAddress ) )
    if ( blockAddress + BLOCK_SIZE <= DISK_SIZE )
      blockAddress += BLOCK_SIZE;
    else
      return -1;

  updateMasterFile(blockAddress);
  setFName(blockAddress, fname);
  setFEndAddress(blockAddress, getStrLen(fname) + ENTRY_SIZE);
  return blockAddress;
}

void hilevel_handler_svc( ctx_t* ctx, uint32_t id ) {
  /* Based on the identifier encoded as an immediate operand in the
   * instruction,
   *
   * - read the arguments from preserved usr mode registers,
   * - perform whatever is appropriate for this system call,
   * - write any return value back to preserved usr mode registers.
   */


  switch( id ) {
    case SYS_YIELD : {
      pcb[ executing ].ctx = *ctx;
      myScheduler( ctx, FROM_YIELD );
      break;
    }
    case SYS_WRITE : {
      pcb[ executing ].ctx = *ctx;
      int   fd = ( int   )( ctx->gpr[ 0 ] );
      char*  x = ( char* )( ctx->gpr[ 1 ] );
      int    n = ( int   )( ctx->gpr[ 2 ] );

      if (fd == STDOUT_FILENO || fd == STDERR_FILENO)
        write_(x, n);
      else {

      }

      myScheduler( ctx, FROM_IO_REQ );
      break;
    }
    case 0x02 : { // => read(...)
      pcb[ executing ].ctx = *ctx;
      int   fd = ( int   )( ctx->gpr[ 0 ] );
      char*  x = ( char* )( ctx->gpr[ 1 ] );
      int    n = ( int   )( ctx->gpr[ 2 ] );

      if (fd == STDOUT_FILENO || fd == STDERR_FILENO)
        for( int i = 0; i < n; i++ ) {
          x[ i ] = PL011_getc( UART1, true );

          if( x[ i ] == '\x0A' ) {
            x[ i ] = '\x00'; break;
          }
        }
      else {

      }

      myScheduler( ctx, FROM_IO_REQ );
      break;
    }
    case SYS_FORK : {
      pcb_t new;
      pcb[ executing ].ctx = *ctx; //update process table
      size_t stackSize = (uint32_t)pcb[ executing ].tos - ctx -> sp;

      new = pcb[ executing ];
      new.pid = pcb[ P_number - 1].pid + 1;
      new.age = 0;
      new.status = STATUS_READY;
      new.tos = malloc_();

      if (new.tos == 0) {
        write_("Out of Memory. Operation is aborted!", 36);
        break;
      }//couldnt find appropriate stack to allocate

      new.ctx.sp = (uint32_t)new.tos - stackSize;
      memcpy((void*)new.ctx.sp, (void*)pcb[ executing ].ctx.sp, stackSize);
      new.ctx.gpr[ 0 ] = new.pid;
      ctx -> gpr[ 0 ] = pcb[ executing ].pid;
      pcb[ P_number ] = new;

      pcb_t *addr = &pcb[ P_number ];

      if (!full(&queues[ cepq ]))
        push(&queues[ cepq ], (void*)&addr); //attempt to put it in its parents' queue
      else {//if not successful, put in next available queue
        int available = getNextNonFilledQ( cepq );
        if (available != -1)
          push(&queues[ available ], (void*)&addr);
        else {
          available = getNextNonFilledQ( 0 );
          if (available != -1) {
            push(&queues[ available ], (void*)&addr); //put in any available
          } else {
            write_("Out of Memory. Operation is aborted!", 36);
            break;
          }
        }
      }

      P_number++;
      break;
    }
    case SYS_EXIT : {
      uint32_t currentR0 = ctx -> gpr[0];
      ctx -> gpr[0] = pcb[executing].pid;
      hilevel_handler_svc(ctx, 0x06);
      ctx -> gpr[0] = currentR0;
      break;
    }
    case SYS_EXEC : {
      pcb[ executing ].main = (uint32_t*)(ctx -> gpr[0]);
      ctx -> pc = ctx -> gpr[ 0 ];
      ctx -> sp = (uint32_t)pcb[ executing ].tos;
      break;
    }
    case SYS_KILL : {

      int PIDtoKill = ctx ->gpr[ 0 ];

      int index = getIndex( PIDtoKill, P_number );
      if ( index == -1 ) return;

      moveProcessesDown( index, P_number );
      descheduleAndUpdateQueues( &pcb[ index ] );
      P_number--;

      if ( index == executing ) {
        hpneq = getNextQ();
        cepq = hpneq;
        pcb_t *nextProc;
        pop(&queues[ hpneq ], (void*)&nextProc);
        executing = nextProc - pcb;

        nextProc -> status = STATUS_EXECUTING;
        *ctx = nextProc -> ctx;

        if (hpneq == -1)
          write_("No processes left!", 18);
      }

      break;
    }
    case SYS_OPEN: {
      const char* fname = (char*)(ctx -> gpr[0]);
      int m = ctx -> gpr[1];
      fmode mode;
      uint32_t fd;
      uint32_t address;

      switch ( m ) {
        case 0 : { mode = READ;       break; }
        case 1 : { mode = WRITE;      break; }
        case 2 : { mode = READ_WRITE; break; }
        default: { break; }
      }

      address = searchForFile(fname);
      if (address != -1)
        fd = open_(fname, address, mode);
      else {
        address = creat(fname);
        fd = open_(fname, address, mode);
      }

      ctx->gpr[ 0 ] = fd;

      break;
    }
    case SYS_GET_PID: {
      ctx -> gpr[ 0 ] = pcb[ executing ].pid;
      break;
    }
    default   : { // 0x?? => unknown/unsupported
      break;
    }
  }

  return;
}
