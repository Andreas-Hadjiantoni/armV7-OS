#include "diningPhilosophers.h"
#include "../kernel/queueAPI/queue.h"
#include <string.h>
#define NUM 16 //number of philosophers

/*
  Since we already have the queue API, we can make use of it do achieve
  parent to child communication (and vice versa), using the following pattern:
  1) The parent declares a queue object (which will be used as a pipe)
  2) The parent declares a pointer to that queue/pipe
  3) The parent uses fork() syscall to spawn a child
  4) The child has access to the parent's pipe using the pointer
  5) The child can optionally delete (free()) its own copy of the pipe, as
    it's of no use
  6) The common pipe they now have can be used to communicate data by pushing
     and poping to the queue.

  The above pattern, can also be used to achieve chilt-to-child communication,
  by declaring a queue, then a pointer to that queue, and then 2 childs which
  will both have access to that pointer.
*/
/*
  We send an 0x0 or an pointer to a fork along the pipes to request a fork,
  or to give a fork respectively
*/
//We implement the Chandy/Misra solution to this problem:

typedef struct fork_ {
  bool clean;
} fork_;

void printMessage(char* str, int n) {
  for( int i = 0; i < n; i++ )
    PL011_putc( UART0, *str++, true );
}

int numOfDigits(int n) {
  if (n < 10)    {return 1;}
  if (n < 100)   {return 2;}
  if (n < 1000)  {return 3;}
  if (n < 10000) {return 4;}
}

uint32_t eat(int id, uint32_t totalEaten) {
  char str1[5];
  char str2[3];
  char *message;

  totalEaten++;

  itoa(str1, totalEaten);
  itoa(str2, id);

/*
  The following message cannot be constructed using strcat(...) as it
  requires a malloc(...) function. Instead, we "cheat".
  message = strcat(strcat(strcat(strcat("Philosopher ", str2), " eats "), str1), " units of spaghetti.\n");
  write(STDOUT_FILENO, message, 45);
*/

  //"cheat" function:
  printMessage("Philosopher ", 12);
  printMessage(str2, 2);
  printMessage(" eats ", 6);
  printMessage(str1, numOfDigits(totalEaten));
  //We use a write in the end, as it has the side effect of making an svc call:
  write(STDOUT_FILENO, " units of spaggetti.\n", 21);

  return totalEaten;
}

void requestFork(queue *neighbour) {
  uint32_t msg;
  msg = 0x0;
  push(neighbour, (void*)&msg);
}

void dealWithRequests(queue* from, queue* to, fork_ **f) {
  uint32_t temp;

  if ((*f) == NULL || (*f) -> clean) return;

  if ( !empty(from) && *peek_front(from) == 0 ){ //there is a request
    pop(from, &temp);

    (*f) -> clean = true;
    push(to, (void*)f);
    *f = NULL;
  }
}

void receiveForks(queue* neighbour, fork_** f) {
  while (empty(neighbour))
    yield();

  pop(neighbour, (void*)f);
}

//philosopher behavior
//Each philosopher can have at most 2 forks (fork1 and fork2)
void philosopher(
  int myID,
  fork_* myFork1,
  fork_* myFork2,
  queue *toRight,
  queue *toLeft,
  queue *fromRight,
  queue *fromLeft ) {

  fork_* fork1 = myFork1;
  fork_* fork2 = myFork2;

  uint32_t totalEaten = 0;

  while (1) {
    dealWithRequests(fromRight, toRight, &fork2);
    dealWithRequests(fromLeft, toLeft, &fork1);

    if ( fork1 == NULL )
      requestFork(toLeft);

    if ( fork2 == NULL )
      requestFork(toRight);

    if (fork1 == NULL)
      receiveForks(fromLeft, &fork1);

    if (fork2 == NULL)
      receiveForks(fromRight, &fork2);

    if ( fork1 != NULL && fork2 != NULL ) {
      totalEaten = eat( myID, totalEaten );
      fork1 -> clean = false;
      fork2 -> clean = false;
    }
  }
}

void main_dp() {

  int parentPID = getMyPid();
  int pid       = parentPID;
  uint8_t philID = 0;

  //We create 32 queues, 2 for every 2 neighbouring Philosophers
  queue p[2][NUM];
  queue *pipe[2][NUM];

  fork_ f[NUM];
  fork_ *forks[NUM];

  for (int i = 0; i < NUM; i++) {
    pipe[ 0 ][ i ] = &p[ 0 ][ i ];
    initialiseQueue( pipe[ 0 ][ i ], sizeof(fork_*) );

    pipe[ 1 ][ i ] = &p[ 1 ][ i ];
    initialiseQueue( pipe[ 1 ][ i ], sizeof(fork_*) );

    forks[ i ] = &f[ i ];
    f[ i ].clean = false;
  }

  for (int i = 0; i < NUM; i++) {
    philID++;
    pid = fork();
    if (pid != parentPID)
      break;
    }

  if (pid == parentPID)
    exit(EXIT_SUCCESS);

  //We use the pointer arrays to access the parent's queues and forks.
  if (philID == 1) {
    // 1st philosopher
    philosopher(philID, forks[0], forks[1], pipe[0][NUM - 1], pipe[1][0], pipe[1][NUM - 1], pipe[0][0]);

  } else {

    if (philID != NUM) {

      philosopher(philID, forks[ philID ], NULL , pipe[0][philID - 2], pipe[1][ philID - 1 ], pipe[1][philID - 2], pipe[0][ philID - 1 ]);

    } else
      //Last philosopher
      philosopher(NUM, NULL, NULL, pipe[0][NUM - 2], pipe[1][NUM - 1], pipe[1][NUM - 2], pipe[0][NUM - 1]);
  }
}
