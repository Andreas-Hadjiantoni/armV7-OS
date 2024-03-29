/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of
 * which can be found via http://creativecommons.org (and should be included as
 * LICENSE.txt within the associated archive or repository).
 */

#ifndef __HILEVEL_H
#define __HILEVEL_H

// Include functionality relating to newlib (the standard C library).

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <string.h>
#include   "GIC.h"
#include "disk.h"
#include "PL011.h"
#include "SP804.h"

// Include functionality relating to the   kernel.

#include "lolevel.h"
#include "int.h"
#include "libc.h"

typedef int pid_t;

typedef enum {
  STATUS_CREATED,
  STATUS_READY,
  STATUS_EXECUTING,
  STATUS_WAITING,
  STATUS_TERMINATED
} status_t;

typedef struct {
  uint32_t cpsr, pc, gpr[ 13 ], sp, lr;
} ctx_t;

typedef enum {
  READ,
  WRITE,
  READ_WRITE,
} fmode;

typedef struct {
  uint32_t fd;
  pid_t pid; //pid of the process which uses this file
  uint32_t startAddress;
  uint32_t endAddress;
  fmode mode;
  uint32_t pointer;
} fileDescriptor;

typedef struct {
  pid_t pid;
  status_t status;
  uint32_t age;
  uint32_t *tos; // <= top of stack
  const void *main; // <= location of main
  ctx_t ctx;
} pcb_t;


#endif
