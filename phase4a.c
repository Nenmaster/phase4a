/*
 *
 *Programmers: Omar Mendivil & Ayman Mohamed
 *Phase3
 *This phase implements device drivers for the clock and terminal devices.
 * It includes system calls for Sleep(), TermRead(), and TermWrite(),
 * along with interrupt handling and process synchronization
 * */

#include <phase1.h>
#include <phase2.h>
#include <phase3.h>
#include <phase3_kernelInterfaces.h>
#include <phase3_usermode.h>
#include <phase4.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <usloss.h>
#include <usyscall.h>

#define MAX_LINES 10
#define MAX_LINE_LENGTH 80

typedef struct SleepRequest {
  int wakeupTime;
  int pid;
  struct SleepRequest *next;
} SleepRequest;

// Globals
SleepRequest *sleepQueue = NULL;
int clockMailbox;
int clockTicks = 0;

int termWriteLock[4];
int termReadMailbox[4];
int termInterruptMailbox[4];
int termWritePid[USLOSS_TERM_UNITS];

int readDoneMailbox;

char writeBuff[4][MAX_LINE_LENGTH];
int writeLen[4];
int writIdx[4];
int writeWaiting[4];

char lineBuff[4][MAX_LINE_LENGTH];
int lineLen[4];

// ProtoTypes
void enqueueSleepRequest(int pid, int wakeupTime);
void wakeUpProc();
int ClockDriver(void *arg);
int TerminalDriver(void *arg);
void sleepHandler(USLOSS_Sysargs *args);
void termWriteHandler(USLOSS_Sysargs *args);
void termReadHandler(USLOSS_Sysargs *args);
void termHandler(int type, void *arg);

void phase4_start_service_processes(void) {
  spork("ClockDriver", ClockDriver, NULL, USLOSS_MIN_STACK, 1);
  for (int i = 0; i < 4; i++) {
    spork("TermDriver", TerminalDriver, (void *)(long)i, USLOSS_MIN_STACK, 1);
  }
  for (int i = 0; i < 4; i++) {
    int ctrl = USLOSS_TERM_CTRL_RECV_INT(0) | USLOSS_TERM_CTRL_XMIT_INT(0);
    USLOSS_DeviceOutput(USLOSS_TERM_DEV, i, (void *)(long)ctrl);
  }
}

void phase4_init(void) {
  systemCallVec[SYS_SLEEP] = sleepHandler;
  systemCallVec[SYS_TERMWRITE] = termWriteHandler;
  systemCallVec[SYS_TERMREAD] = termReadHandler;
  USLOSS_IntVec[USLOSS_TERM_INT] = termHandler;

  clockMailbox = MboxCreate(1, 0);
  readDoneMailbox = MboxCreate(1, 0);
  for (int i = 0; i < 4; i++) {
    int semId;
    termWriteLock[i] = kernSemCreate(1, &semId);
    termReadMailbox[i] = MboxCreate(MAX_LINES, MAX_LINE_LENGTH);
    termInterruptMailbox[i] = MboxCreate(8, sizeof(int));
    termWritePid[i] = -1;
    writeLen[i] = 0;
    writIdx[i] = 0;
    writeWaiting[i] = 0;
    lineLen[i] = 0;
  }
}

// Waits for clock interrupts and wakes up any processes whose sleep time has
// expired
int ClockDriver(void *arg) {
  int status;
  while (1) {
    waitDevice(USLOSS_CLOCK_DEV, 0, &status);
    clockTicks++;
    wakeUpProc();
  }
  return 0;
}

// sends device status to terminal driver via mailbox
void termHandler(int type, void *arg) {
  int unit = (int)(long)arg;
  int status;
  USLOSS_DeviceInput(USLOSS_TERM_DEV, unit, &status);
  MboxCondSend(termInterruptMailbox[unit], &status, sizeof(int));
}

// Handles RECV and XMIT interrupts for one terminal and
// buffers incoming characters and manages transmition
int TerminalDriver(void *arg) {
  int unit = (int)(long)arg;
  int status;
  while (1) {
    if (MboxRecv(termInterruptMailbox[unit], &status, sizeof(int)) !=
        sizeof(int))
      continue;

    if (USLOSS_TERM_STAT_XMIT(status) == USLOSS_DEV_READY) {
      kernSemP(termWriteLock[unit]);
      if (writeWaiting[unit]) {
        if (writIdx[unit] < writeLen[unit]) {
          char ch = writeBuff[unit][writIdx[unit]++];
          kernSemV(termWriteLock[unit]);
          int ctrl = USLOSS_TERM_CTRL_CHAR(0, ch);
          ctrl = USLOSS_TERM_CTRL_XMIT_INT(ctrl);
          ctrl = USLOSS_TERM_CTRL_RECV_INT(ctrl);
          ctrl |= USLOSS_TERM_CTRL_XMIT_CHAR(0);
          USLOSS_DeviceOutput(USLOSS_TERM_DEV, unit, (void *)(long)ctrl);
        } else {
          int pidToUnblock = termWritePid[unit];
          writeWaiting[unit] = 0;
          termWritePid[unit] = -1;
          kernSemV(termWriteLock[unit]);
          if (pidToUnblock != -1)
            unblockProc(pidToUnblock);
          int ctrl =
              USLOSS_TERM_CTRL_RECV_INT(0) | USLOSS_TERM_CTRL_XMIT_INT(0);
          USLOSS_DeviceOutput(USLOSS_TERM_DEV, unit, (void *)(long)ctrl);
        }
      } else
        kernSemV(termWriteLock[unit]);
    }

    if (USLOSS_TERM_STAT_RECV(status) == USLOSS_DEV_BUSY) {
      char ch = USLOSS_TERM_STAT_CHAR(status);
      if (lineLen[unit] < MAX_LINE_LENGTH)
        lineBuff[unit][lineLen[unit]++] = ch;
      if (ch == '\n' || lineLen[unit] == MAX_LINE_LENGTH) {
        lineBuff[unit][lineLen[unit]] = '\0';
        MboxSend(termReadMailbox[unit], lineBuff[unit], lineLen[unit]);
        lineLen[unit] = 0;
      }
      int ctrl = USLOSS_TERM_CTRL_RECV_INT(0);
      kernSemP(termWriteLock[unit]);
      if (writeWaiting[unit])
        ctrl |= USLOSS_TERM_CTRL_XMIT_INT(0);
      kernSemV(termWriteLock[unit]);
      USLOSS_DeviceOutput(USLOSS_TERM_DEV, unit, (void *)(long)ctrl);
    }
  }
  return 0;
}

// Sends a string to the terminal, one character at a time.
void termWriteHandler(USLOSS_Sysargs *args) {
  char *buf = (char *)args->arg1;
  int len = (int)(long)args->arg2;
  int unit = (int)(long)args->arg3;
  int pid = getpid();

  if (!buf || len < 0 || len > MAX_LINE_LENGTH || unit < 0 || unit >= 4) {
    args->arg4 = (void *)(long)-1;
    return;
  }

  kernSemP(termWriteLock[unit]);
  memcpy(writeBuff[unit], buf, len);
  writeLen[unit] = len;
  writIdx[unit] = 0;
  writeWaiting[unit] = 1;
  termWritePid[unit] = pid;
  kernSemV(termWriteLock[unit]);

  if (len > 0) {
    char ch = writeBuff[unit][writIdx[unit]++];
    int ctrl = USLOSS_TERM_CTRL_CHAR(0, ch);
    ctrl = USLOSS_TERM_CTRL_XMIT_INT(ctrl);
    ctrl = USLOSS_TERM_CTRL_RECV_INT(ctrl);
    ctrl |= USLOSS_TERM_CTRL_XMIT_CHAR(0);
    USLOSS_DeviceOutput(USLOSS_TERM_DEV, unit, (void *)(long)ctrl);
    blockMe();
  } else {
    kernSemP(termWriteLock[unit]);
    writeWaiting[unit] = 0;
    termWritePid[unit] = -1;
    kernSemV(termWriteLock[unit]);
  }

  args->arg2 = (void *)(long)len;
  args->arg4 = (void *)(long)0;
}

// Delivers one full line from terminal input buffer to user.
void termReadHandler(USLOSS_Sysargs *args) {
  char *buffer = (char *)args->arg1;
  int bufSize = (int)(long)args->arg2;
  int unit = (int)(long)args->arg3;
  if (!buffer || bufSize <= 0 || unit < 0 || unit > 3) {
    args->arg4 = (void *)(long)-1;
    return;
  }
  char tempBuf[MAX_LINE_LENGTH];
  int len = MboxRecv(termReadMailbox[unit], tempBuf, MAX_LINE_LENGTH);
  if (len > bufSize)
    len = bufSize;
  memcpy(buffer, tempBuf, len);
  args->arg2 = (void *)(long)len;
  args->arg4 = (void *)(long)0;
}

void sleepHandler(USLOSS_Sysargs *args) {
  int seconds = (int)(long)args->arg1;
  if (seconds < 0) {
    args->arg4 = (void *)(long)-1;
    return;
  }
  int pid = getpid();
  int wakeupTick = clockTicks + (seconds * 10); // 10 ticks per second
  enqueueSleepRequest(pid, wakeupTick);
  blockMe();
  args->arg4 = (void *)(long)0;
}
// Helpers

// Inserts a new sleep request into the queue in ascending order by wakeup time
void enqueueSleepRequest(int pid, int wakeupTime) {
  SleepRequest *newReq = malloc(sizeof(SleepRequest));
  newReq->pid = pid;
  newReq->wakeupTime = wakeupTime;
  newReq->next = NULL;

  // Insert at the front if queue is empty or for equal/earlier tick
  if (sleepQueue == NULL || sleepQueue->wakeupTime >= wakeupTime) {
    newReq->next = sleepQueue;
    sleepQueue = newReq;
    return;
  }

  SleepRequest *curr = sleepQueue;
  while (curr->next && curr->next->wakeupTime < wakeupTime) {
    curr = curr->next;
  }
  newReq->next = curr->next;
  curr->next = newReq;
}

// Unblocks and removes all processes from the sleep queue whose wakeup time has
// passed
void wakeUpProc() {
  while (sleepQueue && sleepQueue->wakeupTime <= clockTicks) {
    SleepRequest *tmp = sleepQueue;
    sleepQueue = sleepQueue->next;
    unblockProc(tmp->pid);
    free(tmp);
  }
}
