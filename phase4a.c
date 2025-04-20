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

// ----- Globals and Structures -----
typedef struct SleepRequest {
  int wakeupTime;
  int pid;
  struct SleepRequest *next;
} SleepRequest;

SleepRequest *sleepQueue = NULL;
int clockMailbox;
int clockTicks = 0;

int termWriteLock[4];
int termReadMailbox[4];
int termInterruptMailbox[4];

char lineBuffer[4][MAX_LINE_LENGTH];
int lineLength[4];

// prototypes
void addSleepRequest(int pid, int wakeupTime);
void checkWakeups();
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
    int ctrl = 0;
    ctrl = USLOSS_TERM_CTRL_RECV_INT(
        ctrl); // macro to set the recv interrupt enable bit
    ctrl = USLOSS_TERM_CTRL_XMIT_INT(ctrl);
    USLOSS_DeviceOutput(USLOSS_TERM_DEV, i, (void *)(long)ctrl);
  }
}

void phase4_init(void) {
  systemCallVec[SYS_SLEEP] = sleepHandler;
  systemCallVec[SYS_TERMWRITE] = termWriteHandler;
  systemCallVec[SYS_TERMREAD] = termReadHandler;

  USLOSS_IntVec[USLOSS_TERM_INT] = termHandler;

  clockMailbox = MboxCreate(1, 0);
  for (int i = 0; i < 4; i++) {
    int semId;
    termWriteLock[i] = kernSemCreate(1, &semId);
    termReadMailbox[i] = MboxCreate(MAX_LINES, MAX_LINE_LENGTH);
    termInterruptMailbox[i] = MboxCreate(1, sizeof(int)); // 👈 Add this line
    lineLength[i] = 0;
  }
}

int ClockDriver(void *arg) {
  int status;
  while (1) {
    waitDevice(USLOSS_CLOCK_DEV, 0, &status);
    clockTicks++;
    checkWakeups();
  }
  return 0;
}

int TerminalDriver(void *arg) {
  int unit = (int)(long)arg;
  int status;

  while (1) {
    int result = MboxRecv(termInterruptMailbox[unit], &status, sizeof(int));

    if (result != sizeof(int)) {
      continue;
    }

    if (USLOSS_TERM_STAT_RECV(status)) {
      char ch = USLOSS_TERM_STAT_CHAR(status);

      if (lineLength[unit] < MAX_LINE_LENGTH) {
        lineBuffer[unit][lineLength[unit]++] = ch;
      }

      if (ch == '\n' || lineLength[unit] == MAX_LINE_LENGTH) {
        // Ensure the buffer is null-terminated for string operations
        lineBuffer[unit][lineLength[unit]] = '\0';
        MboxSend(termReadMailbox[unit], lineBuffer[unit], lineLength[unit]);
        lineLength[unit] = 0;
      }
    } else if (USLOSS_TERM_STAT_XMIT(status)) {
      printf("TerminalDriver: XMIT status received for unit %d\n", unit);
      // Just acknowledge the XMIT status, no further action needed
    } else {
      printf("TerminalDriver: Unknown status 0x%08x for unit %d\n", status,
             unit);
    }
  }
  return 0;
}

void sleepHandler(USLOSS_Sysargs *args) {
  int seconds = (int)(long)args->arg1;
  if (seconds < 0) {
    args->arg4 = (void *)(long)-1;
    return;
  }

  int pid = getpid();
  int wakeupTime = clockTicks + seconds * 10;
  addSleepRequest(pid, wakeupTime);
  blockMe();
  args->arg4 = (void *)(long)0;
}

void termWriteHandler(USLOSS_Sysargs *args) {
  char *buf = (char *)args->arg1;
  int len = (int)(long)args->arg2;
  int unit = (int)(long)args->arg3;

  if (!buf || len < 0 || unit < 0 || unit > 3) {
    args->arg4 = (void *)(long)-1;
    return;
  }

  kernSemP(termWriteLock[unit]);

  for (int i = 0; i < len; i++) {
    // Send the character
    int ctrl = USLOSS_TERM_CTRL_XMIT_CHAR((int)buf[i]);
    ctrl = USLOSS_TERM_CTRL_XMIT_INT(ctrl);
    ctrl = USLOSS_TERM_CTRL_RECV_INT(ctrl); // preserve receive interrupts

    USLOSS_DeviceOutput(USLOSS_TERM_DEV, unit, (void *)(long)ctrl);

    // Wait for transmission to complete
    int status;
    waitDevice(USLOSS_TERM_DEV, unit, &status);
  }

  kernSemV(termWriteLock[unit]);

  args->arg2 = (void *)(long)len;
  args->arg4 = (void *)(long)0;
}

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

// ----- Helpers -----
void addSleepRequest(int pid, int wakeupTime) {
  SleepRequest *newReq = malloc(sizeof(SleepRequest));
  newReq->pid = pid;
  newReq->wakeupTime = wakeupTime;
  newReq->next = NULL;

  if (sleepQueue == NULL || sleepQueue->wakeupTime > wakeupTime) {
    newReq->next = sleepQueue;
    sleepQueue = newReq;
    return;
  }

  SleepRequest *curr = sleepQueue;
  while (curr->next && curr->next->wakeupTime <= wakeupTime) {
    curr = curr->next;
  }
  newReq->next = curr->next;
  curr->next = newReq;
}

void checkWakeups() {
  while (sleepQueue && sleepQueue->wakeupTime <= clockTicks) {
    SleepRequest *tmp = sleepQueue;
    sleepQueue = sleepQueue->next;
    unblockProc(tmp->pid);
    free(tmp);
  }
}

void termHandler(int type, void *arg) {
  int unit = (int)(long)arg;
  int status;

  USLOSS_DeviceInput(USLOSS_TERM_DEV, unit, &status);

  int result = MboxCondSend(termInterruptMailbox[unit], &status, sizeof(int));
  if (result != 0) {
    printf("termHandler(): warning - couldn't send status to mailbox\n");
  }
}
