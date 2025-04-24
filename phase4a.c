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
  unsigned int wakeupTime;
  int pid;
  struct SleepRequest *next;
} SleepRequest;

SleepRequest *sleepQueue = NULL;
int clockMailbox;
int clockTicks = 0;

int termWriteLock[4];
int termReadMailbox[4];
int termInterruptMailbox[4];
int termWriterPid[USLOSS_TERM_UNITS];
int readDoneMailbox;
int writeDoneMailbox;
char writeBuffer[4][MAX_LINE_LENGTH];
int writeLength[4];
int writeIndex[4];
int writeReady[4];

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
    ctrl |= USLOSS_TERM_CTRL_RECV_INT(0);
    ctrl |= USLOSS_TERM_CTRL_XMIT_INT(0);
    // printf("phase4_start_service: unit %d ctrl=0x%04x\n", i, ctrl);
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
  writeDoneMailbox = MboxCreate(1, 0);
  for (int i = 0; i < 4; i++) {
    int semId;
    termWriteLock[i] = kernSemCreate(1, &semId);
    termReadMailbox[i] = MboxCreate(MAX_LINES, MAX_LINE_LENGTH);
    termInterruptMailbox[i] = MboxCreate(8, sizeof(int)); // 👈 Add this line
    termWriterPid[i] = -1;
    writeLength[i] = 0;
    writeIndex[i] = 0;
    writeReady[i] = 0;
    lineLength[i] = 0;
  }
}

int ClockDriver(void *arg) {
  int status;
  unsigned int currTime;

  while (1) {
    waitDevice(USLOSS_CLOCK_DEV, 0, &status);

    USLOSS_DeviceInput(USLOSS_CLOCK_DEV, 0, (int *)&currTime);

    checkWakeups(currTime);
  }
  return 0;
}

void termHandler(int type, void *arg) {
  int unit = (int)(long)arg;
  int status;

  USLOSS_DeviceInput(USLOSS_TERM_DEV, unit, &status);

  int result = MboxCondSend(termInterruptMailbox[unit], &status, sizeof(int));
}

int TerminalDriver(void *arg) {
  int unit = (int)(long)arg;
  int status;

  while (1) {
    int result = MboxRecv(termInterruptMailbox[unit], &status, sizeof(int));
    if (result != sizeof(int)) {
      continue;
    }

    int xmit_status = USLOSS_TERM_STAT_XMIT(status);

    if (xmit_status == USLOSS_DEV_READY) {

      kernSemP(termWriteLock[unit]);
      int wasReady = writeReady[unit];

      if (wasReady) {
        if (writeIndex[unit] < writeLength[unit]) {
          char ch = writeBuffer[unit][writeIndex[unit]++];
          kernSemV(termWriteLock[unit]);

          int ctrl = 0;
          ctrl = USLOSS_TERM_CTRL_CHAR(ctrl, ch);
          ctrl = USLOSS_TERM_CTRL_XMIT_INT(ctrl);
          ctrl = USLOSS_TERM_CTRL_RECV_INT(ctrl);
          ctrl |= USLOSS_TERM_CTRL_XMIT_CHAR(0);

          int outPutRes =
              USLOSS_DeviceOutput(USLOSS_TERM_DEV, unit, (void *)(long)ctrl);

        } else {
          int pidToUnblock = termWriterPid[unit];
          writeReady[unit] = 0;
          termWriterPid[unit] = -1;
          kernSemV(termWriteLock[unit]);

          if (pidToUnblock != -1) {
            unblockProc(pidToUnblock);
          }
          int final_ctrl = 0;
          final_ctrl = USLOSS_TERM_CTRL_RECV_INT(final_ctrl);
          final_ctrl = USLOSS_TERM_CTRL_XMIT_INT(final_ctrl);
          USLOSS_DeviceOutput(USLOSS_TERM_DEV, unit, (void *)(long)final_ctrl);
        }
      } else {
        kernSemV(termWriteLock[unit]); // Release lock
      }
    }

    int recv_status = USLOSS_TERM_STAT_RECV(status);
    if (recv_status == USLOSS_DEV_BUSY) {
      char ch = USLOSS_TERM_STAT_CHAR(status);

      if (lineLength[unit] < MAX_LINE_LENGTH) {
        lineBuffer[unit][lineLength[unit]++] = ch;
      }

      if (ch == '\n' || lineLength[unit] == MAX_LINE_LENGTH) {
        lineBuffer[unit][lineLength[unit]] = '\0';
        int send_res =
            MboxSend(termReadMailbox[unit], lineBuffer[unit], lineLength[unit]);
        if (send_res < 0) {
          printf("TerminalDriver[unit %d]: MboxSend to termReadMailbox failed! "
                 "Result = %d\n",
                 unit, send_res);
        }
        lineLength[unit] = 0;
      }
      kernSemP(termWriteLock[unit]);
      int needsXmitInt = writeReady[unit];
      kernSemV(termWriteLock[unit]);

      int ctrl = 0;
      ctrl = USLOSS_TERM_CTRL_RECV_INT(ctrl);
      if (needsXmitInt) {
        ctrl = USLOSS_TERM_CTRL_XMIT_INT(ctrl);
      }
      USLOSS_DeviceOutput(USLOSS_TERM_DEV, unit, (void *)(long)ctrl);
    }
  }
  return 0;
}

void termWriteHandler(USLOSS_Sysargs *args) {
  char *buf = (char *)args->arg1;
  int len = (int)(long)args->arg2;
  int unit = (int)(long)args->arg3;
  int callingPid = getpid(); // Get PID of the process calling TermWrite

  if (!buf || len < 0 || len > MAX_LINE_LENGTH || unit < 0 ||
      unit >= USLOSS_TERM_UNITS) {
    args->arg4 = (void *)(long)-1;
    return;
  }

  kernSemP(termWriteLock[unit]);

  if (writeReady[unit]) {
    kernSemV(termWriteLock[unit]);
    printf("termWriteHandler[unit %d, pid %d]: Driver busy!\n", unit,
           callingPid);
    args->arg4 = (void *)(long)-1;
    return;
  }
  memcpy(writeBuffer[unit], buf, len);
  writeLength[unit] = len;
  writeIndex[unit] = 0;
  writeReady[unit] = 1;
  termWriterPid[unit] = callingPid;

  kernSemV(termWriteLock[unit]);

  if (writeLength[unit] > 0) {
    char ch = writeBuffer[unit][writeIndex[unit]++];
    int ctrl = 0;
    ctrl = USLOSS_TERM_CTRL_CHAR(ctrl, ch);
    ctrl = USLOSS_TERM_CTRL_XMIT_INT(ctrl);
    ctrl = USLOSS_TERM_CTRL_RECV_INT(ctrl);
    ctrl |= USLOSS_TERM_CTRL_XMIT_CHAR(0);
    USLOSS_DeviceOutput(USLOSS_TERM_DEV, unit, (void *)(long)ctrl);

    blockMe();
  } else {
    kernSemP(termWriteLock[unit]);
    writeReady[unit] = 0;
    termWriterPid[unit] = -1;
    kernSemV(termWriteLock[unit]);
  }
  args->arg2 = (void *)(long)len;
  args->arg4 = (void *)(long)0;
}

void termReadHandler(USLOSS_Sysargs *args) {
  char *buffer = (char *)args->arg1;
  int bufSize = (int)(long)args->arg2;
  int unit = (int)(long)args->arg3;
  //  printf("in read handler\n");
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
  MboxSend(readDoneMailbox, NULL, 0);
}

void sleepHandler(USLOSS_Sysargs *args) {
  int seconds = (int)(long)args->arg1;
  if (seconds < 0) {
    args->arg4 = (void *)(long)-1;
    return;
  }

  int pid = getpid();
  unsigned int currTime;
  unsigned int wakeUpTime;

  int retval = USLOSS_DeviceInput(USLOSS_CLOCK_DEV, 0, (int *)&currTime);
  if (retval != USLOSS_DEV_OK) {
    args->arg4 = (void *)(long)-1;
    return;
  }

  wakeUpTime = currTime + ((unsigned long long)seconds * 1000000);

  addSleepRequest(pid, wakeUpTime);
  blockMe();
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

void checkWakeups(unsigned int currTime) {
  while (sleepQueue != NULL && sleepQueue->wakeupTime <= currTime) {
    SleepRequest *tmp = sleepQueue;
    sleepQueue = sleepQueue->next;
    unblockProc(tmp->pid);
    free(tmp);
  }
}
