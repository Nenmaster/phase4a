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
  while (1) {
    waitDevice(USLOSS_CLOCK_DEV, 0, &status);
    clockTicks++;
    checkWakeups();
  }
  return 0;
}

void termHandler(int type, void *arg) {
  int unit = (int)(long)arg;
  int status;

  USLOSS_DeviceInput(USLOSS_TERM_DEV, unit, &status);

  int result = MboxCondSend(termInterruptMailbox[unit], &status, sizeof(int));

  // if (unit == 1) {
  //   printf("termHandler(): received interrupt for unit %d, status=0x%x\n",
  //          (int)(long)arg, status);
  // }

  // if (unit == 1) {
  //   printf("termHandler(unit=%d): USLOSS_DeviceInput -> status = 0x%x (%d), "
  //          "MboxCondSend = %d\n",
  //          unit, status, status, result);
  // }
}

// ----- TerminalDriver (Corrected for Unblocking Caller) -----
int TerminalDriver(void *arg) {
  int unit = (int)(long)arg;
  int status;

  while (1) {
    int result = MboxRecv(termInterruptMailbox[unit], &status, sizeof(int));
    if (result != sizeof(int)) {
      //  printf("TerminalDriver[unit %d]: MboxRecv error or wrong size! Result
      //  = "
      //         "%d\n",
      //         unit, result);
      continue;
    }
    // termHandler prints status, no need to repeat here unless debugging
    // handler itself printf("TerminalDriver[unit %d]: Received status =
    // 0x%x\n", unit, status);

    int xmit_status = USLOSS_TERM_STAT_XMIT(status);
    // Use this print for debugging the XMIT check result
    // printf("TerminalDriver[unit %d]: USLOSS_TERM_STAT_XMIT(status) = %d
    // (0=Ready)\n", unit, xmit_status);

    // --- Transmit Logic ---
    if (xmit_status == USLOSS_DEV_READY) {
      // printf("TerminalDriver[unit %d]: Entered XMIT path check\n", unit);

      // Use semaphore to protect check/update of write state
      kernSemP(termWriteLock[unit]);
      int wasReady = writeReady[unit]; // Check if a write *was* active

      if (wasReady) {
        // printf("TerminalDriver[unit %d]: Write WAS active (writeReady=1)\n",
        // unit);
        if (writeIndex[unit] < writeLength[unit]) {
          // Send next character
          char ch = writeBuffer[unit][writeIndex[unit]++];
          kernSemV(termWriteLock[unit]); // Release lock before device call

          int ctrl = 0;
          ctrl = USLOSS_TERM_CTRL_CHAR(ctrl, ch);
          ctrl = USLOSS_TERM_CTRL_XMIT_INT(ctrl);
          ctrl = USLOSS_TERM_CTRL_RECV_INT(ctrl);
          ctrl |= USLOSS_TERM_CTRL_XMIT_CHAR(0);
          // printf("TerminalDriver[%d]: XMIT '%c' (0x%x), ctrl=0x%x\n", unit,
          // ch,
          //       ch, ctrl);
          int outPutRes =
              USLOSS_DeviceOutput(USLOSS_TERM_DEV, unit, (void *)(long)ctrl);
          // if (outPutRes != USLOSS_DEV_OK) {
          //   printf("TerminalDriver[%d]: !! USLOSS_DeviceOutput FAILED
          //   with "
          //          "result %d !!\n",
          //          unit, outPutRes);
          // }
        } else {
          // Write finished FOR THIS UNIT (index >= length and wasReady was
          // true)
          // printf("TerminalDriver[unit %d]: XMIT complete.\n", unit);
          int pidToUnblock = termWriterPid[unit]; // Get PID before clearing
          writeReady[unit] = 0;                   // Mark write as done
          termWriterPid[unit] = -1;               // Clear stored PID
          kernSemV(termWriteLock[unit]);          // Release lock

          // --- Unblock the waiting process ---
          if (pidToUnblock != -1) {
            //    printf("TerminalDriver[unit %d]: Unblocking process %d\n",
            //    unit,
            //           pidToUnblock);
            unblockProc(pidToUnblock);
          }
          int final_ctrl = 0;
          final_ctrl =
              USLOSS_TERM_CTRL_RECV_INT(final_ctrl); // Keep recv enabled
          final_ctrl = USLOSS_TERM_CTRL_XMIT_INT(
              final_ctrl); // Re-enable xmit for future writes
          USLOSS_DeviceOutput(USLOSS_TERM_DEV, unit, (void *)(long)final_ctrl);
          // --- REMOVED: MboxCondSend(writeDoneMailbox, ...) ---
        }
      } else {
        // Spurious transmit interrupt when no write was active for this unit
        kernSemV(termWriteLock[unit]); // Release lock
        // printf("TerminalDriver[unit %d]: Spurious XMIT interrupt
        // (writeReady=0)\n", unit);
      }
    } // end if(xmit_status == USLOSS_DEV_READY)

    // --- Receive Logic ---
    int recv_status = USLOSS_TERM_STAT_RECV(status);
    if (recv_status == USLOSS_DEV_BUSY) { // Check if character is available
                                          // (BUSY means char available)
      // printf("TerminalDriver[unit %d]: Entered RECV path\n", unit);
      char ch = USLOSS_TERM_STAT_CHAR(status);
      // printf("TerminalDriver[unit %d]: RECV '%c' (0x%x)\n", unit, ch, ch);

      if (lineLength[unit] < MAX_LINE_LENGTH) {
        lineBuffer[unit][lineLength[unit]++] = ch;
      }

      if (ch == '\n' || lineLength[unit] == MAX_LINE_LENGTH) {
        // printf("TerminalDriver[unit %d]: Line complete, sending %d bytes to
        // termReadMailbox[%d]\n", unit, lineLength[unit], unit);
        lineBuffer[unit][lineLength[unit]] = '\0';
        // Use MboxSend as TermRead blocks anyway
        int send_res =
            MboxSend(termReadMailbox[unit], lineBuffer[unit], lineLength[unit]);
        if (send_res < 0) { // MboxSend returns negative on error
          printf("TerminalDriver[unit %d]: MboxSend to termReadMailbox failed! "
                 "Result = %d\n",
                 unit, send_res);
        }
        lineLength[unit] = 0;
      }
      // Important: Re-enable receive interrupt (and xmit if needed) AFTER
      // processing char
      kernSemP(termWriteLock[unit]); // Need lock to check writeReady safely
      int needsXmitInt = writeReady[unit];
      kernSemV(termWriteLock[unit]);

      int ctrl = 0;
      ctrl = USLOSS_TERM_CTRL_RECV_INT(ctrl);
      if (needsXmitInt) {
        ctrl = USLOSS_TERM_CTRL_XMIT_INT(ctrl);
      }
      // printf("TerminalDriver[unit %d]: Re-arming interrupts after RECV, ctrl
      // = 0x%x\n", unit, ctrl);
      USLOSS_DeviceOutput(USLOSS_TERM_DEV, unit, (void *)(long)ctrl);
    } // end if (recv_status == USLOSS_DEV_BUSY)

  } // end while(1)
  return 0;
}

// int TerminalDriver(void *arg) {
//   int unit = (int)(long)arg;
//   int status;
//
//   while (1) {
//     int result = MboxRecv(termInterruptMailbox[unit], &status, sizeof(int));
//     if (result != sizeof(int)) {
//       continue;
//     }
//     printf("TerminalDriver[unit %d]: Received status = 0x%x\n", unit,
//     status);
//
//     int xmit_status = USLOSS_TERM_STAT_XMIT(status);
//     // DEBUG: Show result of XMIT check explicitly
//     printf("TerminalDriver[unit %d]: USLOSS_TERM_STAT_XMIT(status) = %d\n",
//            unit, xmit_status);
//     if (xmit_status == USLOSS_DEV_READY) {
//       printf("MADE it to XMIT\n");
//       if (writeReady[unit] && writeIndex[unit] < writeLength[unit]) {
//         printf("UNDER 2ND IF\n");
//         char ch = writeBuffer[unit][writeIndex[unit]++];
//         printf("TerminalDriver[%d]: XMIT '%c' (0x%x)\n", unit, ch, ch);
//
//         int ctrl = USLOSS_TERM_CTRL_CHAR(0, ch);
//         ctrl = USLOSS_TERM_CTRL_XMIT_INT(ctrl);
//         ctrl = USLOSS_TERM_CTRL_RECV_INT(ctrl);
//         ctrl |= USLOSS_TERM_CTRL_XMIT_CHAR(0);
//         printf("TerminalDriver[%d]: rearm ctrl=0x%04x before sending'%c'\n ",
//                unit, ctrl, ch);
//         USLOSS_DeviceOutput(USLOSS_TERM_DEV, unit, (void *)(long)ctrl);
//       } else {
//         printf("TerminalDriver[unit %d]: XMIT complete for this unit, "
//                "signaling writeDoneMailbox\n",
//                unit);
//         writeReady[unit] = 0;
//         MboxCondSend(writeDoneMailbox, NULL, 0);
//         // writeIndex[unit] = 0;
//         // writeLength[unit] = 0;
//         printf("  [DEBUG] TerminalDriver[%d]: all done, signaled "
//                "writeDoneMailbox\n",
//                unit);
//       }
//     }
//
//     if (USLOSS_TERM_STAT_RECV(status)) {
//       char ch = USLOSS_TERM_STAT_CHAR(status);
//       if (unit == 1)
//         // USLOSS_Console("TerminalDriver[%d]: RECV '%c' (0x%x)\n", unit, ch,
//         // ch);
//
//         if (lineLength[unit] < MAX_LINE_LENGTH) {
//           lineBuffer[unit][lineLength[unit]++] = ch;
//         }
//       if (ch == '\n' || lineLength[unit] == MAX_LINE_LENGTH) {
//         lineBuffer[unit][lineLength[unit]] = '\0';
//         MboxSend(termReadMailbox[unit], lineBuffer[unit], lineLength[unit]);
//         lineLength[unit] = 0;
//       }
//     }
//   }
//
//   return 0;
// }

// ----- termWriteHandler (Corrected for Blocking Caller) -----
void termWriteHandler(USLOSS_Sysargs *args) {
  char *buf = (char *)args->arg1;
  int len = (int)(long)args->arg2;
  int unit = (int)(long)args->arg3;
  int callingPid = getpid(); // Get PID of the process calling TermWrite

  // Validate arguments
  if (!buf || len < 0 || len > MAX_LINE_LENGTH || unit < 0 ||
      unit >= USLOSS_TERM_UNITS) {
    // Use printf for kernel debug messages if USLOSS_Console is problematic
    // printf("termWriteHandler[unit %d, pid %d]: Invalid arguments! len=%d\n",
    //        unit, callingPid, len);
    args->arg4 = (void *)(long)-1;
    return;
  }

  kernSemP(termWriteLock[unit]); // Protect shared state check/update

  // Check if driver is already busy with a write for this unit
  if (writeReady[unit]) {
    kernSemV(termWriteLock[unit]);
    printf("termWriteHandler[unit %d, pid %d]: Driver busy!\n", unit,
           callingPid);
    args->arg4 = (void *)(long)-1; // Return EBUSY or similar error
    return;
  }

  // Setup the write buffer for the driver
  memcpy(writeBuffer[unit], buf, len);
  writeLength[unit] = len;
  writeIndex[unit] = 0;
  writeReady[unit] = 1;             // Mark driver as busy for this unit
  termWriterPid[unit] = callingPid; // Store PID of waiting process

  kernSemV(termWriteLock[unit]); // Release lock before potentially blocking
                                 // device call

  if (writeLength[unit] > 0) {
    // Send the first character to start the process
    char ch = writeBuffer[unit][writeIndex[unit]++];
    int ctrl = 0;
    ctrl = USLOSS_TERM_CTRL_CHAR(ctrl, ch);
    ctrl = USLOSS_TERM_CTRL_XMIT_INT(ctrl);
    ctrl = USLOSS_TERM_CTRL_RECV_INT(ctrl);
    ctrl |= USLOSS_TERM_CTRL_XMIT_CHAR(0);

    // printf("termWriteHandler[unit %d, pid %d]: Sending FIRST char '%c'
    // (0x%x), "
    //        "ctrl = 0x%x, len = %d\n",
    //        unit, callingPid, ch, ch, ctrl, len);
    USLOSS_DeviceOutput(USLOSS_TERM_DEV, unit, (void *)(long)ctrl);

    // --- Block the calling process ---
    // printf("termWriteHandler[unit %d, pid %d]: Blocking process...\n", unit,
    //        callingPid);
    blockMe(); // Block until driver unblocks us
    // printf("termWriteHandler[unit %d, pid %d]: Unblocked by driver.\n", unit,
    //        callingPid);
    // ---

  } else {
    // Zero-length write completes immediately, no need to block
    kernSemP(termWriteLock[unit]); // Re-acquire lock to safely change state
    writeReady[unit] = 0;
    termWriterPid[unit] = -1;
    kernSemV(termWriteLock[unit]);
    // printf("termWriteHandler[unit %d, pid %d]: Zero length write
    // completed.\n",
    //        unit, callingPid);
  }

  // Set return values AFTER being unblocked (or for zero-length write)
  args->arg2 =
      (void *)(long)len; // Return the number of bytes written (or requested)
  args->arg4 = (void *)(long)0; // Return success
  // printf("termWriteHandler[unit %d, pid %d]: System call returning.\n", unit,
  //        callingPid);
}

// void termWriteHandler(USLOSS_Sysargs *args) {
//   char *buf = (char *)args->arg1;
//   int len = (int)(long)args->arg2;
//   int unit = (int)(long)args->arg3;
//
//   // printf("termWriteHandler(): entered, unit=%d, len=%d\n", unit, len);
//   if (!buf || len < 0 || unit < 0 || unit > 3) {
//     args->arg4 = (void *)(long)-1;
//     return;
//   }
//
//   kernSemP(termWriteLock[unit]);
//
//   memcpy(writeBuffer[unit], buf, len);
//   writeLength[unit] = len;
//   writeIndex[unit] = 0;
//   writeReady[unit] = 1;
//
//   char ch = writeBuffer[unit][writeIndex[unit]++];
//   int ctrl = USLOSS_TERM_CTRL_CHAR(0, ch);
//   ctrl = USLOSS_TERM_CTRL_XMIT_INT(ctrl);
//   ctrl = USLOSS_TERM_CTRL_RECV_INT(ctrl);
//   ctrl |= USLOSS_TERM_CTRL_XMIT_CHAR(0);
//
//   printf("termWriteHandler[unit %d]: Sending FIRST char '%c' (0x%x), ctrl = "
//          "0x%x, len = %d\n",
//          unit, ch, ch, ctrl, len);
//   USLOSS_DeviceOutput(USLOSS_TERM_DEV, unit, (void *)(long)ctrl);
//
//   //  printf("termWriteHandler(): write request queued for unit %d\n", unit);
//   //  printf("termWriteHandler: unit %d first-char ctrl=0x%04x\n", unit,
//   ctrl);
//   // MboxRecv(readDoneMailbox, NULL, 0);
//   // printf("  [DEBUG] termWriteHandler: reader done\n");
//   //
//   // printf("termWriteHandler[unit %d]: Blocking on writeDoneMailbox...\n",
//   // unit); MboxRecv(writeDoneMailbox, NULL, 0);
//   printf("termWriteHandler[unit
//   // %d]: Unblocked from writeDoneMailbox\n", unit);
//
//   kernSemV(termWriteLock[unit]);
//
//   args->arg2 = (void *)(long)len;
//   args->arg4 = (void *)(long)0;
//   // ✅ signal write complete
//
//   // printf("termWriteHandler(): initial char sent '%c'\n",
//   // writeBuffer[unit][0]);
// }

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
  int wakeupTime = clockTicks + seconds * 10;
  addSleepRequest(pid, wakeupTime);
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

void checkWakeups() {
  while (sleepQueue && sleepQueue->wakeupTime <= clockTicks) {
    SleepRequest *tmp = sleepQueue;
    sleepQueue = sleepQueue->next;
    unblockProc(tmp->pid);
    free(tmp);
  }
}
