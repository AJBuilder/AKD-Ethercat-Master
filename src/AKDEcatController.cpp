//////////////////////////// Timing ////////////////////////////
#define NSEC_PER_SEC 1000000000
#define EC_TIMEOUTMON 500

//////////////////// PDO Assignment Objects ////////////////////
#define rxPDOEnable 0x1C12, 0
#define rxPDOAssign 0x1C12

#define txPDOEnable 0x1C13, 0
#define txPDOAssign 0x1C13

#define rxFlexPDO1   0x1600
#define rxFlexPDO2   0x1601
#define rxFlexPDO3   0x1602
#define rxFlexPDO4   0x1603

#define txFlexPDO1   0x1a00
#define txFlexPDO2   0x1a01
#define txFlexPDO3   0x1a02
#define txFlexPDO4   0x1a03

///////////////// Control and Status Objects //////////////////
#define COControl    0x6040
#define MnfStatus    0x1002
#define COStatus     0x6041

#define QSTOP_OPT    0x605A, 0
#define REQOPMODE    0x6060, 0
#define ACTOPMODE    0x6061, 0
#define MOTOR_RATIO  0x6091, 1
#define SHAFT_RATIO  0x6091, 2
#define DIG_OUT_MASK 0x60FE, 2
#define DIG_OUT1_MODE 0x35AF, 0
#define DIG_OUT2_MODE 0x35B2, 0

/////////////////////// Homing Objects ///////////////////////
#define HM_MODE          0x50CB, 0
#define HM_DIR           0x50C5, 0 
#define HM_SET           0x35F0, 0 // Start home
#define HM_ACC           0x3502, 0
#define HM_DEC           0x3524, 0 
#define HMACCEL          0x609A, 0 // Bidirectional acceleration
#define HM_AUTOMOVE      0x36D7, 0 // Automove after startup
#define HM_DIST          0x3484, 0 // Aditional move after home
#define HM_P             0x607C, 0 // Encoder offset after home

// Homeing Mode : Limitswitch
#define HM_V             0x6099, 1

// Homing Mode : Position
#define HM_FEEDRATE      0x6099, 2
#define HM_TPOSWND       0x5406, 0

// Homing Mode : Hardstop
#define HM_IPEAKACTIVE   0x5403, 0
#define HM_IPEAK         0x35E2, 0
#define HM_PERRTHRESH    0x3482, 0

/////////////////////// Profile Control Objects ///////////////////////
#define MT_V            0x6081, 0
#define MT_ACC          0x6083, 0
#define MT_DEC          0x6084, 0
#define PF_MAXCURRENT   0x6073, 0

// CoE States (0x6041)
#define NOTRDY2SWCH  0b00000000
#define SWCHDISABLED 0b01000000
#define RDY2SWITCH   0b00100001
#define SWITCHEDON   0b00100011
#define OP_ENABLED   0b00100111
#define FAULT        0b00001000
#define FAULTREACT   0b00001111
#define QUICKSTOP    0b00000111

// CoE Control (0x6040)
#define CTRL_SHUTDOWN_bit   0b0001
#define CTRL_DIS_VOL_bit    0b0010
#define CTRL_QSTOP_bit      0b0100
#define CTRL_ENABLED_bit    0b1000
#define CTRL_FAULTRESET_bit 0b10000000

////// Config //////
//Talker
#define CYCLE_NS (int)(16*62500) //1ms
#define MASTER_SYNCLEAD (300*1000) // .3ms
#define MASTER_WINDOW_NS (50*1000) //.05ms
#define MASTER_AQTIME_NS (100*1000*1000) // 100ms
#define SYNC_DIST (100*1000) // .1ms

//Controller
#define CNTRL_CYCLEMS   500

#define UNINIT_OPMODE   0
#define UNINIT_RXPDO    0
#define UNINIT_TXPDO    0
#define DEFAULT_QSCODE  6
#define DEFAULT_MOVIMM  FALSE
#define DEFAULT_MOVREL  FALSE
#define DEFAULT_DIGOUT  0
#define DEFAULT_HMAUTOMOVE 0

// Operation
#define REALTIME    TRUE

////////////////////// Error Codes //////////////////////

// Generic (errno.h)
#define ECAT_ETIMEDOUT 110

// Update()
#define AKD_MOVEERR -1

#include <sys/syscall.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include <inttypes.h>
#include <string>
#include <cstring>
#include <vector>

#include <pthread.h>
#include <sched.h>
#include <limits.h>
#include <sys/mman.h>

#include "AKDEcatController.h"
#include "soem/ethercat.h"



void add_timespec(struct timespec *ts, int64 addns)
{
   int64 sec, nsec;

   nsec = addns % NSEC_PER_SEC;
   sec = (addns - nsec) / NSEC_PER_SEC;
   ts->tv_sec += sec;
   ts->tv_nsec += nsec;
   if ( ts->tv_nsec >= NSEC_PER_SEC )
   {
      nsec = ts->tv_nsec % NSEC_PER_SEC;
      ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
      ts->tv_nsec = nsec;
   }
}

/** PI control calculation for cycle time delay offset
 * @function   ec_sync
 * @abstract               PI control calculation for cycle time delay offset
 * @param      reftime     The current DC time
 * @param      cycletime   The target cycle time
 * @param      offsettime  Pointer to the variable that will hold the calculated delay offset
 * @param      dist        The sync point after cycle period has elapsed. (ie constant delay)
 * @param      window      The window the time must be within to return TRUE
 * @param      d           Pointer to variable that will hold the calculated delta. For debug
 * @param      i           Pointer to variable that will hold the current integral. For debug
 * @result                 Is TRUE when timing is withing given window.
*/
 bool AKDController::ec_sync(int64 reftime, uint64 cycletime , int64 *offsettime, int64 dist, int64 window, int64 *d, int64 *i)
{
   static int64 integral = 0;
   int64 delta;
   delta = (reftime - dist) % cycletime;
   if(delta > (cycletime / 2)) { delta = delta - cycletime; } // If over half-period, make neg offset behrend next period
   if(delta>0){ integral--; }
   if(delta<0){ integral++; }
   *offsettime = -(delta / 20) + (integral / 5);
   *d = delta;
   *i = integral;
   return (*d < window) && (*d > (-window)) ; // Return TRUE if error is within window
}

#if AKD_ECAT_DEBUG_MODE

int AKDController::getLockedMem(){

   char dir[20];
   char *line=NULL;
   size_t n=0;

   sprintf(dir, "/proc/%d/status", getpid());
   FILE* file = fopen(dir, "r");
   if(!file) return -1;

   while(getline(&line, &n, file) != -1){
      if(strstr(line, "VmLck:") != NULL){
         line[15] = '\n';
         line += 7;
         if(fclose(file) == 0) return atoi(line);
         else return -3;
      }
   }
   fclose(file);
   return -2;
}

/** Adds string to a ringbuffer that will be printed by calling printDebugBuff(). Will overwrite if ring buffer overflows
 * @function   addToDebugBuff
 * @abstract         Adds string to a buffer that will be printed by calling printDebugBuff(). Will overwrite if ring buffer overflows
 * @param      str   Pointer to string to be added.
 * @see AKDController::printDebugBuff
*/
void AKDController::addToDebugBuff(char *str){

   strcpy(debugBuffer[buffTail], str);

   buffTail++; // Inc to next free index
   if(buffTail >= DEBUG_BUFF_SIZE) buffTail = 0; // Loop around if necessary

   if(buffTail == buffHead) buffHead++; // If overflowing, inc head to next oldest data
   if(buffHead >= DEBUG_BUFF_SIZE) buffHead = 0; // Loop around if necessary

   return;
}

/** Prints strings in ringbuffer added with addToDebugBuff(). Only prints the whole buffer once. (ie does not keeping printing if items are added faster)
 * @function   printDebugBuff
 * @abstract         Prints strings in ringbuffer added with addToDebugBuff(). Only prints the whole buffer once. (ie does not keeping printing if items are added faster)
 * @see AKDController::addToDebugBuff
*/
void AKDController::printDebugBuff(){

   int i = 1;
   while(buffTail != buffHead && i < DEBUG_BUFF_SIZE){
      printf("%s",debugBuffer[buffHead]);
      buffHead++;
      if(buffHead >= DEBUG_BUFF_SIZE) buffHead = 0;
      i++;
   }

   return;
}

/** Converts CoE status word to a readable string.
 * @function   getReadableStatus
 * @abstract            Converts CoE status word to a readable string.
 * @param      status   Status word to be converted.
 * @result              Pointer to string holding the readable status.
*/
char* AKDController::getReadableStatus(uint16 status){

   switch (status & 0b01101111){
      case QUICKSTOP: return (this->coeStatusReadable[7]);
      default:
      switch (status & 0b01001111){
         case NOTRDY2SWCH  & 0b01001111   :  return this->coeStatusReadable[0];
         case SWCHDISABLED & 0b01001111   :  return this->coeStatusReadable[1];
         case RDY2SWITCH   & 0b01001111   :  return this->coeStatusReadable[2];
         case SWITCHEDON   & 0b01001111   :  return this->coeStatusReadable[3];
         case OP_ENABLED   & 0b01001111   :  return this->coeStatusReadable[4];
         case FAULT        & 0b01001111   :  return this->coeStatusReadable[5];
         case FAULTREACT   & 0b01001111   :  return this->coeStatusReadable[6];
         default: return this->coeStatusReadable[8];
      }
   }

}

/** Converts CoE control word to a readable string.
 * @function   getReadableCtrl
 * @abstract            Converts CoE control word to a readable string.
 * @param      ctrl     Control word to be converted.
 * @result              Pointer to string holding the readable control word.
*/
char* AKDController::getReadableCtrl(uint16 ctrl){
   static bool wasEnabled = FALSE;
   wasEnabled = wasEnabled;

   if((ctrl & CTRL_FAULTRESET_bit) == TRUE )   {return this->coeCtrlReadable[6];}
   if((ctrl & CTRL_DIS_VOL_bit)    == FALSE)   {return this->coeCtrlReadable[2];}
   if((ctrl & CTRL_QSTOP_bit)      == FALSE)   {return this->coeCtrlReadable[3];}
   if((ctrl & CTRL_SHUTDOWN_bit)   == FALSE)   {return this->coeCtrlReadable[0];}
   if((ctrl & CTRL_ENABLED_bit)    == FALSE){
      if(wasEnabled) {return this->coeCtrlReadable[4];}
      else {return this->coeCtrlReadable[1];}
   }else {
      wasEnabled = TRUE;
      return this->coeCtrlReadable[5];
   }

}

#endif

/** Thread that handles PDO communication. Coordinates data exchange between buffer and IO to SOEM library. Called by ecat_Start().
 * @function   ecat_Talker
 * @abstract            Thread that handles PDO communication. Coordinates data exchange between buffer and IO to SOEM library. Called by ecat_Start().
 * @param      THIS     Pointer to master object. 
 * @result              NULL
 * @see        AKDController::ecat_Init
*/
void* AKDController::ecat_Talker(void* THIS)
{
   AKDController* This = (AKDController*)THIS;

   // Local variables
   struct timespec   cycle_ts, control1_ts, control2_ts, controlOpTime_ts;
   uint64 prevDCtime, controlOpTime;
   uint8*   output_map_ptr;
   uint8*   input_map_ptr;
   uint8*   output_buff_ptr;
   uint8*   input_buff_ptr;
   bool moveQueued[This->slaveCount], newTarget[This->slaveCount] = {FALSE};
   #if AKD_ECAT_DEBUG_MODE
      char buffer[DEBUG_BUFF_WIDTH];
      uint16 prevStatus[This->slaveCount], prevCtrl[This->slaveCount];
   #endif

   // Buffers (To outside of thread)
   int8 wrkCounterb;
   uint inSyncCountb;
   bool targetReachedb[This->slaveCount] = {FALSE};
   int64 toff, sync_delta, sync_integral, diffDCtimeb;

   #if AKD_ECAT_DEBUG_MODE
      printf("\nLocked memory in talker(PID: %ld): %dkB\n", syscall(__NR_gettid), This->getLockedMem());
      printf("\nTalker running on CPU %d\n", sched_getcpu());
   #endif

   clock_gettime(CLOCK_MONOTONIC, &cycle_ts);
   ec_send_processdata();
   while(1)
   {
      wrkCounterb = ec_receive_processdata(EC_TIMEOUTRET);
      
      /* calulate toff to get linux time and DC synced */
      if(This->ec_sync(ec_DCtime, CYCLE_NS, &toff, MASTER_SYNCLEAD + SYNC_DIST, MASTER_WINDOW_NS, &sync_delta, &sync_integral)){ // If withing sync window
         if (inSyncCountb != UINT32_MAX) inSyncCountb++; // Count number of cycles in window
      }
      else inSyncCountb = 0;
      
      /* calculate next cycle start */
      add_timespec(&cycle_ts, CYCLE_NS+ toff);

      /* wait to cycle start */
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &cycle_ts, NULL);

      if(pthread_mutex_trylock(&This->control) == 0){
         // CONTROL LOCKED
         clock_gettime(CLOCK_MONOTONIC, &control1_ts); //Record time it takes to operate with control lock.
         
         if(This->masterState == ecat_masterStates::ms_shutdown) {
            pthread_mutex_unlock(&This->control);
            break;
         }
         This->inSyncCount = inSyncCountb;
         This->wrkCounter = wrkCounterb;
         This->diffDCtime = ec_DCtime - prevDCtime;
         prevDCtime = ec_DCtime;
         
         for(int slaveNum = 0 ; slaveNum < This->slaveCount ; slaveNum++){
            
            This->slaves[slaveNum].targetReached = targetReachedb[slaveNum];

            if(This->slaves[slaveNum].update){        

               if(This->slaves[slaveNum].coeCtrlWord & (1 << 4)) {
                  moveQueued[slaveNum] = true; // Make sure that a rising edge of setpoint ack bit is detected
                  newTarget[slaveNum] = true; // Make sure the a rising edge of target reached bit is detected
                  targetReachedb[slaveNum] = false; // Reset for a new target
               }
               
               output_buff_ptr = This->slaves[slaveNum].outUserBuff;
               input_buff_ptr = This->slaves[slaveNum].inUserBuff;
               output_map_ptr = ec_slave[slaveNum+1].outputs;
               input_map_ptr = ec_slave[slaveNum+1].inputs;

               // Update outputs from user buffers to IOmap
               // Copy data from user's input to IOmap.
               memcpy(output_map_ptr, output_buff_ptr, This->slaves[slaveNum].rxPDO.bytes);

               // Update inputs from user buffers to IOmap
               memcpy(input_buff_ptr, input_map_ptr, This->slaves[slaveNum].txPDO.bytes);

               // Update target flag
               This->slaves[slaveNum].targetReached = targetReachedb[slaveNum];

               // Reset update flag
               This->slaves[slaveNum].update = false;
            }
            else {
               if(This->slaves[slaveNum].mode == ecat_OpModes::profPos || This->slaves[slaveNum].mode == ecat_OpModes::homing){ // If in ProfPos or Homing only
                  This->slaves[slaveNum].coeCtrlWord &= ~0b0010000; // Clear move bit
                  This->slaves[slaveNum].coeCtrlWord |= *This->slaves[slaveNum].coeCtrlMapPtr & (1 << 4); // Copy move bit from talker controled IOmap.
               }
            }
            // Overwrite CoE control and status words with controllers data.
            memcpy(This->slaves[slaveNum].coeCtrlMapPtr, &This->slaves[slaveNum].coeCtrlWord, sizeof(This->slaves[slaveNum].coeCtrlWord));
            memcpy(&This->slaves[slaveNum].coeStatus, This->slaves[slaveNum].coeStatusMapPtr, sizeof(This->slaves[slaveNum].coeStatus));
            
         }

         // Signal that IOmap has been updated (Possibly user buff)
         // Called before unlocking control so that whatever is waiting
         // on the signal has priority. (Pretty sure this is how it works...)
         pthread_cond_signal(&This->IOUpdated);
         

         pthread_mutex_unlock(&This->control);
         // CONTROL UNLOCKED

         clock_gettime(CLOCK_MONOTONIC, &control2_ts); //Record time it takes to operate with control lock.
         controlOpTime = ((control2_ts.tv_sec - control1_ts.tv_sec) * NSEC_PER_SEC) + (control2_ts.tv_nsec - control1_ts.tv_nsec);
      }else{
         // Possibly useless
         controlOpTime_ts.tv_sec = 0;
         controlOpTime_ts.tv_nsec = 0;
         add_timespec(&controlOpTime_ts, controlOpTime);
         clock_nanosleep(CLOCK_MONOTONIC, 0, &controlOpTime_ts, NULL);
      }

      // Detect rising edges of flags
      for(int slaveNum = 0; slaveNum < This->slaveCount; slaveNum++){
         if(moveQueued[slaveNum]){
            if(!(*This->slaves[slaveNum].coeStatusMapPtr & (1 << 12))){ // If buffering setpoint...
               moveQueued[slaveNum] = FALSE; // Wait for setpoint ack
            }
         }
         else if(*This->slaves[slaveNum].coeStatusMapPtr & (1 << 12)){ // If no longer buffering, and setpoint ack
            *This->slaves[slaveNum].coeCtrlMapPtr &= ~(1 << 4); // Clear move bit. So there are no double setpoints
         }    

         if(newTarget[slaveNum]){
            if(!(*This->slaves[slaveNum].coeStatusMapPtr & (1 << 10))){ // If target is not reached...
               newTarget[slaveNum] = FALSE; // Wait for it to be reached
            }
         } else if(*This->slaves[slaveNum].coeStatusMapPtr & (1 << 10)){
            targetReachedb[slaveNum] = true;
         }
      }


      #if AKD_ECAT_DEBUG_MODE
      // Update debug info
      if(pthread_mutex_lock(&This->debug) == 0){
         This->gl_toff = toff;
         This->gl_delta = sync_delta;
         This->gl_integral = sync_integral;

         for(int slaveNum = 0; slaveNum < This->slaveCount; slaveNum++){
            //printf("\n[Slave %i] CoE State: (0x%04x) %-24s \tCoE Control:(0x%04x) %-24s \n", slaveNum + 1, *This->slaves[slaveNum].coeStatusMapPtr, This->getReadableStatus(*This->slaves[slaveNum].coeStatusMapPtr), *This->slaves[slaveNum].coeCtrlMapPtr, This->getReadableCtrl(*This->slaves[slaveNum].coeCtrlMapPtr));
            if((prevStatus[slaveNum] != *This->slaves[slaveNum].coeStatusMapPtr) || (prevCtrl[slaveNum] != *This->slaves[slaveNum].coeCtrlMapPtr)){
               sprintf(buffer, "\n[Slave %i] CoE State: (0x%04x) %-24s \tCoE Control:(0x%04x) %-24s \n", slaveNum + 1, *This->slaves[slaveNum].coeStatusMapPtr, This->getReadableStatus(*This->slaves[slaveNum].coeStatusMapPtr), *This->slaves[slaveNum].coeCtrlMapPtr, This->getReadableCtrl(*This->slaves[slaveNum].coeCtrlMapPtr));
               This->addToDebugBuff(buffer);
               prevStatus[slaveNum] = *This->slaves[slaveNum].coeStatusMapPtr;
               prevCtrl[slaveNum] = *This->slaves[slaveNum].coeCtrlMapPtr;
            }
         }
         pthread_mutex_unlock(&This->debug);
      }

      #endif

      ec_send_processdata();
   }
   return nullptr;
}

/** Thread that handles the CoE state machine and EtherCAT state machine. Called by ecat_Start().
 * @function   ecat_Controller
 * @abstract            Thread that handles the CoE state machine. Called by ecat_Start().
 * @param      THIS     Pointer to master object. 
 * @result              NULL
 * @see        AKDController::ecat_Start
*/
void* AKDController::ecat_Controller(void* THIS)
{
   printf("ECAT: Controller Spawned\n");  

   // Local variables
   AKDController* This = (AKDController*)THIS;
   uint currentgroup = 0, noDCCount;
   
   #if AKD_ECAT_DEBUG_MODE
      printf("\nLocked memory in controller(PID: %ld): %dkB\n", syscall(__NR_gettid), This->getLockedMem());
      printf("\nController running on CPU %d\n", sched_getcpu());
   #endif

   while(1)
   {
      // CONTROL LOCKED
      pthread_mutex_lock(&This->control);
      if(This->masterState == ecat_masterStates::ms_shutdown){
         pthread_mutex_unlock(&This->control);  
         break;
      }
      if(This->diffDCtime == 0) noDCCount++;
      else noDCCount = 0;
      if(noDCCount > 20) This->masterState = ecat_masterStates::ms_stop;


      for(int slaveNum = 0; slaveNum < This->slaveCount; slaveNum++){

         // CANopen state machine
         switch(This->slaves[slaveNum].coeStatus & 0b01101111){
            case QUICKSTOP : break;
            default :
            {
               switch(This->slaves[slaveNum].coeStatus & 0b01001111){
                  case NOTRDY2SWCH  & 0b01001111: 
                  case SWCHDISABLED & 0b01001111:
                     if(This->masterState != ecat_masterStates::ms_shutdown)This->slaves[slaveNum].coeCtrlWord = (This->slaves[slaveNum].coeCtrlWord & ~0b00000111) | 0b0110; break; // Ready2Switch : 0bx---x110
                     break;
                  case RDY2SWITCH   & 0b01001111:
                  case SWITCHEDON   & 0b01001111:
                  case OP_ENABLED   & 0b11011111:
                     switch(This->masterState){
                        default :
                        case ecat_masterStates::ms_shutdown :  This->slaves[slaveNum].coeCtrlWord = (This->slaves[slaveNum].coeCtrlWord & ~0b00000111);          break; // SwitchOnDisabled : 0bx---xx0x
                        case ecat_masterStates::ms_stop     :  This->slaves[slaveNum].coeCtrlWord = (This->slaves[slaveNum].coeCtrlWord & ~0b00000111) | 0b0110; break; // Ready2Switch : 0bx---x110
                        case ecat_masterStates::ms_disable  :  This->slaves[slaveNum].coeCtrlWord = (This->slaves[slaveNum].coeCtrlWord & ~0b00001111) | 0b0111; break; // SwitchedOn/OpDisable : 0bx---0111
                        case ecat_masterStates::ms_enable   :  This->slaves[slaveNum].coeCtrlWord = (This->slaves[slaveNum].coeCtrlWord & ~0b00001111) | 0b1111; break; // OpEnabled : 0bx---1111
                     }
                     break;
                  case FAULT : This->slaves[slaveNum].coeCtrlWord = (This->slaves[slaveNum].coeCtrlWord & ~0b00001111) | 0b0111; // SwitchedOn/OpDisable : 0bx---0111
                  case FAULTREACT : break;
                  default : This->slaves[slaveNum].coeCtrlWord = (This->slaves[slaveNum].coeCtrlWord & ~0b00000111) | 0b0110; // Ready2Switch : 0bx---x110
                  
               }
            }
         }
      }

      if(This->inOP && (This->wrkCounter < This->expectedWKC) || ec_group[currentgroup].docheckstate)
         {
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (int slave = 1; slave <= This->slaveCount; slave++)
            {
               if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
               {
                  ec_group[currentgroup].docheckstate = TRUE;
                  if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                  {
                     printf("\nECAT: (ERROR) slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                     ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                     printf("\nECAT: (WARNING) slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                     ec_slave[slave].state = EC_STATE_OPERATIONAL;
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state > EC_STATE_NONE)
                  {
                     if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("\nECAT: (MESSAGE) slave %d reconfigured\n",slave);
                     }
                  }
                  else if(!ec_slave[slave].islost)
                  {
                     /* re-check state */
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (ec_slave[slave].state == EC_STATE_NONE)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("\nECAT: (ERROR) slave %d lost\n",slave);
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(ec_slave[slave].state == EC_STATE_NONE)
                  {
                     if (ec_recover_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("\nECAT: (MESSAGE) slave %d recovered\n",slave);
                     }
                  }
                  else
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("\nECAT: (MESSAGE) slave %d found\n",slave);
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
               printf("\nECAT: (OK) all slaves resumed OPERATIONAL.\n");
         }
        
      
      
      #if AKD_ECAT_DEBUG_MODE
      pthread_mutex_lock(&This->debug);

      This->printDebugBuff();
      
      printf("\r");
      printf("WKC: %2i(%2i)\t", This->wrkCounter, This->expectedWKC);
      printf("DCErr: %12" PRIi64 "\t", CYCLE_NS - This->diffDCtime);
      printf("inSyncCount: %4i\t", This->inSyncCount);
      printf("toff: %8" PRIi64 "\t", This->gl_toff);
      printf("delta: %8" PRIi64 "\t", This->gl_delta);
      printf("integral: %8" PRIi64 "\t", This->gl_integral);
      //printf("coeStatus: 0x%04"PRIx16"\t", This->coeStatus);
      //printf("CtrlWord: 0x%04"PRIx16"\t", This->coeCtrlWord);
      
      if(((CYCLE_NS - This->diffDCtime)*(((CYCLE_NS - This->diffDCtime) > 0) - ((CYCLE_NS - This->diffDCtime) < 0))) > (CYCLE_NS / 2)) 
         printf("\tClock slipping! DCErr = %12" PRIi64 "  CYCLE_NS/2 = %d\n", ((CYCLE_NS - This->diffDCtime)*(((CYCLE_NS - This->diffDCtime) > 0) - ((CYCLE_NS - This->diffDCtime) < 0))), CYCLE_NS / 2);
      pthread_mutex_unlock(&This->debug);
      fflush(stdout);
      #endif

      pthread_mutex_unlock(&This->control);
      // CONTROL UNLOCKED

      pthread_cond_signal(&This->stateUpdated);

      osal_usleep(100000); //100ms
      
   }
   
   return nullptr;
}

/** Initializes master communication and variables.
 * @function   ecat_Init
 * @abstract            Initializes master communication and variables.
 * @param      ifname   Pointer to string holding the name of the ethernet interface to initialize on.
 * @result              Returns TRUE if successful
*/
bool AKDController::ecat_Init(const char* ifname){

   if(this->masterState != ecat_masterStates::ms_shutdown){
      printf("ECAT: Already initialized. Shutdown before restarting.\n");
      return FALSE;
   }

   if (ec_init(ifname))
   {
      printf("ECAT: ec_init on %s succeeded.\n", ifname);
      this->ifname = (char*)ifname;

      /* find and auto-config slaves */
      this->expectedWKC = ec_config_init(FALSE);
      if(this->expectedWKC > 0) {
         this->masterState = ecat_masterStates::ms_stop;

         printf("ECAT: Putting slaves into PRE_OP state... ");
         ec_slave[0].state = EC_STATE_PRE_OP;
         ec_writestate(0);
         if(ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE * 4) == EC_STATE_PRE_OP)
            printf( "State: PRE_OP\n");
         else{
            printf("ERR! NOT IN PRE_OP!\n");
            this->masterState = ecat_masterStates::ms_shutdown;
            ec_close();
            return false;
         }

         // Allocate slave data
         this->slaveCount = ec_slavecount;
         this->slaves = new ecat_slave [this->slaveCount];

         //this->expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;


         // Initialize threading resources
         struct sched_param rt_param;
         rt_param.sched_priority = 99; // RT priority

         cpu_set_t cpuset;
         CPU_ZERO(&cpuset);
         CPU_SET(7, &cpuset); // Run on cpu 8

         int err = 0;
         err += pthread_attr_init(&this->rt_attr);
         err += pthread_attr_setinheritsched(&this->rt_attr, PTHREAD_EXPLICIT_SCHED);
         err += pthread_attr_setschedpolicy(&this->rt_attr, SCHED_FIFO);
         err += pthread_attr_setschedparam(&this->rt_attr, &rt_param);
         err += pthread_attr_setaffinity_np(&this->rt_attr, sizeof(cpuset), &cpuset);

         printf("\nMain running on CPU %d\n", sched_getcpu());

         if(err != 0){
            this->masterState = ecat_masterStates::ms_shutdown;
            delete this->slaves;
            
            ec_close(); // stop SOEM, close socket
         }

         err = 0;
         //err += mlockall(MCL_CURRENT | MCL_FUTURE);
         

         if(err != 0){
            this->masterState = ecat_masterStates::ms_shutdown;
            delete this->slaves;
            
            ec_close(); // stop SOEM, close socket
         }

         pthread_mutex_init(&this->control, NULL);
         pthread_mutex_init(&this->debug, NULL);

         pthread_cond_init(&this->IOUpdated, NULL);
         pthread_cond_init(&this->stateUpdated, NULL);


         return TRUE;
      }
      else
         printf("ECAT: No slaves found!\n");
   }
   else
      printf("ECAT: No socket connection on %s. Execute as root!\n", ifname);

      return FALSE;
}

/** Sets up and starts communication based on config set by other functions. (Starts ecat_Controller and ecat_Talker)
 * @function   ecat_Start
 * @abstract            Sets up and starts communication based on config set by other functions. (Starts ecat_Controller and ecat_Talker)
 * @result              Returns TRUE if successful.
 * @see AKDController::ecat_Controller
 * @see AKDController::ecat_Talker
*/
bool AKDController::ecat_Start(){

   printf("ECAT: Starting EtherCAT Master\n");

   uint32 sdoBuff;
   int sdoBuffSize;
   int sdosNotConfigured = 1, wkc;
   int PDOAssign, pdoObject, pdoCnt, entry, entryCnt, bytesTillCoE;
   struct ecat_slave::mappings_t *pdoMappings;
   

   if(this->masterState == ecat_masterStates::ms_shutdown){
      printf("ECAT: Not initialized. Call ecat_Init()\n");
      return FALSE;
   }

   for(int i = 0; i < this->slaveCount; i++){
      if(this->slaves[i].outUserBuff == nullptr){
         printf("ECAT: Not all slave PDOs configured.\n");
         return FALSE;
      }
   }

   // Configure PDO assignments
   printf("ECAT: %d slaves found. Configuring PDO assigments...\n",slaveCount);
   printf("ECAT: Mode: %i\n", ec_slave[1].state);

   for(int slaveNum = 1 ; slaveNum <= this->slaveCount ; slaveNum++){
      
      sdosNotConfigured = 1;
      for(int i = 1; (sdosNotConfigured != 0) && (i <= 10); i++){
         if(i != 1) osal_usleep(100000); // If looping, wait 100ms
         sdosNotConfigured = 0;

         //////// Write Config ////////

         // Assign PDOs

         //Disable
         sdoBuff = 0;
         ec_SDOwrite(slaveNum, rxPDOEnable, FALSE, 1, &sdoBuff, EC_TIMEOUTRXM); // Disable rxPDO
         ec_SDOwrite(slaveNum, txPDOEnable, FALSE, 1, &sdoBuff, EC_TIMEOUTRXM); // Disable txPDO

         //Write
         for(int j = 0 ; j < this->slaves[slaveNum - 1].rxPDO.numOfPDOs ; j++){
            sdoBuff = this->slaves[slaveNum - 1].rxPDO.mapObject[j];
            ec_SDOwrite(slaveNum, rxPDOAssign, j + 1, FALSE, 2, &sdoBuff, EC_TIMEOUTRXM); // Assign rxPDO1
         }
         for(int j = 0 ; j < this->slaves[slaveNum - 1].txPDO.numOfPDOs ; j++){
            sdoBuff = this->slaves[slaveNum - 1].txPDO.mapObject[j];
            ec_SDOwrite(slaveNum, txPDOAssign, j + 1, FALSE, 2, &sdoBuff, EC_TIMEOUTRXM); // Assign txPDO1
         }

         // Set num of pdos
         sdoBuff = this->slaves[slaveNum - 1].rxPDO.numOfPDOs;
         ec_SDOwrite(slaveNum, rxPDOEnable, FALSE, 1, &sdoBuff, EC_TIMEOUTRXM); // Enable rxPDO
         sdoBuff = this->slaves[slaveNum - 1].txPDO.numOfPDOs;
         ec_SDOwrite(slaveNum, txPDOEnable, FALSE, 1, &sdoBuff, EC_TIMEOUTRXM); // Enable txPDO

         // Configure Homing settings
         sdoBuff = DEFAULT_HMAUTOMOVE;
         ec_SDOwrite(slaveNum, HM_AUTOMOVE , FALSE, 1, &sdoBuff, EC_TIMEOUTRXM); // Disable automove

         // Configure QuickStop
         sdoBuff = DEFAULT_QSCODE;
         ec_SDOwrite(slaveNum, QSTOP_OPT , FALSE, 2, &sdoBuff, EC_TIMEOUTRXM); // Set Quickstop option

         //////// Verify Assignments ////////

         // Verify PDOs
         sdoBuffSize = 1;
         sdoBuff = 0;
         ec_SDOread(slaveNum, rxPDOEnable, FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTRXM); // Check rxPDO enabled
         if(sdoBuff != this->slaves[slaveNum - 1].rxPDO.numOfPDOs) sdosNotConfigured++;

         sdoBuffSize = 1;
         sdoBuff = 0;
         ec_SDOread(slaveNum, txPDOEnable, FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTRXM); // Check txPDO enabled
         if(sdoBuff != this->slaves[slaveNum - 1].txPDO.numOfPDOs) sdosNotConfigured++;

         for(int j = 0 ; j < this->slaves[slaveNum - 1].rxPDO.numOfPDOs ; j++){
            sdoBuffSize = 2;
            sdoBuff = 0;
            ec_SDOread(slaveNum, rxPDOAssign, j + 1, FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTRXM); // Read rxPDO1
            if(sdoBuff != this->slaves[slaveNum - 1].rxPDO.mapObject[j]) sdosNotConfigured++;
         }
         for(int j = 0 ; j < this->slaves[slaveNum - 1].txPDO.numOfPDOs ; j++){
            sdoBuffSize = 2;
            sdoBuff = 0;
            ec_SDOread(slaveNum, txPDOAssign, j + 1, FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTRXM); // Read txPDO1
            if(sdoBuff != this->slaves[slaveNum - 1].txPDO.mapObject[j]) sdosNotConfigured++;
         }

         // Verify Homing Config
         sdoBuffSize = 1;
         sdoBuff = 0;
         ec_SDOread(slaveNum, HM_AUTOMOVE , FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTRXM); // Check homing automove
         if(sdoBuff != DEFAULT_HMAUTOMOVE) sdosNotConfigured++;

         // Verify QuickStop Config
         sdoBuffSize = 2;
         sdoBuff = 0;
         ec_SDOread(slaveNum, QSTOP_OPT , FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTRXM); // Check txPDO enabled
         if(sdoBuff != DEFAULT_QSCODE) sdosNotConfigured++;

         if(sdosNotConfigured != 0){
            printf("\rECAT: PDO assignments failed to configure %2d time(s). Retrying...  ", i);
            fflush(stdout);
         }
            
      }


      if(sdosNotConfigured == 0) {
         printf("ECAT:PDO's configured!\n");
      }
      else {
         printf("PDO's failed to configure.\n");
         return FALSE;
      }

   }

   //ec_reconfig_slave(0, EC_TIMEOUTRET3); Repopulate ec_slave data with new PDO asssignment configuration. NOT WORKING???
   ecx_context.manualstatechange = 1; // Prevents putting slaves into SAFEOP when configuring IOmap.
   ec_config_map(&this->IOmap); // Set up IOmap.
   printf("ECAT: Mode: %i\n", ec_slave[1].state);
   
   // Check to make sure user buffer is sized appropriately. Find CoE control and status words. Finally, finish setting up user buffer.
    for(int slaveNum = 1 ; slaveNum <= this->slaveCount ; slaveNum++){
      this->slaves[slaveNum - 1].rxPDO.bytes = ec_slave[slaveNum].Obytes;
      this->slaves[slaveNum - 1].txPDO.bytes = ec_slave[slaveNum].Ibytes;

      if(this->slaves[slaveNum-1].totalBytes != (this->slaves[slaveNum - 1].rxPDO.bytes + this->slaves[slaveNum - 1].txPDO.bytes)){
         printf("ECAT: Input struct of slave %i is of incorrect size. Is %i. Needs to be %i.\n", slaveNum, slaves[slaveNum-1].totalBytes, (this->slaves[slaveNum - 1].rxPDO.bytes + this->slaves[slaveNum - 1].txPDO.bytes));
         return FALSE;
      }

      slaves[slaveNum-1].coeCtrlOffset= -1;
      slaves[slaveNum-1].coeStatusOffset = -1;

      pdoMappings = &slaves[slaveNum - 1].rxPDO;
      PDOAssign = rxPDOAssign;
      while(1){

         bytesTillCoE = 0;
         pdoMappings->bytes = 0;
         
         //Read PDO assign subindex 0 ( = number of PDO's)
         sdoBuffSize = 2; sdoBuff = 0;         
         ec_SDOread(slaveNum, PDOAssign, 0x00, FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTRXM);

         if (sdoBuff > 0)
         {
            // Check to see if read value matches user's specified
            if(sdoBuff != pdoMappings->numOfPDOs){
               printf("ECAT: Number of user specified PDOs(%i) does not match the number of read PDOs(%i). Aborting Start()\n", pdoCnt, sdoBuff);
               return FALSE;
            }

            // Read all PDO assignments
            for (int idxloop = 1; idxloop <= pdoMappings->numOfPDOs; idxloop++)
            {
               // Read PDO map object
               sdoBuffSize = 2; sdoBuff = 0;
               ec_SDOread(slaveNum, PDOAssign, (uint8)idxloop, FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTRXM);
               pdoObject = sdoBuff;

               
               if (pdoObject > 0)
               {
                  // Read number of entries mapped
                  sdoBuffSize = 2; sdoBuff = 0;
                  ec_SDOread(slaveNum, pdoObject, 0x00, FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTRXM);
                  entryCnt = sdoBuff;

                  // Read each entries size
                  for (int subidxloop = 1; subidxloop <= entryCnt; subidxloop++)
                  {
                     // Read object
                     sdoBuffSize = 4; sdoBuff = 0;
                     ec_SDOread(slaveNum, pdoObject, (uint8)subidxloop, FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTRXM);
                     
                     //extract bitlength of SDO
                     if(PDOAssign == rxPDOAssign){
                        if(((sdoBuff & 0xFFFF0000) >> 16) == COControl){
                           if(slaves[slaveNum-1].coeCtrlOffset!= -1){
                              printf("ECAT: Found multiple CoE control word mappings when reading Slave %i's mapping. Aborting Start().\n", slaveNum);
                              return FALSE;
                           }
                           slaves[slaveNum-1].coeCtrlOffset= bytesTillCoE;
                        }
                        else{
                           bytesTillCoE += (sdoBuff & 0xFF) / 8;
                        }
                     }
                     else{
                        if(((sdoBuff & 0xFFFF0000) >> 16) == COStatus){
                           if(slaves[slaveNum-1].coeStatusOffset != -1){
                              printf("ECAT: Found multiple CoE status mappings when reading Slave %i's mapping. Aborting Start().\n", slaveNum);
                              return FALSE;
                           }  
                           slaves[slaveNum-1].coeStatusOffset = bytesTillCoE;
                        }
                        else{
                           bytesTillCoE += (sdoBuff & 0xFF) / 8;
                        }
                     }
                     pdoMappings->bytes += (sdoBuff & 0xFF) / 8;
                  }
               }
               else{
                  printf("ECAT: Tried to read PDO mapping. Slave %i, assign 0x%x, PDO %i is zero? Aborting Start().\n", slaveNum, PDOAssign, idxloop);
                  return FALSE;
               }
            }
         }
         else{
            printf("ECAT: Tried to read PDO assignments. Slave %i, assign 0x%x is disabled? Aborting Start().\n", slaveNum, PDOAssign);
            return FALSE;
         }

         // Repeat but with txPDOs
         pdoMappings = &slaves[slaveNum - 1].txPDO;
         if(PDOAssign == txPDOAssign) break;
         PDOAssign = txPDOAssign;
         
      }
   
      
      // Find where the AKD CoE control word is in IOmap
      this->slaves[slaveNum - 1].coeCtrlMapPtr = (uint16*)(ec_slave[slaveNum].outputs + this->slaves[slaveNum - 1].coeCtrlOffset);

      // Find where the AKD CoE status word is in IOmap
      this->slaves[slaveNum - 1].coeStatusMapPtr = (uint16*)(ec_slave[slaveNum].inputs + this->slaves[slaveNum - 1].coeStatusOffset);

      // Finish Populating User Buffer PDO maps
      this->slaves[slaveNum - 1].inUserBuff = this->slaves[slaveNum - 1].outUserBuff + ec_slave[slaveNum].Obytes;

   }


   
   printf("ECAT: Mode: %i\n", ec_slave[1].state);

   // Config DC (In PREOP)
   printf("ECAT: Configuring DC. Should be in PRE-OP. ");

   if(ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE * 4) == EC_STATE_PRE_OP)
      printf( "State: PRE_OP\n");
   else{
      printf("ERR! NOT IN PRE_OP!\n");
      return false;
   }

   if(ec_configdc())
      printf("ECAT: Found slaves with DC.\n");
   else{
      printf("ECAT: Did not find slaves with DC.\n");
      return false;
   }
   ec_dcsync0(1, FALSE, CYCLE_NS, 0);
   
   
   
   // Syncing (In SAFEOP)
   printf("ECAT: Slaves mapped, state to SAFE_OP... ");
   ec_slave[0].state = EC_STATE_SAFE_OP;
   ec_writestate(0);
   if(ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4) == EC_STATE_SAFE_OP)
      printf( "State: SAFE_OP\n");
   else 
      printf("ERR! NOT IN SAFE_OP!\n");

   printf("ECAT: Calculated workcounter %d\n", this->expectedWKC);


   // Enable talker to handle slave PDOs in OP
   if(pthread_create(&this->talker, &this->rt_attr, &AKDController::ecat_Talker, this) != 0) {
      printf("ECAT: Couldn't start talker thread.\n");
      return FALSE;
   }
   if(pthread_create(&this->controller, NULL, &ecat_Controller, this) != 0) {
      printf("ECAT: Couldn't start controller thread.\n");
      this->masterState = ecat_masterStates::ms_shutdown; // Set shutdown state to signal talker thread to shutdown.
      pthread_join(this->talker, nullptr); // Pretty sure this isn't necessary?
      this->masterState = ecat_masterStates::ms_stop; // Reset to initialized state.
      return FALSE;
   }
   printf( "ECAT: Talker started! Synccount needs to reach : %" PRIi64 "\n", (uint64)MASTER_AQTIME_NS / CYCLE_NS);
   pthread_mutex_lock(&this->control);
   while(this->inSyncCount < (uint64)(MASTER_AQTIME_NS / CYCLE_NS)){ // Poll until timing is within acceptable window
      pthread_mutex_unlock(&this->control);
      osal_usleep(100000); // 100ms
      pthread_mutex_lock(&this->control);
   }
   pthread_mutex_unlock(&this->control);
   printf("\nECAT: Master within sync window. Enabling sync0 generation!\n");
   ec_dcsync0(1, TRUE, CYCLE_NS, 0); //Enable sync0 generation


   // Go into OPERATIONAL mode
   printf("\nECAT: Requesting operational state for all slaves\n");
   ec_slave[0].state = EC_STATE_OPERATIONAL;
   ec_send_processdata();  // Send one valid data to prep slaves for OP state
   ec_receive_processdata(EC_TIMEOUTRET);
   ec_writestate(0);
   ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE); //wait for OP


   // Return results
   if (ec_slave[0].state == EC_STATE_OPERATIONAL )
   {
      printf("\nECAT: Operational state reached for all slaves!\n");
      this->inOP = TRUE;
      return TRUE;
   }
   else
   {
      printf("\nECAT: Not all slaves reached operational state.\n");
      ec_readstate();
      for(int i = 1; i <= this->slaveCount ; i++)
      {
         if(ec_slave[i].state != EC_STATE_OPERATIONAL)
         {
            printf("ECAT: Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                  i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
         }
      }
   }

   return FALSE;
}

/** Updates user buffer and can set move flag. Blocking.
 * @function   ecat_Update
 * @abstract               Updates user buffer and can set move flag. Blocking.
 * @param      slave       Specifies slave(s) to be updated. When = 0, update all slaves.
 * @param      move        When TRUE, In profile position mode: sets setpoint. In homing: start homing. 
 * @param      timeout_ms  Timeout in ms. When = 0, no timeout.
 * @result                 Returns error code. 0 when successful.
 * @see AKDController::ecat_Talker
*/
int AKDController::Update(uint slave, bool move, int timeout_ms){

   pthread_mutex_lock(&this->control);
   if(this->masterState == ecat_masterStates::ms_shutdown) {
      pthread_mutex_unlock(&this->control);
      return FALSE;
   }

   if(slave > this->slaveCount) {
      pthread_mutex_unlock(&this->control);
      return -3;
   }

   int err = 0, slaveNum;
   struct timespec timeout;
   bool allUpdated = FALSE;

   clock_gettime(CLOCK_REALTIME, &timeout);
   add_timespec(&timeout, (int64)timeout_ms * 1000 * 1000);

   if(slave != 0) slaveNum = slave - 1;
   else slaveNum = 0;
   do{ // Set update flag (And possibly bit 4)

      this->slaves[slaveNum].update = TRUE;
      if(move && (slaves[slaveNum].mode == ecat_OpModes::homing || slaves[slaveNum].mode == ecat_OpModes::profPos || slaves[slaveNum].mode == ecat_OpModes::intPos) ) 
         this->slaves[slaveNum].coeCtrlWord |= 0b0010000; // Should only be used if in homing, interpolated or profPos mode.

      slaveNum++;
   } while((slaveNum < this->slaveCount) && (slave == 0));


   allUpdated = FALSE;
   while(!allUpdated && err == 0){ // Check and make sure all slave update flags were recognized.
      #if AKD_ECAT_DEBUG_MODE 
         pthread_mutex_lock(&this->debug);
         printDebugBuff(); 
         pthread_mutex_unlock(&this->debug);
      #endif
      if(timeout_ms != 0) err = pthread_cond_timedwait(&this->IOUpdated, &this->control, &timeout); //Wait for talker thread to confirm update to IOmap
      else pthread_cond_wait(&this->IOUpdated, &this->control);
      #if AKD_ECAT_DEBUG_MODE 
         pthread_mutex_lock(&this->debug);
         printDebugBuff(); 
         pthread_mutex_unlock(&this->debug); 
      #endif

      allUpdated = TRUE;
      if(slave != 0) slaveNum = slave - 1;
      else slaveNum = 0;
      do{ 
         if (this->slaves[slaveNum].update == TRUE){ // Loop through each slaves, check if update flag was ack
            allUpdated = FALSE;
            break;
         }
         slaveNum++;
      }while((slaveNum < this->slaveCount) && (slave == 0));

   }

   #if AKD_ECAT_DEBUG_MODE 
      pthread_mutex_lock(&this->debug);
      printDebugBuff(); 
      printf("\nUpdate(): Slaves Updated\n");
      printDebugBuff();
      pthread_mutex_unlock(&this->debug);
   #endif
   

   allUpdated = FALSE;
   while(!allUpdated && err == 0 && move){ // Check to see if move was processed or error (ie err bit = 1)

      #if AKD_ECAT_DEBUG_MODE 
         pthread_mutex_lock(&this->debug);
         printDebugBuff(); 
         pthread_mutex_unlock(&this->debug); 
      #endif
      if(timeout_ms != 0) err = pthread_cond_timedwait(&this->IOUpdated, &this->control, &timeout); //Wait for talker thread to confirm update to move status.
      else pthread_cond_wait(&this->IOUpdated, &this->control);
      #if AKD_ECAT_DEBUG_MODE 
         pthread_mutex_lock(&this->debug);
         printDebugBuff(); 
         pthread_mutex_unlock(&this->debug);  
      #endif

      allUpdated = TRUE;
      if(slave != 0) slaveNum = slave - 1;
      else slaveNum = 0;
      do{
         if(this->slaves[slaveNum].coeStatus & (1 << 3)) return AKD_MOVEERR; // Fault (Untested)

         if (!(this->slaves[slaveNum].coeCtrlWord & (1 << 4))){ // If move bit is cleared by talker, move was processed
            if(this->slaves[slaveNum].coeStatus & (1 << 13)) return AKD_MOVEERR; // Move error (Untested)
         }
         else{
            allUpdated = FALSE; // If move bit is still set, move is still being processed
            break;
         }
         slaveNum++;
      }while((slaveNum < this->slaveCount) && (slave == 0));
         
   }
   #if AKD_ECAT_DEBUG_MODE 
      pthread_mutex_lock(&this->debug);
      printDebugBuff(); 
      printf("\nUpdate(): Move was processed\n");
      printDebugBuff(); 
      pthread_mutex_unlock(&this->debug);
   #endif
   
   pthread_mutex_unlock(&this->control);
   return err; // Return err
}
 
/** Changes overall master state. Can block for 5 seconds max.
 * @function   ecat_State
 * @abstract               Changes overall master state. Can block for 5 seconds max.
 * @param      reqState    Master state to change to. (shutdown, stop, disable, enable)
 * @result                 Returns TRUE if successful.
*/
bool AKDController::State(ecat_masterStates reqState){

   pthread_mutex_lock(&this->control);
   if(this->masterState == ecat_masterStates::ms_shutdown) {
      pthread_mutex_unlock(&this->control);
      return FALSE;
   }

   struct timespec timeout;
   uint sdoBuffSize, sdoBuff;
   int err = 0, waitingFor;
   bool allStatesChanged = FALSE;
   ecat_masterStates oldState;

   if(this->masterState != reqState){
         if(reqState != ecat_masterStates::ms_shutdown){

            oldState = this->masterState;
            this->masterState = reqState;

            clock_gettime(CLOCK_REALTIME, &timeout);
            timeout.tv_sec += 5; // 5 sec timeout
            
            switch(reqState){ //Wait for talker thread to confirm update
               case ecat_masterStates::ms_stop :
                  waitingFor = RDY2SWITCH & 0b11001111;
               break;
               case ecat_masterStates::ms_disable :
                  waitingFor = SWITCHEDON & 0b11001111;
               break;
               case ecat_masterStates::ms_enable :
                  waitingFor = OP_ENABLED & 0b11001111;
               break;
               default :
                  err = -1;
            }

            while(!allStatesChanged && err == 0){
               allStatesChanged = TRUE;
               for(int slaveNum = 0 ; slaveNum < this->slaveCount ; slaveNum++){
                  if ((this->slaves[slaveNum].coeStatus & 0b11001111) != waitingFor){
                     
                     if(this->slaves[slaveNum].coeStatus & (1 << 3)){ // Check fault bit
                        pthread_mutex_unlock(&this->control);
                        this->masterState = oldState; // Reset state
                        return FALSE; // If a slave is in fault, return failed
                     }

                     allStatesChanged = FALSE;
                     break;
                  }
               }
               err = pthread_cond_timedwait(&this->stateUpdated, &this->control, &timeout);
            }
         }
        else{// Shutdown threads
            this->masterState = ecat_masterStates::ms_shutdown;
            pthread_mutex_unlock(&this->control);
            pthread_join(this->talker, NULL);
            pthread_join(this->controller, NULL);

            pthread_mutex_destroy(&this->control);
            pthread_mutex_destroy(&this->debug);

            pthread_cond_destroy(&this->IOUpdated);
            pthread_cond_destroy(&this->stateUpdated);

            delete slaves;

            printf("\nECAT: Requesting init state for all slaves\n");
            ec_slave[0].state = EC_STATE_INIT;
            /* request INIT state for all slaves */
            ec_writestate(0);

            printf("ECAT: Shutting down master on %s, close socket\n", this->ifname);
            /* stop SOEM, close socket */
            ec_close();

            this->inOP = false;
        }
   }

   pthread_mutex_unlock(&this->control);
   return err == 0;

}

/** Tries to put master into enable state. Can block for 5 seconds max.
 * @function   Enable
 * @abstract               Tries to put master into enable state. Can block for 5 seconds max.
 * @result                 Returns TRUE if successful.
 * @see AKDController::State
*/
bool AKDController::Enable(){
   return AKDController::State(ecat_masterStates::ms_enable);
}

/** Tries to put master into disable state. Can block for 5 seconds max.
 * @function   Disable
 * @abstract               Tries to put master into disable state. Can block for 5 seconds max.
 * @result                 Returns TRUE if successful.
 * @see AKDController::State
*/
bool AKDController::Disable(){
   return AKDController::State(ecat_masterStates::ms_disable);
}

/** Tries to put master into stop state. Can block for 5 seconds max.
 * @function   Stop
 * @abstract               Tries to put master into stop state. Can block for 5 seconds max.
 * @result                 Returns TRUE if successful.
 * @see AKDController::State
*/
bool AKDController::Stop(){
   return AKDController::State(ecat_masterStates::ms_stop);
}

/** Tries to put master into shutdown state. Resources are freed. Can block for 5 seconds max.
 * @function   Shutdown
 * @abstract               Tries to put master into shutdown state. Resources are freed. Can block for 5 seconds max.
 * @result                 Returns TRUE if successful.
 * @see AKDController::State
*/
bool AKDController::Shutdown(){
   return AKDController::State(ecat_masterStates::ms_shutdown);
}

/** Enables/disables quickstop for specified slave(s).
 * @function   QuickStop
 * @abstract               Enables/disables quickstop for specified slave(s).
 * @param      slave       Specifies slave(s) to be set. When = 0, set all slaves.
 * @param enableQuickStop  When TRUE, set quickstop. When FALSE, release quickstop.
 * @result                 Returns TRUE if successful.
 * @see AKDController::ecat_Talker
*/
bool AKDController::QuickStop(uint slave, bool enableQuickStop){ 

   pthread_mutex_lock(&this->control);
   if(this->masterState == ecat_masterStates::ms_shutdown) {
      pthread_mutex_unlock(&this->control);
      return FALSE;
   }

   if(slave > this->slaveCount) {
      pthread_mutex_unlock(&this->control);
      return FALSE;
   }

   int slaveNum;
   if(slave != 0) slaveNum = slave - 1;
   else slaveNum = 0;
   do{

      if(enableQuickStop){
         this->slaves[slaveNum].coeCtrlWord &= ~0b100;
         *this->slaves[slaveNum].coeCtrlMapPtr &= ~0b100; // May not set properly, but just in case Talker isn't working or can't lock control mutex.
      }
      else
         this->slaves[slaveNum].coeCtrlWord |= 0b100; // Disable quickstop

      slaveNum++;
   }while((slaveNum < this->slaveCount) && (slave == 0));

   pthread_mutex_unlock(&this->control);
   return TRUE;
}

/** Changes operation mode of slave(s). Currently only profPos and Homing are tested.
 * @function   setOpMode
 * @abstract               Changes operation mode of slave(s). Currently only profPos and Homing are tested.
 * @param      slave       Specifies slave(s) to be set. When = 0, set all slaves.
 * @param      reqMode     Requested operational mode.
 * @result                 Returns TRUE if successful.
*/
bool AKDController::setOpMode(uint slave, ecat_OpModes reqMode){

   pthread_mutex_lock(&this->control);
   if(this->masterState == ecat_masterStates::ms_shutdown) {
      pthread_mutex_unlock(&this->control);

      return FALSE;
   }
   pthread_mutex_unlock(&this->control);

   if(slave > this->slaveCount) return FALSE;

   int sdoBuffSize, sdoBuff, slaveNum;
   struct timespec timeout, curtime;
   clock_gettime(CLOCK_MONOTONIC, &timeout);
   timeout.tv_sec += 5;

   
   if((this->inOP == false) || Disable()){  // Only change opmode when either the master is not in operation
                                    // or in operation and has successfully disabled.
                                    // Disable() shouldn't evaluate if inOp == false.
                                    // Also, inOP shouldn't need a mutex/lock since it shouldn't be set 
                                    // outside of Start() and Shutdown(), which are called by the user.
      
      pthread_mutex_lock(&this->control);
      
      if(slave != 0) slaveNum = slave;
      else slaveNum = 1;
      do{

         if(this->slaves[slaveNum].mode != reqMode){  
   
            do{
               sdoBuffSize = 1;
               sdoBuff = (int)reqMode;
               ec_SDOwrite(slaveNum, REQOPMODE, FALSE, sdoBuffSize, &sdoBuff, EC_TIMEOUTRXM); // Assign Operational Mode

               sdoBuffSize = 1; // Check if drive has acknowledged
               sdoBuff = 0;
               ec_SDOread(slaveNum, ACTOPMODE, FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTRXM);

               clock_gettime(CLOCK_MONOTONIC, &curtime);
               if(curtime.tv_sec > timeout.tv_sec){
                  pthread_mutex_unlock(&this->control);
                  return FALSE;
               }

            } while(sdoBuff != (int)reqMode);

            this->slaves[slaveNum-1].mode = reqMode;
         }

         slaveNum++;
      }while((slaveNum <= this->slaveCount) && (slave == 0));

      pthread_mutex_unlock(&this->control);

      if(this->inOP == true) Enable();
   } else return FALSE;

   
   
   return TRUE;
}

/** Configures profPos settings when in profPos mode.
 * @function   confProfPos
 * @abstract               Configures profPos settings when in profPos mode.
 * @param      slave       Specifies slave(s) to be configured. When = 0, configured all slaves.
 * @param    moveImmediate Set move immediately flag. Flag persists. Otherwise set point is put into buffer.
 * @param    moveRelative  Set move relatively flag. Flag persists.
 * @result                 Returns TRUE if successful.
*/
bool AKDController::confProfPos(uint slave, bool moveImmediate, bool moveRelative){

   pthread_mutex_lock(&this->control);
   if(this->masterState == ecat_masterStates::ms_shutdown) {
      pthread_mutex_unlock(&this->control);
      return FALSE;
   }
   printf("Checking lsaves\n");
   if(slave > this->slaveCount) {
      pthread_mutex_unlock(&this->control);
      return FALSE;
   }

   int slaveNum;
   if(slave != 0) slaveNum = slave - 1;
   else slaveNum = 0;
   do{
      if(this->slaves[slaveNum].mode != ecat_OpModes::profPos){
         pthread_mutex_unlock(&this->control);
         printf("Invalid op mode: %i\n", (int)this->slaves[slaveNum].mode);
         return FALSE;
      }

      if(moveImmediate) this->slaves[slaveNum].coeCtrlWord |= 0b00100000;
      else this->slaves[slaveNum].coeCtrlWord &= ~0b00100000;

      if(moveRelative) this->slaves[slaveNum].coeCtrlWord |= 0b01000000;
      else this->slaves[slaveNum].coeCtrlWord &= ~0b01000000;

      slaveNum++;
   } while((slaveNum < this->slaveCount) && (slave == 0));
   pthread_mutex_unlock(&this->control);
   
   return TRUE;
}

/** Configures DS402 unit scaling.
 * @function   confUnits
 * @abstract               Configures DS402 unit scaling.
 * @param      slave       Specifies slave(s) to be configured. When = 0, configured all slaves.
 * @param      motorRev    Sets motor revolutions per shaft revolutions.
 * @param      shaftRev    Sets shaft revolutions per motor revolutions.
 * @result                 Returns TRUE if successful.
*/
bool AKDController::confUnits(uint slave, uint32 motorRev, uint32 shaftRev){

   pthread_mutex_lock(&this->control);
   if(this->masterState == ecat_masterStates::ms_shutdown) {
      pthread_mutex_unlock(&this->control);
      return FALSE;
   }
   pthread_mutex_unlock(&this->control);

   if(slave > this->slaveCount) return FALSE;

   int slaveNum, sdoBuff, sdoBuffSize;
   
   if(slave != 0) slaveNum = slave;
   else slaveNum = 1;
   do{ 
      sdoBuff = motorRev;
      ec_SDOwrite(slaveNum, MOTOR_RATIO , FALSE, 4, &sdoBuff, EC_TIMEOUTRXM); // Set acceleration for profile position mode
      sdoBuff = shaftRev;
      ec_SDOwrite(slaveNum, SHAFT_RATIO , FALSE, 4, &sdoBuff, EC_TIMEOUTRXM); // Set decceleration for profile position mode
      
      sdoBuffSize = 4;
      ec_SDOread(slaveNum, MOTOR_RATIO, FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTTXM);
      if(sdoBuff != motorRev) return FALSE;
      sdoBuffSize = 4;
      ec_SDOread(slaveNum, SHAFT_RATIO, FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTTXM);
      if(sdoBuff != shaftRev) return FALSE;
   
      slaveNum++;
   } while((slaveNum <= this->slaveCount) && (slave == 0));

   return TRUE;
}

/** Configures motion task values. (Acceleration, velocity)
 * @function   confMotionTask
 * @abstract               Configures motion task values. (Acceleration, velocity)
 * @param      slave       Specifies slave(s) to be configured. When = 0, configured all slaves.
 * @param      vel         Velocity (MT.V)
 * @param      acc         Accelerlation (MT.ACC)
 * @param      dec         Decceleration (MT.DEC)
 * @result                 Returns TRUE if successful.
*/
bool AKDController::confMotionTask(uint slave, uint vel, uint acc, uint dec){

   pthread_mutex_lock(&this->control);
   if(this->masterState == ecat_masterStates::ms_shutdown) {
      pthread_mutex_unlock(&this->control);
      return FALSE;
   }
   pthread_mutex_unlock(&this->control);

   
   if(slave > this->slaveCount) return FALSE;

   uint slaveNum, sdoBuff;
   int sdoBuffSize, i;
   bool usingDS402Scaling;
   if(slave != 0) slaveNum = slave;
   else slaveNum = 1;
   do{
      
      i = 0;
      do{ 
         sdoBuff = acc; // Set acceleration for profile position mode  
         //printf("CONFMOTION: Writing to MT_ACC (%i)\n", i);
         if(i >= 10) return FALSE;
         else i++;
      }while(ec_SDOwrite(slaveNum, MT_ACC , FALSE, 4, &sdoBuff, EC_TIMEOUTRXM) != 1);

      i = 0;
      do{
         sdoBuff = dec; // Set decceleration for profile position mode
         //printf("CONFMOTION: Writing to MT_DEC (%i)\n", i);
         if(i >= 10) return FALSE;
         else i++;
      }while(ec_SDOwrite(slaveNum, MT_DEC , FALSE, 4, &sdoBuff, EC_TIMEOUTRXM) != 1);

      i = 0;
      do{
         sdoBuff = vel; // Set velocity for profile position mode
         //printf("CONFMOTION: Writing to MT_V (%i)\n", i);
         if(i >= 10) return FALSE;
         else i++;
      }while(ec_SDOwrite(slaveNum, MT_V , FALSE, 4, &sdoBuff, EC_TIMEOUTRXM) != 1);
      

   
      slaveNum++;
   } while((slaveNum <= this->slaveCount) && (slave == 0));

   return TRUE;
}

/** Configures slave PDO assign objects for when the master is started. 
 * Also stores pointer to the buffer that the user will use to interface with the slave.
 * @function   confSlavePDOs
 * @abstract               Configures slave PDO assign objects for when the master is started. 
 *                         Also stores pointer to the buffer that the user will use to interface with the slave.
 * @param      slave       Specifies slave to be configured. Cannot be 0.
 * @param      usrControl  Pointer to the buffer for this slave. BUFFER MUST BE PACKED AND HAVE EACH ENTRY SIZED AND ORDERED APPROPRIATELY. (Outputs first then inputs)
 * @param      bufferSize  Size of the user buffer. ie sizeOf(*usrControl) Could potentially segment fault if wrong size is entered.
 * @param      rxPDO1      Object index for 1st output(to slave) PDO assignment.
 * @param      rxPDO2      Object index for 2st output(to slave) PDO assignment.
 * @param      rxPDO3      Object index for 3st output(to slave) PDO assignment.
 * @param      rxPDO4      Object index for 4st output(to slave) PDO assignment.
 * @param      txPDO1      Object index for 1st input(from slave) PDO assignment.
 * @param      txPDO2      Object index for 2st input(from slave) PDO assignment.
 * @param      txPDO3      Object index for 3st input(from slave) PDO assignment.
 * @param      txPDO4      Object index for 4st input(from slave) PDO assignment.
*/
void AKDController::confSlavePDOs(uint slave, const void* usrControl, int bufferSize, uint16 rxPDO1, uint16 rxPDO2, uint16 rxPDO3, uint16 rxPDO4, uint16 txPDO1, uint16 txPDO2, uint16 txPDO3, uint16 txPDO4){
   int rx = 0, tx = 0;
   
   pthread_mutex_lock(&this->control);
   if(this->masterState == ecat_masterStates::ms_shutdown) {
      pthread_mutex_unlock(&this->control);
      return;
   }

   if(slave > this->slaveCount || slave == 0) return;

   if(rxPDO1 != 0) rx++;
   if(rxPDO2 != 0) rx++;
   if(rxPDO3 != 0) rx++;
   if(rxPDO4 != 0) rx++;

   if(txPDO1 != 0) tx++;
   if(txPDO2 != 0) tx++;
   if(txPDO3 != 0) tx++;
   if(txPDO4 != 0) tx++;


   slaves[slave-1].rxPDO.numOfPDOs = rx;
   slaves[slave-1].txPDO.numOfPDOs = tx;

   slaves[slave-1].rxPDO.mapObject[0] = rxPDO1;
   slaves[slave-1].rxPDO.mapObject[1] = rxPDO2;
   slaves[slave-1].rxPDO.mapObject[2] = rxPDO3;
   slaves[slave-1].rxPDO.mapObject[3] = rxPDO4;
   
   slaves[slave-1].txPDO.mapObject[0] = txPDO1;
   slaves[slave-1].txPDO.mapObject[1] = txPDO2;
   slaves[slave-1].txPDO.mapObject[2] = txPDO3;
   slaves[slave-1].txPDO.mapObject[3] = txPDO4;

   slaves[slave-1].totalBytes = bufferSize;

   slaves[slave-1].outUserBuff = (uint8*)usrControl;
      
   

   pthread_mutex_unlock(&this->control);
}

/** Configures slave flexible PDO entry objects. (Used by overloaded functions)
 * @function   _confSlaveEntries
 * @abstract               Configures slave flexible PDO entry objects.
 *                         Also stores pointer to the buffer that the user will use to interface with the slave.
 * @param      slave       Specifies slave to be configured. When = 0, update all slaves.
 * @param   flexPdoObject  Object index to map entries. Can only be 0x1600, 0x1601, 0x1602, 0x1603 for txPDOs and 0x1a00, 0x1a01, 0x1a02, 0x1a03 for rxPDOs. (See manual pg 44)
 * @param      objEntries  Array of object indexes for Nth PDO entry.
 * @param      subIEntries Array of subindexes for Nth PDO entry.
*/
bool AKDController::confSlaveEntries(uint slave, ecat_pdoEntry_t *rxEntries, int numOfRx, ecat_pdoEntry_t *txEntries, int numOfTx){

   int slaveNum, sdoBuff, sdoBuffSize, errCnt, totalBits, wkc;
   int mapObject, assignObject;
   int txSizes[numOfTx], rxSizes[numOfRx], *sizes;
   ecat_pdoEntry_t *entries;
   int entry, pdoEntry;
   int numOfEntries, numOfPDOs;
   
   pthread_mutex_lock(&this->control);
   if(this->masterState == ecat_masterStates::ms_shutdown) {
      pthread_mutex_unlock(&this->control);
      return false;
   }
   pthread_mutex_unlock(&this->control);

   if(slave != 0) slaveNum = slave;
   else slaveNum = 1;
   do{

      // Check size of object entries
      entries = rxEntries;
      sizes = rxSizes;
      numOfEntries = numOfRx;
      for(int tx = 0; tx <= 1; tx++){ // Check rx first then tx.
      entry = 0;
         totalBits = 0;
         while(entry < numOfEntries){
            errCnt = 0;
            do{
               sdoBuffSize = 4;
               if(errCnt > 10) return false;
               else errCnt++;
            }while(ec_SDOread(slaveNum, entries[entry].index, entries[entry].subIndex , FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTRXM) != 1);
            sizes[entry] = sdoBuffSize * 8;
            totalBits += sdoBuffSize * 8;
            entry++;
         }
         if(tx){
            if(totalBits > (32*8)) return false; //If too many tx bits, fail.
         }else{
            if(totalBits > (20*8)) return false; //If too many rx bits, fail.
         }

         entries = txEntries;
         sizes = txSizes;
         numOfEntries = numOfTx;
      }

      // Disable all assignment and mapping objects. Preps for writing.
      assignObject = rxPDOAssign;
      mapObject = rxFlexPDO1;
      for(int tx = 0; tx <= 1; tx++){ // Loop rx first then tx.

         // Disable assignment object
         sdoBuff = 0;
         errCnt = 0;
         do{
            if(errCnt > 10) return false;
            else errCnt++;
            wkc = ec_SDOwrite(slaveNum, assignObject, 0 , false, 1, &sdoBuff, EC_TIMEOUTRXM);
         }while(wkc != 1);
         
         // Disable all flex mapping objects
         while(1){

            sdoBuff = 0;
            errCnt = 0;
            do{
               if(errCnt > 10) return false;
               else errCnt++;
               wkc = ec_SDOwrite(slaveNum, mapObject, 0 , false, 1, &sdoBuff, EC_TIMEOUTRXM);
            }while(wkc != 1);

            if(tx){
               if(mapObject == txFlexPDO4) break;
            }
            else{
               if(mapObject == rxFlexPDO4) break;
            }
            mapObject++;
         }

         assignObject = txPDOAssign;
         mapObject = txFlexPDO1;
      }

      assignObject = rxPDOAssign;
      mapObject = rxFlexPDO1;
      entries = rxEntries;
      sizes = rxSizes;
      numOfEntries = numOfRx;
      for(int tx = 0; tx <= 1; tx++){ // Loop rx first then tx.

         // Cycle through every map object
         entry = 0;
         numOfPDOs = 0;
         while(1){

            totalBits = 0;
            pdoEntry = 0;
            // Cycle through every entry mapping and fill flexible pdo object map with (entry,subindex,size)
            while(1){

               // If the PDO is full (max of 8 entries),
               // OR the next entry doesn't exist,
               // OR the next entry will exceed the PDOs size of 8 bytes,
               if((pdoEntry > 8) || (entry >= numOfEntries) || ((sizes[entry] + totalBits) > (8*8))){
                  // procceed to use the next PDO.

                  // But first, enable all the entries of previous map
                  errCnt = 0;
                  if(pdoEntry > 8) sdoBuff = 8;
                  else sdoBuff = pdoEntry;
                  do{ 
                     if(errCnt > 10) return false;
                     else errCnt++;
                  }while(ec_SDOwrite(slaveNum, mapObject, 0 , FALSE, 1, &sdoBuff, EC_TIMEOUTRXM) != 1);

                  if(pdoEntry > 0) numOfPDOs++;

                  break;
               }
               pdoEntry++;
               
               sdoBuff = (entries[entry].index << 16) + (entries[entry].subIndex << 8) + (sizes[entry]);
               #if AKD_ECAT_DEBUG_MODE
                  printf("ECAT: confSlaveEntries: Slave %i PDO %04x Entry %i = 0x%08x\n", slaveNum, mapObject, pdoEntry, sdoBuff);
               #endif

               errCnt = 0;
               do{
                  if(errCnt > 10) return false;
                  else errCnt++;
               }while(ec_SDOwrite(slaveNum, mapObject, pdoEntry , FALSE, 4, &sdoBuff, EC_TIMEOUTRXM) != 1);

               totalBits += sizes[entry];
               entry++;
            }

            if(((mapObject == txFlexPDO4) && tx) || ((mapObject == rxFlexPDO4) && !tx)){
               if(entry != numOfEntries) return false; // If the entries won't fit in the given order, fail.
               else break;
            }
            mapObject++;            
         }

         errCnt = 0;
         sdoBuff = numOfPDOs;
         do{
            if(errCnt > 10) return false;
            else errCnt++;
         }while(ec_SDOwrite(slaveNum, assignObject, 0 , FALSE, 1, &sdoBuff, EC_TIMEOUTRXM) != 1);


         assignObject = txPDOAssign;
         mapObject = txFlexPDO1;
         entries = txEntries;
         sizes = txSizes;
         numOfEntries = numOfTx;
      }
   
   slaveNum++;
   } while((slaveNum <= this->slaveCount) && (slave == 0));


   return true;
}

/** Configure digital outputs.
 * @function   confDigOutputs
 * @abstract               Configure digital outputs.
 * @param      slave       Specifies slave(s) to be updated. When = 0, update all slaves.
 * @param      enableOut1  Enables output 1 to be set by user.
 * @param      enableOut2  Enables output 2 to be set by user.
 * @param      out1Mode    Changes output 1 mode. (0 = user controlled)
 * @param      out2Mode    Changes output 2 mode. (0 = user controlled)
 * @result                 Returns TRUE if successful.
*/
bool AKDController::confDigOutputs(uint slave, bool enableOut1, bool enableOut2, uint8 out1Mode, uint8 out2Mode){

   pthread_mutex_lock(&this->control);
   if(this->masterState == ecat_masterStates::ms_shutdown) {
      pthread_mutex_unlock(&this->control);
      return FALSE;
   }
   pthread_mutex_unlock(&this->control);

   int slaveNum, sdoBuff, sdoBuffSize;

   if(slave != 0) slaveNum = slave - 1;
   else slaveNum = 0;
   do{

      sdoBuff = (enableOut1 << 16) || (enableOut2 << 17);
      ec_SDOwrite(slaveNum, DIG_OUT_MASK , FALSE, 4, &sdoBuff, EC_TIMEOUTRXM); // Set bitmask
      sdoBuff = out1Mode;
      ec_SDOwrite(slaveNum, DIG_OUT1_MODE , FALSE, 4, &sdoBuff, EC_TIMEOUTRXM); // Set acceleration for profile position mode
      sdoBuff = out2Mode;
      ec_SDOwrite(slaveNum, DIG_OUT2_MODE , FALSE, 4, &sdoBuff, EC_TIMEOUTRXM); // Set acceleration for profile position mode
      
      sdoBuffSize = 4;
      ec_SDOread(slaveNum, DIG_OUT_MASK, FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTTXM);
      if(sdoBuff != (enableOut1 << 16) || (enableOut2 << 17)) return FALSE;

      sdoBuffSize = 4;
      ec_SDOread(slaveNum, DIG_OUT1_MODE, FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTTXM);
      if(sdoBuff != out1Mode) return FALSE;

      sdoBuffSize = 4;
      ec_SDOread(slaveNum, DIG_OUT2_MODE, FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTTXM);
      if(sdoBuff != out2Mode) return FALSE;

      
      slaveNum++;
   } while((slaveNum < this->slaveCount) && (slave == 0));

   

   return TRUE;
}

/** Homes the specified slave with the specified home mode and paramaters. Blocking. (Only mode 0 is tested.)
 * @function   Home
 * @abstract               Homes the specified slave with the specified home mode and paramaters. Blocking. (Only mode 0 is tested.)
 * @param      slave       Specifies slave(s) to be updated. When = 0, update all slaves.
 * @param      mode        Homing mode to use. (Only 0 tested)
 * @param      dir         Direction to move during home. (For limit switches)
 * @param      speed       Homing speed.
 * @param      acc         Homing acceleration and decceleration.
 * @param      dist        Distance to move after home.
 * @param      pos         Position to set feedback to after home.
 * @param      timeout_ms  Timeout in milliseconds. If = 0, no timeout. 
 * @result                 Returns TRUE if successful. Returns FALSE if timed out.
*/
bool AKDController::Home(uint slave, int mode, int dir, int speed, int acc, int dist, int pos, int timeout_ms){

   pthread_mutex_lock(&this->control);
   if(this->masterState == ecat_masterStates::ms_shutdown) {
      pthread_mutex_unlock(&this->control);
      return FALSE;
   }

   if(slave > this->slaveCount) {
      pthread_mutex_unlock(&this->control);
      return FALSE;
   }

   uint sdoBuff, err = 0, slaveNum;
   int subTimeout;
   ecat_OpModes prevMode[slaveCount];
   struct timespec timeout1, timeout2;

   clock_gettime(CLOCK_REALTIME, &timeout1);
   add_timespec(&timeout1, (int64)timeout_ms * 1000 * 1000);
   

   if(slave != 0) slaveNum = slave;
   else slaveNum = 1;
   do{ 
      prevMode[slaveNum-1] = this->slaves[slaveNum-1].mode;

      // Configure Homing parameters.
      sdoBuff = mode;
      ec_SDOwrite(slaveNum, HM_MODE , FALSE, 4, &sdoBuff, EC_TIMEOUTRXM);
      
      // HOME.DIR
      sdoBuff = dir;
      ec_SDOwrite(slaveNum, HM_DIR , FALSE, 4, &sdoBuff, EC_TIMEOUTRXM); 

      // HOME.V
      sdoBuff = speed;
      ec_SDOwrite(slaveNum, HM_V , FALSE, 4, &sdoBuff, EC_TIMEOUTRXM); 

      // HOME.ACC/HOME.DEC
      sdoBuff = acc;
      ec_SDOwrite(slaveNum, HMACCEL , FALSE, 4, &sdoBuff, EC_TIMEOUTRXM);

      // HOME.DIST
      sdoBuff = dist;
      ec_SDOwrite(slaveNum, HM_DIST , FALSE, 4, &sdoBuff, EC_TIMEOUTRXM);

      // HOME.P
      sdoBuff = pos;
      ec_SDOwrite(slaveNum, HM_P , FALSE, 4, &sdoBuff, EC_TIMEOUTRXM); 

      slaveNum++;
   } while((slaveNum <= this->slaveCount) && (slave == 0));

   
   pthread_mutex_unlock(&this->control);


   if(!setOpMode(slave, ecat_OpModes::homing)) {
      #if AKD_ECAT_DEBUG_MODE
         printf("\nECAT: [Home] Failed to set homing mode.\n");
      #endif
      return false;
   }
   #if AKD_ECAT_DEBUG_MODE
      printf("\nECAT: [HOME] Opmode to homing.\n");
   #endif

   if(timeout_ms != 0){
      clock_gettime(CLOCK_REALTIME, &timeout2);
      timeout2.tv_sec = timeout1.tv_sec - timeout2.tv_sec;
      timeout2.tv_nsec = timeout1.tv_nsec - timeout2.tv_nsec;
      subTimeout = (timeout2.tv_sec * 1000) + (timeout2.tv_nsec / 1000000);
   }
   else{
      subTimeout = 0;
   }
   if(Update(slave, TRUE, subTimeout) != 0){
      #if AKD_ECAT_DEBUG_MODE
         printf("\nECAT: [Home] Update() failed.\n");
      #endif
      return false;
   }
   #if AKD_ECAT_DEBUG_MODE
      printf("\nECAT: [HOME] Updated.\n");
   #endif

   if(timeout_ms != 0){
      clock_gettime(CLOCK_REALTIME, &timeout2);
      timeout2.tv_sec = timeout1.tv_sec - timeout2.tv_sec;
      timeout2.tv_nsec = timeout1.tv_nsec - timeout2.tv_nsec;
      subTimeout = (timeout2.tv_sec * 1000) + (timeout2.tv_nsec / 1000000);
   }
   else{
      subTimeout = 0;
   }
   if(!waitForTarget(slave, subTimeout)){
      #if AKD_ECAT_DEBUG_MODE
         printf("\nECAT: [HOME] Timedout waiting to reach target.\n");
      #endif
      return false;
   }
   #if AKD_ECAT_DEBUG_MODE
      printf("\nECAT: [HOME] Target reached.\n");
   #endif

   if(slave != 0) slaveNum = slave - 1;
   else slaveNum = 0;
   do{ 
      if(!setOpMode(slaveNum+1, prevMode[slaveNum])) {
         #if AKD_ECAT_DEBUG_MODE
            printf("\nECAT: [Home] Failed to revert opmode.\n");
         #endif
         return false;
      }
      slaveNum++;
   } while((slaveNum < this->slaveCount) && (slave == 0));

   #if AKD_ECAT_DEBUG_MODE
      printf("\nECAT: [HOME] Reverted to previous opmode. Finished!\n");
   #endif

   return true;
}

/** Blocking function that waits for the current position target to be reached.
 * @function   waitForTarget
 * @abstract               Blocking function that waits for the current position target to be reached.
 * @param      slave       Specifies slave(s) to wait on. When = 0, wait for all slaves.
 * @param      timeout_ms  Timeout in milliseconds. If = 0, no timeout. 
 * @result                 Returns TRUE if successful. Returns FALSE if timed out.
*/
bool AKDController::waitForTarget(uint slave, uint timeout_ms){

   pthread_mutex_lock(&this->control);
   if(this->masterState == ecat_masterStates::ms_shutdown) {
      pthread_mutex_unlock(&this->control);
      return FALSE;
   }
   int slaveNum, err = 0;
   bool allFin = FALSE;
   struct timespec timeout;

   clock_gettime(CLOCK_REALTIME, &timeout);
   add_timespec(&timeout, (int64)timeout_ms * 1000 * 1000);

   #if AKD_ECAT_DEBUG_MODE
         pthread_mutex_lock(&this->debug);
         printDebugBuff();
         pthread_mutex_unlock(&this->debug);
   #endif

   while(!allFin && err == 0){ // Check to see if update was acknowledged or error

      
      if(timeout_ms != 0) err = pthread_cond_timedwait(&this->IOUpdated, &this->control, &timeout); //Wait for talker thread to confirm update to move status.
      else err = pthread_cond_wait(&this->IOUpdated, &this->control);
      #if AKD_ECAT_DEBUG_MODE
         pthread_mutex_lock(&this->debug);
         printDebugBuff();
         pthread_mutex_unlock(&this->debug);
      #endif

      allFin = TRUE;
      if(slave != 0) slaveNum = slave - 1;
      else slaveNum = 0;
      do{
         // Check target reached bit
         if (!(this->slaves[slaveNum].targetReached) && ((this->slaves[slaveNum].mode == ecat_OpModes::profPos) || (this->slaves[slaveNum].mode == ecat_OpModes::homing) || (this->slaves[slaveNum].mode == ecat_OpModes::profVel))){
            allFin = FALSE;

            // Check internal limit active bit
            if (this->slaves[slaveNum].coeStatus & (1 << 11)){ // UNTESTED
               err = -1; // If internal limit active, return -1;
            }
            break;
         }

         slaveNum++;
      }while((slaveNum < this->slaveCount) && (slave == 0));

      #if AKD_ECAT_DEBUG_MODE
         pthread_mutex_lock(&this->debug);
         printDebugBuff();
         pthread_mutex_unlock(&this->debug);
      #endif
   }

   pthread_mutex_unlock(&this->control);
   return err == 0;
}

/** Check for drive faults.
 * @function   readFault
 * @abstract               Check for drive faults.
 * @param      slave       Specifies slave(s) to wait on. When = 0, wait for all slaves.
 * @result                 Returns TRUE if a drive has a fault.
*/
bool AKDController::readFault(uint slave){

   pthread_mutex_lock(&this->control);
   if(this->masterState == ecat_masterStates::ms_shutdown) {
      pthread_mutex_unlock(&this->control);
      return FALSE;
   }

   int slaveNum;

   if(slave != 0) slaveNum = slave - 1;
   else slaveNum = 0;
   do{ 

      if(this->slaves[slaveNum].coeStatus & (1 << 3)){
         pthread_mutex_unlock(&this->control);
         return TRUE;
      }
   
      slaveNum++;
   } while((slaveNum < this->slaveCount) && (slave == 0));

   pthread_mutex_unlock(&this->control);
   return FALSE;
}

/** Clear drive faults. Can block for 5 seconds.
 * @function   clearFault
 * @abstract               Clear drive faults. Can block for 5 seconds.
 * @param      slave       Specifies slave(s) to wait on. When = 0, wait for all slaves.
 * @param   persistClear   When TRUE, let faults automatically clear in the future.
 * @result                 Returns TRUE if fault(s) are clear.
*/
bool AKDController::clearFault(uint slave, bool persistClear){

   pthread_mutex_lock(&this->control);
   if(this->masterState == ecat_masterStates::ms_shutdown) {
      pthread_mutex_unlock(&this->control);
      return FALSE;
   }


   int err = 0, slaveNum;
   bool noFaults;
   struct timespec timeout;
   clock_gettime(CLOCK_REALTIME, &timeout);
   timeout.tv_sec += 5;

   // Loop through all slaves and set clear fault bit
   if(slave != 0) slaveNum = slave - 1;
   else slaveNum = 0;
   do{
      this->slaves[slaveNum].coeCtrlWord |= 0b10000000; // Clear Fault : 0b1---xxxx
      slaveNum++;
   } while((slaveNum < this->slaveCount) && (slave == 0));

   // Loop through all slaves and check that all are out of fault.
   while(!noFaults && err == 0) {
      noFaults = TRUE;

      err = pthread_cond_timedwait(&this->stateUpdated, &this->control, &timeout);

      if(slave != 0) slaveNum = slave - 1;
      else slaveNum = 0;
      do{
         if(this->slaves[slaveNum].coeStatus & (1 << 3)){
            noFaults = FALSE;
            break;
         }
         slaveNum++;
      } while((slaveNum < this->slaveCount) && (slave == 0));
   }

   if(slave != 0) slaveNum = slave - 1;
   else slaveNum = 0;
   do{
      if(!persistClear) this->slaves[slaveNum].coeCtrlWord &= ~0b10000000; // Clear Fault : 0b1---xxxx
      slaveNum++;
   } while((slaveNum < this->slaveCount) && (slave == 0));

   pthread_mutex_unlock(&this->control);


   return err == 0;
}

bool AKDController::writeObject(uint slave, uint16_t object, uint8 subIndex, uint32_t data){

   uint32 sdoBuff;
   int sdoBuffSize, errCnt, slaveNum;

   if(slave == 0) slaveNum = 1;
   else slaveNum = slave;
   do{
      errCnt = 0;
      do{
         sdoBuffSize = 4;
         if(errCnt > 10) return false;
         else errCnt++;
      }while(ec_SDOread(slaveNum, object, subIndex , FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTRXM) != 1);
      
      sdoBuff = data;
      errCnt = 0;
      do{
         if(errCnt > 10) return false;
         else errCnt++;
      }while(ec_SDOwrite(slaveNum, object, subIndex , false, sdoBuffSize, &sdoBuff, EC_TIMEOUTRXM) != 1);

   slaveNum++;
   } while((slaveNum <= this->slaveCount) && (slave == 0));

   return true;

}
