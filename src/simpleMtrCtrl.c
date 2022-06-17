
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/queue.h>
#include <inttypes.h>
#include <pthread.h>

#include "ethercat.h"

//////////////////////////// Timing ////////////////////////////
#define NSEC_PER_SEC 1000000000
#define EC_TIMEOUTMON 500

//////////////////// PDO Assignment Objects ////////////////////
#define rxPDOEnable 0x1C12, 0
#define rxPDOAssign1 0x1C12, 1
#define rxPDOFixed   0x1725

#define txPDOEnable 0x1C13, 0
#define txPDOAssign1 0x1C13, 1
#define txPDOFixed   0x1B20

///////////////// Control and Status Objects //////////////////
#define COControl    0x6040
#define REQOPMODE    0x6060, 0
#define ACTOPMODE    0x6061, 0
#define MnfStatus    0x1002
#define COStatus     0x6041

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
#define MT_V          0x6081, 0
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

// Config
#define CYCLE_NS (8*1000*1000) //8ms
#define SYNC_WINDOW_NS (100*1000) //.1ms
#define SYNC_AQTIME_NS (1000*1000*1000) // 1000ms
#define SYNC_DIST (2000*1000) // 2ms

#define DEFAULTOPMODE profPos

// Operation
#define DEBUG_MODE TRUE







// Left as struct to allow future support of multiple masters on different interfaces.
// Init would dynamically spawn new shared structs and add it to a static master list.
// Right now, one shared struct for one master is sufficient
static struct ecat_Data_T{ 
   
   // Init
   char* ifname;

   // PDO buffers
   char IOmap[4096];//[4096];
   char *pdoBuff;
   uint numOfPDOs;
   int coeCtrlIdx, coeStatusIdx;
   char *coeCtrlMapPtr, *coeStatusMapPtr;
   int outSizes[4*8 + 4], inSizes[4*8 + 4]; // 0 is num of entries to a PDO, followed by byte size of object entry.
                              // Repeats for each PDO
   // Assuming 1 byte minimum, up to 8 objects can be assigned per PDO. For PDOs max.

   // Control signals
   boolean inOP, update;
   uint inSyncCount;
   int8 wrkCounter, expectedWKC;
   uint16 coeCtrlWord, coeStatus;
   uint64 diffDCtime;
   enum ecat_States{shutdown, stop, disable, enable} state;

   // Profile Control
   enum ecat_Modes {profPos = 1, profVel = 3, profTor = 4, homing = 6, intPos = 7, syncPos = 8} mode;
   boolean moveAck, moveErr;

   // Debug 
   int64 gl_toff, gl_delta;
   uint8 gl_integral;

   // Threading
   pthread_mutex_t debug, control;
   pthread_cond_t IOUpdated;
   pthread_cond_t moveSig, stateUpdated;
   pthread_t talker, controller;
} shared;






static int cpyData(void* dest, void* source, int bytes){

   switch(bytes){
      case 1: *(uint8*)dest = *(uint8*)source; break;
      case 2: *(uint16*)dest = *(uint16*)source; break;
      case 4: *(uint32*)dest = *(uint32*)source; break;
      case 8: *(uint64*)dest = *(uint64*)source; break;
      default: return -1; 
   }
return bytes;
}

void readPDOAssignments(uint16 Slave, uint16 PDOassign, int* sizeList, uint pdoAssignments)
{
   uint16 idxloop, nidx, subidxloop, rdat, idx, subidx;
   uint8 subcnt, sizeIndex = 0;
   int wkc, rdl;
   int32 rdat2;

   rdl = sizeof(rdat); rdat = 0;
   /* read PDO assign subindex 0 ( = number of PDO's) */
   wkc = ec_SDOread(Slave, PDOassign, 0x00, FALSE, &rdl, &rdat, EC_TIMEOUTRXM);
   pdoAssignments = rdat;
   /* positive result from slave ? */
   if ((wkc > 0) && (rdat > 0))
   {
      /* number of available sub indexes */
      nidx = rdat;
      /* read all PDO's */
      for (idxloop = 1; idxloop <= nidx; idxloop++)
      {
         rdl = sizeof(rdat); rdat = 0;
         /* read PDO assign */
         wkc = ec_SDOread(Slave, PDOassign, (uint8)idxloop, FALSE, &rdl, &rdat, EC_TIMEOUTRXM);
         /* result is index of PDO */
         idx = rdat;
         
         if (idx > 0)
         {
            rdl = sizeof(subcnt); subcnt = 0;
            /* read number of subindexes of PDO */
            wkc = ec_SDOread(Slave, idx, 0x00, FALSE, &rdl, &subcnt, EC_TIMEOUTRXM);
            subidx = subcnt;
            sizeList[sizeIndex] = subidx;
            /* for each subindex */
            for (subidxloop = 1; subidxloop <= subidx; subidxloop++)
            {
               rdl = sizeof(rdat2); rdat2 = 0;
               /* read SDO that is mapped in PDO */
               wkc = ec_SDOread(Slave, idx, (uint8)subidxloop, FALSE, &rdl, &rdat2, EC_TIMEOUTRXM);
               /* extract bitlength of SDO */
               if(((rdat2 & 0xFFFF0000) >> 16) == COControl) shared.coeCtrlIdx = subidxloop;
               if(((rdat2 & 0xFFFF0000) >> 16) == COStatus) shared.coeStatusIdx = subidxloop;
               sizeList[sizeIndex + subidxloop] = (rdat2 & 0xFF) / 8;
            }
         }
         sizeIndex += subidx + 1;
      }
   }
}


/* add ns to timespec */
static void add_timespec(struct timespec *ts, int64 addtime)
{
   int64 sec, nsec;

   nsec = addtime % NSEC_PER_SEC;
   sec = (addtime - nsec) / NSEC_PER_SEC;
   ts->tv_sec += sec;
   ts->tv_nsec += nsec;
   if ( ts->tv_nsec >= NSEC_PER_SEC )
   {
      nsec = ts->tv_nsec % NSEC_PER_SEC;
      ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
      ts->tv_nsec = nsec;
   }
}

/* PI calculation to get linux time synced to DC time */
static boolean ec_sync(int64 reftime, uint64 cycletime , int64 *offsettime, int64 dist, int64 window, int64 *d, int64 *i)
{
   static int64 integral = 0;
   int64 delta;
   /* set linux sync point 50us later than DC sync, just as example */
   delta = (reftime - dist) % cycletime;
   if(delta > (cycletime / 2)) { delta = delta - cycletime; } // If over half-period, make neg offset behrend next period
   if(delta>0){ integral--; }
   if(delta<0){ integral++; }
   *offsettime = -(delta / 120) + (integral / 5);
   *d = delta;
   *i = integral;
   return (*d < window) && (*d > (-window)) ; // Return TRUE if error is within window
}

/* RT EtherCAT thread */
static void* ecat_Talker(void* ptr)
{

   // Local variables
   struct timespec   ts;
   uint64 prevDCtime;
   int* dataToMap;
   char*    output_map_ptr;
   char*    input_map_ptr;
   char*     output_buff_ptr;
   char*     input_buff_ptr;

   // Buffers (To outside of thread)
   int8 wrkCounterb;
   uint inSyncCountb;
   int64 toff, sync_delta, sync_integral, diffDCtimeb;
   

   clock_gettime(CLOCK_MONOTONIC, &ts);
   ec_send_processdata();
   while(1)
   {
      wrkCounterb = ec_receive_processdata(EC_TIMEOUTRET);
      
      /* calculate next cycle start */
      add_timespec(&ts, CYCLE_NS+ toff);

      /* wait to cycle start */
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);

      /* calulate toff to get linux time and DC synced */
      if(ec_sync(ec_DCtime, CYCLE_NS, &toff, SYNC_DIST, SYNC_WINDOW_NS, &sync_delta, &sync_integral)){ // If withing sync window
         if (inSyncCountb != UINT32_MAX) inSyncCountb++; // Count number of cycles in window
      }
      else inSyncCountb = 0;

      #if DEBUG_MODE
      // Try to update debug info
      if(pthread_mutex_trylock(&shared.debug) == 0){
         shared.gl_toff = toff;
         shared.gl_delta = sync_delta;
         shared.gl_integral = sync_integral;
         pthread_mutex_unlock(&shared.debug);
      }
      #endif

      
      
      if(pthread_mutex_trylock(&shared.control) == 0){
         // CONTROL LOCKED
         if(shared.state == shutdown) {
            pthread_mutex_unlock(&shared.control);
            break;
         }
         shared.inSyncCount = inSyncCountb;
         shared.wrkCounter = wrkCounterb;
         shared.diffDCtime = ec_DCtime - prevDCtime;
         prevDCtime = ec_DCtime;  

         // Update DS402 Control and Status variables for ecat_Controller
         cpyData(shared.coeCtrlMapPtr, &shared.coeCtrlWord, sizeof(shared.coeCtrlWord));
         cpyData(&shared.coeStatus, shared.coeStatusMapPtr, sizeof(shared.coeStatus));

         if(shared.coeStatus & 0b01000000000000) {
            shared.moveAck = TRUE;
            pthread_cond_signal(&shared.moveSig);
         } else shared.moveAck = FALSE;

         if(shared.coeStatus & 0b10000000000000) {
            shared.moveErr = TRUE;
            pthread_cond_signal(&shared.moveSig);
         } else shared.moveErr = FALSE;

         if(shared.update){
            

            // Update outputs from user buffers to IOmap
            output_map_ptr = ec_slave[1].outputs;
            output_buff_ptr = shared.pdoBuff;
            for(int i = 1 ; i <= shared.outSizes[0] ; i++){ 
               if(i != shared.coeCtrlIdx) cpyData(output_map_ptr, output_buff_ptr, shared.outSizes[i]); // Copy data from user's input to IOmap. (Skipping Ctrl Word used by ecat_Controller)
               output_map_ptr += shared.outSizes[i];
               output_buff_ptr += shared.outSizes[i];
            }

            // Update inputs from user buffers to IOmap
            input_map_ptr = ec_slave[1].inputs;
            input_buff_ptr = shared.pdoBuff + shared.outSizes[0] + 1;
            for(int i = 1 ; i <= shared.inSizes[0] ; i++){
               cpyData(input_buff_ptr, input_map_ptr, shared.inSizes[i]); // Copy input data to user's buffer
               input_map_ptr += shared.inSizes[i];
               input_buff_ptr += shared.inSizes[i];
            }

            // Release caller of ecat_Update()
            shared.update = FALSE;
            pthread_cond_signal(&shared.IOUpdated);
            
         }
         else {
            if(shared.mode == profPos || shared.mode == homing) shared.coeCtrlWord &= ~0b0010000; // Clear move bit. In homing : start_homing, In profPos : new_setpoint, In intPos : Interpolate
         }

         pthread_mutex_unlock(&shared.control);
         // CONTROL UNLOCKED
      }
      
      
      ec_send_processdata();
   }

}

static void* ecat_Controller(void* ptr)
{
   printf("Spawning Controller\n");  

   // Local variables
   uint16 prevcStatus = -1;
   char coeStateReadable[9][25] = {
      "Not Ready To Switch-On",  //0
      "Switch-On Disabled",      //1
      "Ready To Switch-On",      //2
      "Switched-On",             //3
      "Op-Enabled",              //4
      "FAULT!",                  //5
      "Fault Reaction Active",   //6
      "QUICK STOP ACTIVE",       //7
      "Unknown State"            //8
   };
   char coeCtrlReadable[7][20] = {
      "Shutdown",                //0
      "Switch-On",               //1
      "Disable Voltage",         //2
      "Quick Stop",              //3
      "Disable Operation",       //4
      "Enable Operation",        //5
      "Fault Reset"              //6
   };
   uint coeCurrent, coeRequest, currentgroup, noDCCount;

   

   while(1)
   {
      // CONTROL LOCKED
      pthread_mutex_lock(&shared.control);
      if(shared.state == shutdown){
         pthread_mutex_unlock(&shared.control);  
         break;
      }
      if(shared.diffDCtime == 0) noDCCount++;
      else noDCCount = 0;
      if(noDCCount > 20) shared.state = stop;
      

      if(shared.inOP){

         // CANopen state machine
         switch(shared.coeStatus & 0b01101111){
            case QUICKSTOP :
            {
               coeCurrent = 7;
               coeRequest = 4;
               shared.coeCtrlWord = shared.coeCtrlWord & 0b100;
               break;
            }
            default :
            {
               switch(shared.coeStatus & 0b01001111){
                  case NOTRDY2SWCH : 
                  {
                     coeCurrent = 0;
                     coeRequest = 0; // Ready to switch
                     break;
                  }
                  case SWCHDISABLED : 
                  {
                     coeCurrent = 1;
                     coeRequest = 0; // Ready to switch
                     shared.coeCtrlWord = (shared.coeCtrlWord & ~0b10000111) | 0b110; // Ready2Switch/Shutdown : 0bx---x110
                     break;
                  }
                  case RDY2SWITCH & 0b11011111:
                  { //Stop
                     coeCurrent = 2; 
                     coeRequest = 1; // Switch on
                     if((shared.state == enable) || (shared.state = disable)) shared.coeCtrlWord = (shared.coeCtrlWord & ~0b10000111) | 0b0111; // Switch-On : 0bx---x111
                     break;
                  }
                  case SWITCHEDON  & 0b11011111:
                  { // Disable
                     coeCurrent = 3;
                     if((shared.state == stop) || (shared.state == shutdown)) {
                        coeRequest = 0; // Switch off
                        shared.coeCtrlWord = (shared.coeCtrlWord & ~0b10001111) | 0b0110; // Ready2Switch : 0bx---x110
                     }
                     else if(shared.state == enable){
                        coeRequest = 5; // Operation enable
                        shared.coeCtrlWord = (shared.coeCtrlWord & ~0b10001111) | 0b1111; // Operation Enabled : 0bx---1111
                     }
                     break;
                  }
                  case OP_ENABLED  & 0b11011111:
                  { // Enable
                     coeCurrent = 4;
                     if (shared.state != enable) {
                        coeRequest = 4; // Disable operation
                        shared.coeCtrlWord = (shared.coeCtrlWord & ~0b10001111) | 0b0111; // Operation Disabled : 0bx---0111
                     }
                     break;
                  }
                  case FAULT :
                  {
                     coeCurrent = 5;
                     coeRequest = 6;
                     shared.coeCtrlWord = shared.coeCtrlWord | 0b10000000 ; // Clear Fault : 0b1---xxxx
                     break;
                  }
                  case FAULTREACT :
                  {
                     coeCurrent = 6;
                     coeRequest = 0;
                     shared.coeCtrlWord = (shared.coeCtrlWord & ~0b10000111) | 0b110; // Ready2Switch/Shutdown : 0bx---x110
                     break;
                  }
                  default :
                  {
                     coeCurrent = 8;
                     coeRequest = 0;
                     shared.coeCtrlWord = (shared.coeCtrlWord & ~0b10000111) | 0b110; // Shutddown : 0bx---x110
                     break;
                  }
               }
               
            }
         }
         #if DEBUG_MODE
         if(prevcStatus != shared.coeStatus){
            printf("\nCoE State: %-24s (0x%04x)\tCoE Control: %-24s (0x%04x)\n", coeStateReadable[coeCurrent], shared.coeStatus, coeCtrlReadable[coeRequest], shared.coeCtrlWord);
            prevcStatus = shared.coeStatus;
         }
         #endif

         if((shared.wrkCounter < shared.expectedWKC) || ec_group[currentgroup].docheckstate)
         {
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
               if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
               {
                  ec_group[currentgroup].docheckstate = TRUE;
                  if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                  {
                     printf("\nERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                     ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                     printf("\nWARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                     ec_slave[slave].state = EC_STATE_OPERATIONAL;
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state > EC_STATE_NONE)
                  {
                     if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("\nMESSAGE : slave %d reconfigured\n",slave);
                     }
                  }
                  else if(!ec_slave[slave].islost)
                  {
                     /* re-check state */
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (ec_slave[slave].state == EC_STATE_NONE)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("\nERROR : slave %d lost\n",slave);
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
                        printf("\nMESSAGE : slave %d recovered\n",slave);
                     }
                  }
                  else
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("\nMESSAGE : slave %d found\n",slave);
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
               printf("\nOK : all slaves resumed OPERATIONAL.\n");
         }
         
      }
      
      
      #if DEBUG_MODE
      pthread_mutex_lock(&shared.debug);
      
      printf("\r");
      printf("WKC: %2i(%2i)\t", shared.wrkCounter, shared.expectedWKC);
      printf("DiffDC: %12"PRIi64"\t", shared.diffDCtime);
      printf("inSyncCount: %4i\t", shared.inSyncCount);
      //printf("coeStatus: 0x%04"PRIx16"\t", shared.coeStatus);
      //printf("CtrlWord: 0x%04"PRIx16"\t", shared.coeCtrlWord);

      if(shared.gl_toff*((shared.gl_toff>0) - (shared.gl_toff<0)) > (CYCLE_NS / 2)) printf("\tClock slipping! toff = %"PRIi64"  CYCLE_NS/2 = %d\n", shared.gl_toff, CYCLE_NS / 2);
      pthread_mutex_unlock(&shared.debug);
      fflush(stdout);
      #endif

      pthread_mutex_unlock(&shared.control);
      // CONTROL UNLOCKED

      pthread_cond_signal(&shared.stateUpdated);

      osal_usleep(50000); // 100ms
      
   }
   

}

boolean ecat_Init(char *ifname, void* usrControl, int size, uint16 outPDOObj, uint16 inPDOObj){

   printf("Starting EtherCAT Master\n");

   uint32 sdoBuff;
   uint sdoBuffSize;
   boolean sdosNotConfigured = 1;
   uint8 inputFrameSize, outputFrameSize;


   /* initialise SOEM, bind socket to ifname */
   if (ec_init(ifname))
   {
      printf("ec_init on %s succeeded.\n",shared.ifname);
      /* find and auto-config slaves */

       if ( ec_config_init(FALSE) > 0 )
      {
         // Initialize threading resources
         pthread_mutex_init(&shared.control, NULL);
         pthread_mutex_init(&shared.debug, NULL);
         pthread_cond_init(&shared.IOUpdated, NULL);
         pthread_cond_init(&shared.stateUpdated, NULL);
         pthread_cond_init(&shared.moveSig, NULL);

         // Initialize shared data
         memset(&shared, 0, sizeof(shared));
         shared.pdoBuff = usrControl;
         shared.ifname = ifname;

         // Configure PDO assignments
         printf("%d slaves found. Configuring PDO assigments...\n",ec_slavecount);
         for(int i = 1; sdosNotConfigured != 0 && i <=10 ; i++){
            if(i != 1) osal_usleep(100000); // If looping, wait 100ms
            sdosNotConfigured = 0;

            sdoBuff = 0;
            ec_SDOwrite(1, rxPDOEnable, FALSE, 1, &sdoBuff, EC_TIMEOUTRXM); // Disable rxPDO
            ec_SDOwrite(1, txPDOEnable, FALSE, 1, &sdoBuff, EC_TIMEOUTRXM); // Disable txPDO

            sdoBuff = outPDOObj;
            ec_SDOwrite(1, rxPDOAssign1, FALSE, 2, &sdoBuff, EC_TIMEOUTRXM); // Assign rxPDO1
            sdoBuff = inPDOObj;
            ec_SDOwrite(1, txPDOAssign1, FALSE, 2, &sdoBuff, EC_TIMEOUTRXM); // Assign txPDO1

            sdoBuff = 1;
            ec_SDOwrite(1, rxPDOEnable, FALSE, 1, &sdoBuff, EC_TIMEOUTRXM); // Disable rxPDO
            ec_SDOwrite(1, txPDOEnable, FALSE, 1, &sdoBuff, EC_TIMEOUTRXM); // Disable txPDO

            sdoBuffSize = 1;
            sdoBuff = 0;
            ec_SDOread(1, rxPDOEnable, FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTRXM); // Check rxPDO assignment = 1
            if(sdoBuff != 1) sdosNotConfigured++;
            sdoBuffSize = 1;
            sdoBuff = 0;
            ec_SDOread(1, txPDOEnable, FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTRXM); // Check txPDO assignment = 1
            if(sdoBuff != 1) sdosNotConfigured++;

            sdoBuffSize = 2;
            sdoBuff = 0;
            ec_SDOread(1, rxPDOAssign1, FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTRXM); // Assign rxPDO1
            if(sdoBuff != outPDOObj) sdosNotConfigured++;
            sdoBuffSize = 2;
            sdoBuff = 0;
            ec_SDOread(1, txPDOAssign1, FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTRXM); // Assign rxPDO1
            if(sdoBuff != inPDOObj) sdosNotConfigured++;
            
            readPDOAssignments(1,0x1C12, shared.outSizes, shared.numOfPDOs);
            sdoBuff = shared.numOfPDOs;
            readPDOAssignments(1,0x1C13, shared.inSizes, shared.numOfPDOs);
            shared.numOfPDOs += sdoBuff;

            if(sdosNotConfigured != 0) printf("\rPDO assignments failed to configure %2d time(s). Retrying...\n", i);
         }
         if(sdosNotConfigured == 0) printf("PDO's configured!\n");
         else {
            printf("PDO's failed to configure.\n");
            return FALSE;
         }

         // Configure Homing settings
         sdoBuff = 0;
         ec_SDOwrite(1, HM_AUTOMOVE , FALSE, 1, &sdoBuff, EC_TIMEOUTRXM); // Disable automove

         // Configure Profile Position settings
         //sdoBuff = 10;
         //ec_SDOwrite(1, MT_ACC , FALSE, 4, &sdoBuff, EC_TIMEOUTRXM); // Set acceleration for profile position mode
         //sdoBuff = 10;
         //ec_SDOwrite(1, MT_DEC , FALSE, 4, &sdoBuff, EC_TIMEOUTRXM); // Set acceleration for profile position mode
         
         //sdoBuff = 10;
         //ec_SDOwrite(1, MT_V, FALSE, 4, &sdoBuff, EC_TIMEOUTRXM); // Set acceleration for profile position mode

         // Set to stop state
         shared.state = stop;

         // Set to default mode
         //shared.mode = DEFAULTOPMODE;

         ecx_context.manualstatechange = 1;
         ec_config_map(&shared.IOmap);

         
         // Find where the DS402 CoE control word is in IOmap
         shared.coeCtrlMapPtr = ec_slave[1].outputs;
         for(int i = 1; i < shared.coeCtrlIdx; i++){
            shared.coeCtrlMapPtr = shared.coeCtrlMapPtr + shared.outSizes[i]; 
         }

         // Find where the DS402 CoE status word is in IOmap
         shared.coeStatusMapPtr = ec_slave[1].inputs;
         for(int i = 1; i < shared.coeStatusIdx; i++){
            shared.coeStatusMapPtr = shared.coeStatusMapPtr + shared.inSizes[i]; 
         }

         
         // Config DC (In PREOP)
         printf("Configuring DC. Should be in PRE-OP. ");
         printf(ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE * 4) == EC_STATE_PRE_OP ? "State: PRE_OP\n" : "ERR! NOT IN PRE_OP!\n") ;
         printf(ec_configdc() ? "Found slaves with DC.\n" : "Did not find slaves with DC.\n") ;
         ec_dcsync0(1, FALSE, CYCLE_NS, 0);
         
         
         
         // Syncing (In SAFEOP)
         printf("Slaves mapped, state to SAFE_OP... ");
         ec_slave[0].state = EC_STATE_SAFE_OP;
         ec_writestate(0);
         printf(ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4) == EC_STATE_SAFE_OP ? "State: SAFE_OP\n" : "ERR! NOT IN SAFE_OP!\n") ;
         shared.expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Calculated workcounter %d\n", shared.expectedWKC);
         
         /*for(int i = 0 ; i < 100 ; i++){ // Send 10,000 cycles of process data to tune other slaves
            ec_send_processdata();
            wkc = ec_receive_processdata(EC_TIMEOUTRET);
            printf("\rSending 100 process cycles: %5d WKC : %d", i, wkc);
         }*/

         /* Enable talker to handle slave PDOs in OP */
         pthread_create(&shared.talker, NULL, ecat_Talker, NULL);
         pthread_create(&shared.controller, NULL, ecat_Controller, NULL);
         printf("Talker started! Synccount needs to reach : %"PRIi64"\n", (uint64)SYNC_AQTIME_NS / CYCLE_NS);
         pthread_mutex_lock(&shared.control);
         while(shared.inSyncCount < (uint64)(SYNC_AQTIME_NS / CYCLE_NS)){
            pthread_mutex_unlock(&shared.control);
            osal_usleep(100000); // 100ms
            pthread_mutex_lock(&shared.control);
         }
         pthread_mutex_unlock(&shared.control);
         printf("\nMaster within sync window. Enabling sync0 generation!\n");
         ec_dcsync0(1, TRUE, CYCLE_NS, 0); //Enable sync0 generation


         // Go into OPERATIONAL
         printf("\nRequesting operational state for all slaves\n");
         ec_slave[0].state = EC_STATE_OPERATIONAL;
         ec_send_processdata();  // Send one valid data to prep slaves for OP state
         ec_receive_processdata(EC_TIMEOUTRET);
			ec_writestate(0);
			ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE); //wait for OP


         // Return results
         if (ec_slave[0].state == EC_STATE_OPERATIONAL )
			{
				printf("\nOperational state reached for all slaves!\n");
            shared.inOP = TRUE;
            return TRUE;
         }
         else
         {
            printf("\nNot all slaves reached operational state.\n");
            ec_readstate();
            for(int i = 1; i<=ec_slavecount ; i++)
            {
               ec_dcsync0(i+1, FALSE, 0, 0); // SYNC0,1
               if(ec_slave[i].state != EC_STATE_OPERATIONAL)
               {
                  printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                        i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
               }
            }
         }
      }
      else
        {
            printf("No slaves found!\n");
        }
   }
   else
   {
      printf("No socket connection on %s\nExecute as root\n", shared.ifname);
   }

   return FALSE;
}

// Blocks for timeout time
boolean ecat_Update(boolean move, int timeout_ms){
   int err;
   struct timespec updateTimeout;

   err = 0;
   pthread_mutex_lock(&shared.control);

   shared.update = TRUE;
   if(move) shared.coeCtrlWord |= 0b0010000;

   clock_gettime(CLOCK_REALTIME, &updateTimeout);
   add_timespec(&updateTimeout, (uint64)(timeout_ms * 1000000));

   while(shared.update == TRUE && err == 0)
      err = pthread_cond_timedwait(&shared.IOUpdated, &shared.control, &updateTimeout); //Wait for talker thread to confirm update to IOmap
   while(shared.moveAck == FALSE && shared.moveErr == FALSE && err == 0 && shared.mode != intPos)
      err = pthread_cond_timedwait(&shared.moveSig, &shared.control, &updateTimeout); //Wait for talker thread to confirm setpoint acknoledgement
   

   pthread_mutex_unlock(&shared.control);
   return err == 0 && !shared.moveErr; // Return 1 if no error
}
 
 // Blocks for max of 1 sec;
static boolean ecat_COEState(enum ecat_States reqState){

   struct timespec timeout;
   uint sdoBuffSize, sdoBuff;
   int err = 0;

   pthread_mutex_lock(&shared.control);
   if(shared.state != reqState){
      shared.state = reqState;
      clock_gettime(CLOCK_REALTIME, &timeout);
      timeout.tv_sec += 1;
      
      switch(reqState){ //Wait for talker thread to confirm update
         case stop :
            while(((shared.coeStatus & 0b1101111) != (RDY2SWITCH & 0b1101111)) && err == 0)
               err = pthread_cond_timedwait(&shared.stateUpdated, &shared.control, &timeout);
         break;
         case disable :
            while(((shared.coeStatus & 0b1101111) != (SWITCHEDON & 0b1101111)) && err == 0)
               err = pthread_cond_timedwait(&shared.stateUpdated, &shared.control, &timeout);
         break;
         case enable :
            while(((shared.coeStatus & 0b1101111) != (OP_ENABLED & 0b1101111)) && err == 0)
               err = pthread_cond_timedwait(&shared.stateUpdated, &shared.control, &timeout);
         break;
         default :
            err = -1;
         break;
      }
   }

   pthread_mutex_unlock(&shared.control);
   return err == 0;

}

boolean ecat_Enable(){
   return ecat_COEState(enable);
}

boolean ecat_Disable(){
   return ecat_COEState(disable);
}

boolean ecat_Stop(){
   return ecat_COEState(stop);
}

boolean ecat_Shutdown(){

   pthread_mutex_lock(&shared.control);
   shared.state = shutdown;
   pthread_mutex_unlock(&shared.control);
   pthread_join(shared.talker, NULL);
   pthread_join(shared.controller, NULL);

   printf("\nRequest init state for all slaves\n");
   ec_slave[0].state = EC_STATE_INIT;
   /* request INIT state for all slaves */
   ec_writestate(0);

   printf("End simple test, close socket\n");

   pthread_mutex_destroy(&shared.control);
   pthread_mutex_destroy(&shared.debug);

   pthread_cond_destroy(&shared.IOUpdated);
   pthread_cond_destroy(&shared.stateUpdated);
   pthread_cond_destroy(&shared.moveSig);

   /* stop SOEM, close socket */
   ec_close();
}

// Blocks for 1 sec
static boolean ecat_OpMode(enum ecat_Modes reqMode){

   uint sdoBuffSize, sdoBuff;
   struct timespec timeout, curtime;

   pthread_mutex_lock(&shared.control);
   if(shared.mode != reqMode){
      pthread_mutex_unlock(&shared.control);
      if(ecat_Disable()){ // Don't assign if not disabled   

         clock_gettime(CLOCK_MONOTONIC, &timeout);
         timeout.tv_sec += 1;
         do{
            sdoBuffSize = 1;
            sdoBuff = reqMode;
            ec_SDOwrite(1, REQOPMODE, FALSE, sdoBuffSize, &sdoBuff, EC_TIMEOUTRXM); // Assign Operational Mode
            sdoBuffSize = 1; // Check if drive has acknowledged
            sdoBuff = 0;
            ec_SDOread(1, ACTOPMODE, FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTRXM);
            clock_gettime(CLOCK_MONOTONIC, &curtime);
         } while(sdoBuff != reqMode && curtime.tv_sec < timeout.tv_sec);

         ecat_Enable();
      }
   }
   else pthread_mutex_unlock(&shared.control);

   if(sdoBuff == reqMode) {
      shared.mode = reqMode;
      return TRUE;
   }
   else return FALSE;
}

boolean ecat_Mode_PP(boolean immediateMode){

   if(ecat_OpMode(profPos)){
      pthread_mutex_lock(&shared.control);
      if(immediateMode) shared.coeCtrlWord |= 0b00100000;
      else shared.coeCtrlWord &= ~0b00100000;
      pthread_mutex_unlock(&shared.control);
      return TRUE;
   }
   return FALSE;
}

boolean ecat_OpMode_PV(){
   return ecat_OpMode(profVel);
}

boolean ecat_OpMode_PT(){
   return ecat_OpMode(profTor);
}

boolean ecat_OpMode_SyncP(){
   return ecat_OpMode(syncPos);
}

boolean ecat_OpMode_IP(){

   if(ecat_OpMode(intPos)){
      pthread_mutex_lock(&shared.control);
      shared.coeCtrlWord |= 0b00010000;
      pthread_mutex_unlock(&shared.control);
      return TRUE;
   }
   return FALSE;
}

int ecat_Home(int HOME_MODE, int HOME_DIR, int speed, int acceleration, int HOME_DIST, int HOME_P, int timeout_ms){

   uint sdoBuff, prevMode, err = 0;
   struct timespec stopTimeout;

   pthread_mutex_lock(&shared.control);
   prevMode = shared.mode;
   pthread_mutex_unlock(&shared.control);

   sdoBuff = HOME_MODE;
   ec_SDOwrite(1, HM_MODE , FALSE, 1, &sdoBuff, EC_TIMEOUTRXM);
   
   // HOME.DIR
   sdoBuff = HOME_DIR;
   ec_SDOwrite(1, HM_DIR , FALSE, 1, &sdoBuff, EC_TIMEOUTRXM); 

   // HOME.V
   sdoBuff = speed;
   ec_SDOwrite(1, HM_V , FALSE, 1, &sdoBuff, EC_TIMEOUTRXM); 

   // HOME.ACC/HOME.DEC
   sdoBuff = acceleration;
   ec_SDOwrite(1, HMACCEL , FALSE, 4, &sdoBuff, EC_TIMEOUTRXM);

   // HOME.DIST
   sdoBuff = HOME_DIST;
   ec_SDOwrite(1, HM_DIST , FALSE, 4, &sdoBuff, EC_TIMEOUTRXM);

   // HOME.P
   sdoBuff = HOME_P;
   ec_SDOwrite(1, HM_P , FALSE, 4, &sdoBuff, EC_TIMEOUTRXM); 

   
   if(!ecat_OpMode(homing)) return FALSE;

   if(!ecat_Update(TRUE, timeout_ms)) return FALSE;

   if(!ecat_OpMode(prevMode)) return FALSE;

   return TRUE;
}

boolean ecat_Fault(){
   pthread_mutex_lock(&shared.control);
   boolean fault = (shared.coeStatus & 0b1000) != 0;
   pthread_mutex_unlock(&shared.control);
   return fault;
}



int main(int argc, char *argv[])
{
   printf("Starting simpleMtrCtrl wrapper for acrEcat\n");


   
   int currentgroup = 0;

   if (argc > 1)
   {

      

      struct __attribute__((__packed__)){ 
         //0x1722
         //rxPDOs
         //uint16   ctrlWord; 
         //uint32  targetPos;
         //uint16   latchCtrl;
         //int16    tqFdFwd;
         //uint16   digOutputs;
         //uint16   maxTorque;

         //0x1725
         //rxPDOs
         uint16   ctrlWord; 
         uint32   targetPos;
         uint32   digOutputs;
         uint16   tqFdFwd;         
         uint16   maxTorque;

         //0x1702
         //uint32   targetVel;
         //uint16   ctrlWord;

         //0x1B20
         //txPDOs
         int32    posActual;
         int32    posFdback2;
         int32    velActual;
         uint32   digInputs;
         int32    followErr;
         uint16   latchPos;
         uint16   coeStatus;
         int16    tqActual;
         uint16   latchStatus;
         int16    analogInput;
      } PDOs = {0};


      ecat_Init(argv[1], &PDOs, sizeof(PDOs), rxPDOFixed, txPDOFixed);
      osal_usleep(10000);
      printf("\nWaiting for fault to clear!\n");
      while(!ecat_Fault()){}
      printf("\nFault cleared!\n");
      while(!ecat_Enable()){
         printf("\nEnable failed\n");
      }
      while(!ecat_Mode_PP(FALSE)){
         printf("\nMode switch failed\n");
      }
      
      ecat_Home(0, 0, 60, 1000, 0, 0, 1000);
      
      printf("\nHomed\n");


      PDOs.maxTorque = 1000;
      //PDOs.targetVel = 60*1000;
      /*for(int i = 0, pos = 0; i < 10; i++, pos += 100){
         PDOs.targetPos = pos;
         ecat_Update(TRUE);
         osal_usleep(1000000);
      }*/

      PDOs.targetPos = 360;
      ecat_Update(TRUE, 10000000);
      printf("\nSetpoint set\n");
      osal_usleep(10000000);

     ecat_Shutdown();
      
   }
   else
   {
      ec_adaptert * adapter = NULL;
      printf("Usage: simple_test ifname1\nifname = eth0 for example\n");

      printf ("\nAvailable adapters:\n");
      adapter = ec_find_adapters ();
      while (adapter != NULL)
      {
         printf ("    - %s  (%s)\n", adapter->name, adapter->desc);
         adapter = adapter->next;
      }
      ec_free_adapters(adapter);
   }


   printf("End program\n");
   return 0;
}
