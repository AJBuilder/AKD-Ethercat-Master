/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : simple_test [ifname1]
 * ifname is NIC interface, f.e. eth0
 *
 * This is a minimal test.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/queue.h>
#include <inttypes.h>
#include <pthread.h>

#include "ethercat.h"

// Timing
#define NSEC_PER_SEC 1000000000
#define EC_TIMEOUTMON 500

// PDO assignment objects
#define rxPDOEnable 0x1C12, 0
#define rxPDOAssign1 0x1C12, 1
#define rxPDOFixed   0x1702

#define txPDOEnable 0x1C13, 0
#define txPDOAssign1 0x1C13, 1
#define txPDOFixed   0x1B20

// Control and status objects
#define COControl    0x6040
#define MODEofOP     0x6060, 0
#define MnfStatus    0x1002
#define COStatus     0x6041

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

#define OPMODE 3

// Operation
#define DEBUG_MODE TRUE








// Left as struct to allow future support of multiple masters on different interfaces.
// Init would dynamically spawn new shared structs and add it to a static master list.
// Right now, one shared struct for one master is sufficient
static struct ecatData_T{ 
   
   // Init
   char* ifname;

   // PDO buffers
   char IOmap[4096];//[4096];
   int *pdoBuff;
   uint numOfPDOs;
   int coeCtrlIdx, coeStatusIdx;
   int outSizes[4*8 + 4], inSizes[4*8 + 4]; // 0 is num of entries to a PDO, followed by byte size of object entry.
                              // Repeats for each PDO
   // Assuming 1 byte minimum, up to 8 objects can be assigned per PDO. For PDOs max.

   // Control signals
   boolean inOP, update;
   uint inSyncCount;
   int8 wrkCounter, expectedWKC;
   uint16 coeCtrlWord, coeStatus;
   uint64 diffDCtime;
   enum modes{enable, disable, stop, shutdown} mode;

   // Debug 
   int64 gl_toff, gl_delta;
   uint8 gl_integral;

   // Threading
   pthread_mutex_t debug, control;
   pthread_cond_t updated;
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
static int ecatTalker()
{

   // Local variables
   struct timespec   ts;
   uint64 prevDCtime;
   int* dataToMap;
   char*    output_map_ptr;
   char*    input_map_ptr;
   int*     output_buff_ptr;
   int*     input_buff_ptr;

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

      shared.coeCtrlWord &= ~0b10000;

      
      if(pthread_mutex_trylock(&shared.control) == 0){
         // CONTROL LOCKED
         if(shared.mode == shutdown) {
            pthread_mutex_unlock(&shared.control);
            break;
         }
         shared.inSyncCount = inSyncCountb;
         shared.wrkCounter = wrkCounterb;
         shared.diffDCtime = ec_DCtime - prevDCtime;
         prevDCtime = ec_DCtime;
         //shared.coeCtrlWord |= 0b10000;
         
         

         if(shared.update){
            shared.update = FALSE;
            output_map_ptr = ec_slave[1].outputs;
            output_buff_ptr = shared.pdoBuff;
            for(int i = 1 ; i <= shared.outSizes[0] ; i++){ // FOR NOW FIRST PDO IS CTRL
               if(i == shared.coeCtrlIdx) cpyData(output_buff_ptr, &shared.coeCtrlWord, sizeof(shared.coeCtrlWord));
               cpyData(output_map_ptr, output_buff_ptr, shared.outSizes[i]);
               output_map_ptr += shared.outSizes[i];
               if(shared.outSizes[i] <= sizeof(int*)) output_buff_ptr++;
               else output_buff_ptr += 2;
            }

            input_map_ptr = ec_slave[1].inputs;
            input_buff_ptr = shared.pdoBuff + shared.outSizes[0] + 1;
            for(int i = 1 ; i <= shared.inSizes[0] ; i++){
               cpyData(input_buff_ptr, input_map_ptr, shared.inSizes[i]);
               if(i == shared.coeStatusIdx) cpyData(&shared.coeStatus, input_buff_ptr, sizeof(shared.coeStatus));
               input_map_ptr += shared.inSizes[i];
               if(shared.inSizes[i] <= sizeof(int*)) input_buff_ptr++;
               else input_buff_ptr += 2;
            }
            pthread_cond_signal(&shared.updated);
         }
         pthread_mutex_unlock(&shared.control);
      }
      
      ec_send_processdata();
   }

}

static void ecatController()
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
      if(shared.mode == shutdown) break;
      if(shared.diffDCtime == 0) noDCCount++;
      else noDCCount = 0;
      if(noDCCount > 20) shared.mode = stop;
      

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
                  {
                     coeCurrent = 2; 
                     coeRequest = 1; // Switch on 
                     if((shared.mode == enable) || (shared.mode = disable)) shared.coeCtrlWord = (shared.coeCtrlWord & ~0b10000111) | 0b0111; // Switch-On : 0bx---x111
                     break;
                  }
                  case SWITCHEDON  & 0b11011111:
                  {
                     coeCurrent = 3;
                     if((shared.mode == stop) || (shared.mode == shutdown)) {
                        coeRequest = 0; // Switch off
                        shared.coeCtrlWord = (shared.coeCtrlWord & ~0b10001111) | 0b0110; // Ready2Switch : 0bx---x110
                     }
                     else if(shared.mode == enable){
                        coeRequest = 5; // Operation enable
                        shared.coeCtrlWord = (shared.coeCtrlWord & ~0b10001111) | 0b1111; // Operation Enabled : 0bx---1111
                     }
                     break;
                  }
                  case OP_ENABLED  & 0b11011111:
                  {
                     coeCurrent = 4;
                     if (shared.mode != enable) {
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

      osal_usleep(50000); // 100ms
      
   }
   

}

boolean ecatUpdate(){
   static int err;
   static struct timespec updateTimeout;

   err = 0;
   pthread_mutex_lock(&shared.control);

   shared.update = TRUE;
   //shared.coeCtrlWord |= 0b110000;

   clock_gettime(CLOCK_MONOTONIC, &updateTimeout);
   updateTimeout.tv_sec += 1;
   err = pthread_cond_timedwait(&shared.updated, &shared.control, &updateTimeout); //Wait for talker thread to confirm update

   pthread_mutex_unlock(&shared.control);
   return err == 0; // Return TRUE if no error
}


boolean ecatInit(char *ifname, int* usrControl, int size){

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

         pthread_cond_init(&shared.updated, NULL);

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

            sdoBuff = rxPDOFixed;
            ec_SDOwrite(1, rxPDOAssign1, FALSE, 2, &sdoBuff, EC_TIMEOUTRXM); // Assign rxPDO1
            sdoBuff = txPDOFixed;
            ec_SDOwrite(1, txPDOAssign1, FALSE, 2, &sdoBuff, EC_TIMEOUTRXM); // Assign txPDO1

            sdoBuff = 1;
            ec_SDOwrite(1, rxPDOEnable, FALSE, 1, &sdoBuff, EC_TIMEOUTRXM); // Disable rxPDO
            ec_SDOwrite(1, txPDOEnable, FALSE, 1, &sdoBuff, EC_TIMEOUTRXM); // Disable txPDO

            sdoBuff = OPMODE;
            ec_SDOwrite(1, MODEofOP, FALSE, 1, &sdoBuff, EC_TIMEOUTRXM); // Assign Operational Mode

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
            if(sdoBuff != rxPDOFixed) sdosNotConfigured++;
            sdoBuffSize = 2;
            sdoBuff = 0;
            ec_SDOread(1, txPDOAssign1, FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTRXM); // Assign rxPDO1
            if(sdoBuff != txPDOFixed) sdosNotConfigured++;

            sdoBuffSize = 1;
            sdoBuff = 0;
            ec_SDOread(1, MODEofOP, FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTRXM);
            if(sdoBuff != OPMODE) sdosNotConfigured++;
            
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

         ecx_context.manualstatechange = 1;
         ec_config_map(&shared.IOmap);
         
         /*
         if(outputFrameSize != ec_slave[1].Obits){
            printf("\nGenerated output frame does not fit in slave's IOmap! IOmap: %d, oFrame: %d\n\n", ec_slave[1].Obits, outputFrameSize);
            return FALSE;
         }
         if(inputFrameSize != ec_slave[1].Ibits){
            printf("\nRead input frame does not fit in slave's IOmap! IOmap: %d, iFrame: %d\n\n", ec_slave[1].Ibits, inputFrameSize);
            return FALSE;
         }*/

         
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
         pthread_create(&shared.talker, NULL, &ecatTalker, NULL);
         pthread_create(&shared.controller, NULL, &ecatController, NULL);
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
				printf("\nOperational state reached for all slaves. Starting controller...\n");
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

boolean ecatStop(){

   pthread_mutex_lock(&shared.control);
   shared.mode = shutdown;
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

   pthread_cond_destroy(&shared.updated);

   /* stop SOEM, close socket */
   ec_close();
}





int main(int argc, char *argv[])
{
   printf("Starting simpleMtrCtrl wrapper for acrEcat\n");


   
   int currentgroup = 0;

   if (argc > 1)
   {


      struct { 
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
         /*uint16   ctrlWord; 
         uint32   targetPos;
         uint16   digOutputs;
         int16    tqFdFwd;         
         uint16   maxTorque;
         uint32   EMPTY;*/

         //0x1702
         uint32   targetVel;
         uint16   ctrlWord;

         //0x1B20
         //txPDOs
         int32    posActual;
         int32    posFdback2;
         int32    velActual;
         uint32   digInputs;
         uint32   followErr;
         uint32   latchPos;
         uint16   coeStatus;
         int16    tqActual;
         uint16   latchStatus;
         int16    analogInput;
      } PDOs;
      
      

      ecatInit(argv[1], &PDOs, sizeof(PDOs));
      //PDOs.maxTorque = 100;
      //PDOs.tqFdFwd = 100;
      PDOs.targetVel = 60*1000;
      for(int i = 0; i < 10000; i++){
         //PDOs.targetPos = PDOs.posActual + 10;

         ecatUpdate();
         osal_usleep(2000);
      }
      ecatStop();
      
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
