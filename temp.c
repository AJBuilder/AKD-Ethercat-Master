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
#define rxPDOFixed   0x1725

#define txPDOEnable 0x1C13, 0
#define txPDOAssign1 0x1C13, 1
#define txPDOFixed   0x1B20

// Control and status objects
#define COControl    0x6040, 0
#define MODEofOP     0x6060, 0
#define MnfStatus    0x1002, 0
#define COStatus     0x6041, 0

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
#define SYNC_DIST (1000*1000) // 1ms

#define OPMODE 1

// Operation
#define OPTIME 100; //100*100ms = 10sec
#define DEBUG_MODE TRUE





struct rxPDOs{ //0x1725
   //rxPDOs
   uint16*   ctrlWord;
   int32*    targetPos;
   uint16*   digOutput;
   int16*    tqFdFwd;
   uint16*   maxTorque;   
} ;

struct txPDOs{ //0x1B20
   //txPDOs
   int32*    posActual;
   int32*    posFdback2;
   int32*    velActual;
   uint32*   digInputs;
   uint32*   followErr;
   uint32*   latchPos;
   uint16*   coeStatus;
   int16*    tqActual;
   uint16*   latchStatus;
} ;

// PDO data structure
typedef struct {
   void **toSlave;
   void **fromSlave;
} PDOS_T; //336bits, 42bytes

struct ecatData_T{
   
   // Init
   char* ifname;

   // PDO buffers
   char IOmap[4096];
   PDOS_T pointerMap;

   // Control signals
   uint64 gl_prevDCtimeb, curDCtimeb;
   int8 inSyncCount, noDCCount, wrkCounter, expectedWKC;
   boolean inOP, shutdown;

   // Debug 
   
   int64 gl_toff, gl_delta;
   uint8 gl_integral;

   // Threading
   pthread_mutex_t debug, pdos, control;
   pthread_t talker, controller;
};






int writeDatamap(void** map, void* data, int bytes){
   
   switch(bytes){ // Assign to map
      case 1 : *(uint8  *)*map = *(uint8  *)data; break; //| (*(uint8  *)*map & (~mask))
      case 2 : *(uint16 *)*map = *(uint16 *)data; break;
      case 4 : *(uint32 *)*map = *(uint32 *)data; break;
      case 8 : *(uint64 *)*map = *(uint64 *)data; break;
      default : return 0;
   }
   *(uint8 *)map += bytes; // Increment to next entry

   return bytes * 8;
}

int readDatamap(void** map, void* data, int bytes){

   switch(bytes){ // Read from map
      case 1 : *(uint8  *)data = *(uint8  *)*map; break;
      case 2 : *(uint16 *)data = *(uint16 *)*map; break;
      case 4 : *(uint32 *)data = *(uint32 *)*map; break;
      case 8 : *(uint64 *)data = *(uint64 *)*map; break;
      default : return 0;
   }
   *(uint8 *)map += bytes; // Increment to next entry

   return bytes * 8;
}



/* add ns to timespec */
void add_timespec(struct timespec *ts, int64 addtime)
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
boolean ec_sync(int64 reftime, uint64 cycletime , int64 *offsettime, int64 dist, int64 window, int64 *d, int64 *i)
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
void ecatTalker(struct ecatData_T* shared)
{

   // Local variables
   struct timespec   ts, tleft;
   uint64 prevDCtime;
   void*    output_ptr;   
   void*    input_ptr;

   // Buffers (To outside of thread)
   int8 inSyncCountb, noDCCountb;
   int64 toff, db_delta, db_integral;


   // CONTROL LOCKED
   pthread_mutex_lock(&shared->control);
   // PDOS LOCKED
   //pthread_mutex_lock(&shared->pdos);

   clock_gettime(CLOCK_MONOTONIC, &ts);
   ec_send_processdata();
   while(!shared->shutdown)
   {
      pthread_mutex_unlock(&shared->control);
      // CONTROL UNLOCKED
      shared->wrkCounter = ec_receive_processdata(EC_TIMEOUTRET);
      //pthread_mutex_unlock(&shared->pdos);
      /// PDOS UNLOCKED

      
      
      
      /* calculate next cycle start */
      add_timespec(&ts, CYCLE_NS+ toff);

      /* wait to cycle start */
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);

      /* calulate toff to get linux time and DC synced */
      if(ec_sync(ec_DCtime, CYCLE_NS, &toff, SYNC_DIST, SYNC_WINDOW_NS, &db_delta, &db_integral)){ // If withing sync window
         if (inSyncCountb < SYNC_AQTIME_NS / CYCLE_NS) inSyncCountb++; // Count number of cycles in window
      }
      else inSyncCountb = 0;

      #if DEBUG_MODE
      // Try to update debug info
      if(pthread_mutex_trylock(&shared->debug) == 0){
         shared->gl_toff = toff;
         shared->gl_delta = db_delta;
         shared->gl_integral = db_integral;
         shared->gl_curDCtime = ec_DCtime;
         shared->gl_prevDCtime = prevDCtime;
         pthread_mutex_unlock(&shared->debug);
      }
      #endif

      
      prevDCtime = ec_DCtime;
      
      // CONTROL LOCKED
      if(pthread_mutex_trylock(&shared->control) == 0){
         shared->inSyncCount = inSyncCountb;
         shared->noDCCount = noDCCountb;
      }
      // PDOS LOCKED
      //pthread_mutex_lock(&shared->pdos);
      ec_send_processdata();
   }
   pthread_mutex_unlock(&shared->control);
   //pthread_mutex_unlock(&shared->pdos);
}

void ecatController(struct ecatData_T* shared)
{
   printf("Starting Controller\n");  

   // Local variables
   uint16 prevcStatus = -1, currentcStatus;
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
   uint8 coeCurrent, coeRequest, currentgroup, opFrames;
   boolean finished = 0;


   // PDO Buffers
   uint16 ctrlWordb = 0;

   // Control Buffers
   int8 inSyncCountb, noDCCountb, wrkCounterb, expectedWKCb;
   boolean talkingb, inOPb, shutdownb;
   
   // CONTROL LOCKED
   pthread_mutex_lock(&shared->control);
   expectedWKCb = shared->expectedWKC;
   shared->inOP = FALSE;

   // PDOS LOCKED
   pthread_mutex_lock(&shared->pdos);

   while(!shared->shutdown)
   {
      inOPb = shared->inOP;
      inSyncCountb = shared->inSyncCount;
      noDCCountb = shared->noDCCount;
      wrkCounterb = shared->wrkCounter;
      pthread_mutex_unlock(&shared->control);
      // CONTROL UNLOCKED

      currentcStatus = *((uint16**)(shared->pointerMap.fromSlave))[6];
      pthread_mutex_unlock(&shared->pdos);
      // PDOS UNLOCKED

      osal_usleep(100000); // 100ms

      if(inOPb){

         // CANopen state machine
         switch(currentcStatus & 0b01101111){
            case QUICKSTOP :
            {
               coeCurrent = 7;
               coeRequest = 4;
               ctrlWordb = ctrlWordb & 0b100;
               break;
            }
            default :
            {
               switch(currentcStatus & 0b01001111){
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
                     ctrlWordb = (ctrlWordb & ~0b10000111) | 0b110; // Ready2Switch/Shutdown : 0bx---x110
                     break;
                  }
                  case RDY2SWITCH & 0b11011111:
                  {
                     coeCurrent = 2;
                     coeRequest = 1; // Switch on
                     if (finished) shutdownb = TRUE;
                     else ctrlWordb = (ctrlWordb & ~0b10000111) | 0b0111; // Switch-On : 0bx---x111
                     break;
                  }
                  case SWITCHEDON  & 0b11011111:
                  {
                     coeCurrent = 3;
                     if (finished) {
                        coeRequest = 0; // Switch off
                        ctrlWordb = (ctrlWordb & ~0b10001111) | 0b0110; // Ready2Switch : 0bx---x110
                     }
                     else {
                        coeRequest = 5; // Operation enable
                        ctrlWordb = (ctrlWordb & ~0b10001111) | 0b1111; // Operation Enabled : 0bx---1111
                     }
                     break;
                  }
                  case OP_ENABLED  & 0b11011111:
                  {
                     coeCurrent = 4;
                     if (opFrames >= 100) {
                        coeRequest = 4; // Disable operation
                        ctrlWordb = (ctrlWordb & ~0b10001111) | 0b0111; // Operation Disabled : 0bx---0111
                        finished = 1;
                     }else opFrames++;
                     break;
                  }
                  case FAULT :
                  {
                     coeCurrent = 5;
                     coeRequest = 6;
                     ctrlWordb = ctrlWordb | 0b10000000 ; // Clear Fault : 0b1---xxxx
                     break;
                  }
                  case FAULTREACT :
                  {
                     coeCurrent = 6;
                     coeRequest = 0;
                     ctrlWordb = (ctrlWordb & ~0b10000111) | 0b110; // Ready2Switch/Shutdown : 0bx---x110
                     break;
                  }
                  default :
                  {
                     coeCurrent = 8;
                     coeRequest = 0;
                     ctrlWordb = (ctrlWordb & ~0b10000111) | 0b110; // Shutddown : 0bx---x110
                     break;
                  }
               }
               
            }
         }
         if(prevcStatus != currentcStatus){
            printf("\nCoE State: %-24s (0x%04x)\tCoE Control: %-24s (0x%04x)\n", coeStateReadable[coeCurrent], currentcStatus, coeCtrlReadable[coeRequest], ctrlWordb);
            prevcStatus = currentcStatus;
         }

         if((wrkCounterb < expectedWKCb) || ec_group[currentgroup].docheckstate)
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

      // Check to see if DC master is alive.
      if(prevDCtime == ec_DCtime){
         noDCCountb++;
         if(noDCCountb > 20) shutdownb = TRUE;
      }
      else noDCCountb = 0;

      

      #if DEBUG_MODE
      pthread_mutex_lock(&shared->debug);
      
      printf("\r");
      printf("WKC: %2i(%2i)\t", wrkCounterb, expectedWKCb);
      printf("DiffDC: %12"PRIi64"\t", shared->curDCtimeb - shared->prevDCtimeb);
      printf("inSyncCount: %4i\t", shared->inSyncCount);
      //printf("coeStatus: 0x%04x\t", currentcStatus);
      //printf("CtrlWord: 0x%04x\t", ctrlWordb);

      if(shared->gl_toff*((shared->gl_toff>0) - (shared->gl_toff<0)) > (CYCLE_NS / 2)) printf("\tClock slipping! toff = %"PRIi64"  CYCLE_NS/2 = %d\n", shared->gl_toff, CYCLE_NS / 2);
      pthread_mutex_unlock(&shared->debug);
      fflush(stdout);
      #endif


      // CONTROL LOCKED
      pthread_mutex_lock(&shared->control);
      if(shutdownb) shared->shutdown++;

      // PDOS LOCKED
      pthread_mutex_lock(&shared->pdos);
      *((uint16**)(shared->pointerMap.toSlave))[0] = ctrlWordb;
   }
   pthread_mutex_unlock(&shared->pdos);
   pthread_mutex_unlock(&shared->control);

}

boolean ecatInit(struct ecatData_T* shared){

   printf("Starting EtherCAT Master\n");

   pthread_mutex_init(&shared->pdos, NULL);
   pthread_mutex_init(&shared->control, NULL);
   pthread_mutex_init(&shared->debug, NULL);


   uint32 sdoBuff;
   uint sdoBuffSize;
   boolean sdosNotConfigured = 1;
   uint8 inputFrameSize, outputFrameSize;
   uint8 *outputs, *inputs;
   int **tempTo, **tempFrom;
   char tempifname;
   

   /* initialise SOEM, bind socket to ifname */
   if (ec_init(shared->ifname))
   {
      printf("ec_init on %s succeeded.\n",shared->ifname);
      /* find and auto-config slaves */

       if ( ec_config_init(FALSE) > 0 )
      {
         // Initialize shared data
         tempTo = shared->pointerMap.toSlave;
         tempFrom = shared-> pointerMap.fromSlave;
         tempifname = shared->ifname;
         memset(shared, 0, sizeof(*shared));
         shared->pointerMap.toSlave = tempTo;
         shared-> pointerMap.fromSlave = tempFrom;
         shared->ifname = tempifname;

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
            /*
            ec_PDOassignt pdoConfig;
            ec_readPDOassign(1, pdoConfig);*/
            if(sdoBuff != OPMODE) sdosNotConfigured++;

            if(sdosNotConfigured != 0) printf("\rPDO assignments failed to configure %2d time(s). Retrying...\n", i);
         }

         if(sdosNotConfigured == 0) printf("PDO's configured!\n");
         else {
            printf("PDO's failed to configure.\n");
            return FALSE;
         }

         ecx_context.manualstatechange = 1;
         ec_config_map(&shared->IOmap);
         
         outputs = ec_slave[1].outputs;
         inputs = ec_slave[1].inputs;

         shared->pointerMap.toSlave[0] = (uint16*)outputs;  outputs += sizeof(*(uint16*)outputs);
         shared->pointerMap.toSlave[1] = (int32*)outputs;   outputs += sizeof(*(int32*)outputs);
         shared->pointerMap.toSlave[2] = (uint16*)outputs;  outputs += sizeof(*(uint16*)outputs);
         shared->pointerMap.toSlave[3] = (int16*)outputs;   outputs += sizeof(*(int16*)outputs);
         shared->pointerMap.toSlave[4] = (uint16*)outputs;  outputs += sizeof(*(uint16*)outputs);


         shared->pointerMap.fromSlave[0] = (int32*)inputs;  inputs += sizeof(*(int32*)inputs);
         shared->pointerMap.fromSlave[1] = (int32*)inputs;  inputs += sizeof(*(int32*)inputs);
         shared->pointerMap.fromSlave[2] = (int32*)inputs;  inputs += sizeof(*(int32*)inputs);
         shared->pointerMap.fromSlave[3] = (uint32*)inputs; inputs += sizeof(*(uint32*)inputs);
         shared->pointerMap.fromSlave[4] = (uint32*)inputs; inputs += sizeof(*(uint32*)inputs);
         shared->pointerMap.fromSlave[5] = (uint32*)inputs; inputs += sizeof(*(uint32*)inputs); 
         shared->pointerMap.fromSlave[6] = (uint16*)inputs; inputs += sizeof(*(uint16*)inputs);
         shared->pointerMap.fromSlave[7] = (int16*)inputs;  inputs += sizeof(*(int16*)inputs);
         shared->pointerMap.fromSlave[8] = (uint16*)inputs; inputs += sizeof(*(uint16*)inputs);
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
         shared->expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Calculated workcounter %d\n", shared->expectedWKC);
         
         /*for(int i = 0 ; i < 100 ; i++){ // Send 10,000 cycles of process data to tune other slaves
            ec_send_processdata();
            wkc = ec_receive_processdata(EC_TIMEOUTRET);
            printf("\rSending 100 process cycles: %5d WKC : %d", i, wkc);
         }*/

         /* Enable talker to handle slave PDOs in OP */
         pthread_create(&(shared->talker), NULL, &ecatTalker, shared);
         pthread_create(&shared->controller, NULL, &ecatController, shared);
         printf("Talker started! Synccount needs to reach : %"PRIi64"\n", (uint64)SYNC_AQTIME_NS / CYCLE_NS);
         while(shared->inSyncCount != (uint64)(SYNC_AQTIME_NS / CYCLE_NS)){
            if(shared->noDCCount > 20) return FALSE;
            //pthread_mutex_unlock(&shared->pdos);
            //osal_usleep(500000); // 500ms
            //pthread_mutex_lock(&shared->pdos);
         }
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
            shared->inOP = TRUE;
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
      printf("No socket connection on %s\nExecute as root\n", shared->ifname);
   }

   return FALSE;
}

boolean ecatStop(struct ecatData_T* shared){

   shared->shutdown = 1;
   pthread_join(shared->talker, NULL);
   pthread_join(shared->controller, NULL);

   printf("\nRequest init state for all slaves\n");
   ec_slave[0].state = EC_STATE_INIT;
   /* request INIT state for all slaves */
   ec_writestate(0);

   printf("End simple test, close socket\n");

   pthread_mutex_destroy(&shared->pdos);
   pthread_mutex_destroy(&shared->control);
   pthread_mutex_destroy(&shared->debug);

   /* stop SOEM, close socket */
   ec_close();
}





int main(int argc, char *argv[])
{
   printf("Starting simpleMtrCtrl wrapper for acrEcat\n");


   
   int currentgroup = 0;

   if (argc > 1)
   {

      

      struct ecatData_T data;
      
      data.ifname = argv[1];
      struct rxPDOs to;
      struct txPDOs from;
      data.pointerMap.toSlave = &to;
      data.pointerMap.fromSlave = &from;
      

      ecatInit(&data);
      *to.maxTorque = 5000;
      for(int i = 0; i < 20; i++){
         *to.targetPos = i * 1000;
         osal_usleep(1000000);
      }
      ecatStop(&data);
      
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
   return (0);
}
