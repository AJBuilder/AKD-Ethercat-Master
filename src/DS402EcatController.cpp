
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/queue.h>
#include <inttypes.h>
#include <pthread.h>

#include "DS402EcatController.h"
#include "ethercat.h"







int DS402Controller::cpyData(void* dest, void* source, int bytes){

   switch(bytes){
      case 1: *(uint8*)dest = *(uint8*)source; break;
      case 2: *(uint16*)dest = *(uint16*)source; break;
      case 4: *(uint32*)dest = *(uint32*)source; break;
      case 8: *(uint64*)dest = *(uint64*)source; break;
      default: return -1; 
   }
return bytes;
}

void DS402Controller::readPDOAssignments(uint16 Slave, uint16 PDOassign, uint8* sizeList, uint* pdoAssignments, int* ctrlIndex, int* statIndex)
{
   uint16 idxloop, nidx, subidxloop, rdat, idx, subidx;
   uint8 subcnt, sizeIndex = 0;
   int wkc, rdl;
   int32 rdat2;

   rdl = sizeof(rdat); rdat = 0;
   /* read PDO assign subindex 0 ( = number of PDO's) */
   wkc = ec_SDOread(Slave, PDOassign, 0x00, FALSE, &rdl, &rdat, EC_TIMEOUTRXM);
   *pdoAssignments = rdat;
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
               if(((rdat2 & 0xFFFF0000) >> 16) == COControl) *ctrlIndex = subidxloop;
               if(((rdat2 & 0xFFFF0000) >> 16) == COStatus) *statIndex = subidxloop;
               sizeList[sizeIndex + subidxloop] = (rdat2 & 0xFF) / 8;
            }
         }
         sizeIndex += subidx + 1;
      }
   }
}


/* add ns to timespec */
void DS402Controller::add_timespec(struct timespec *ts, int64 addtime)
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
 bool DS402Controller::ec_sync(int64 reftime, uint64 cycletime , int64 *offsettime, int64 dist, int64 window, int64 *d, int64 *i)
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
void* DS402Controller::ecat_Talker(void* THIS)
{

   DS402Controller* This = (DS402Controller*)THIS;

   // Local variables
   struct timespec   ts;
   uint64 prevDCtime;
   int* dataToMap;
   uint8*   output_map_ptr;
   uint8*   input_map_ptr;
   uint8*   output_buff_ptr;
   uint8*   input_buff_ptr;

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
      This->add_timespec(&ts, CYCLE_NS+ toff);

      /* wait to cycle start */
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);

      /* calulate toff to get linux time and DC synced */
      if(This->ec_sync(ec_DCtime, CYCLE_NS, &toff, SYNC_DIST, SYNC_WINDOW_NS, &sync_delta, &sync_integral)){ // If withing sync window
         if (inSyncCountb != UINT32_MAX) inSyncCountb++; // Count number of cycles in window
      }
      else inSyncCountb = 0;

      #if DEBUG_MODE
      // Try to update debug info
      if(pthread_mutex_trylock(&This->debug) == 0){
         This->gl_toff = toff;
         This->gl_delta = sync_delta;
         This->gl_integral = sync_integral;
         pthread_mutex_unlock(&This->debug);
      }
      #endif

      
      
      if(pthread_mutex_trylock(&This->control) == 0){
         // CONTROL LOCKED
         if(This->masterState == ms_shutdown) {
            pthread_mutex_unlock(&This->control);
            break;
         }
         This->inSyncCount = inSyncCountb;
         This->wrkCounter = wrkCounterb;
         This->diffDCtime = ec_DCtime - prevDCtime;
         prevDCtime = ec_DCtime;


         // Update DS402 Control and Status variables for Controller
         for(int slaveNum = 0 ; slaveNum < ec_slavecount ; slaveNum++){
            This->cpyData(This->slaves[slaveNum].coeCtrlMapPtr, &This->slaves[slaveNum].coeCtrlWord, sizeof(This->slaves[slaveNum].coeCtrlWord));
            This->cpyData(&This->slaves[slaveNum].coeStatus, This->slaves[slaveNum].coeStatusMapPtr, sizeof(This->slaves[slaveNum].coeStatus));
            if(This->slaves[slaveNum].quickStop)
               This->slaves[slaveNum].coeCtrlWord &= ~0b100;
         }
         
         for(int slaveNum = 0 ; slaveNum < ec_slavecount ; slaveNum++){
            

            if(This->slaves[slaveNum].coeStatus & 0b01000000000000) {
               This->slaves[slaveNum].moveAck = TRUE;
               pthread_cond_signal(&This->moveSig);
            } else This->slaves[slaveNum].moveAck = FALSE;

            if(This->slaves[slaveNum].coeStatus & 0b10000000000000) {
               This->slaves[slaveNum].moveErr = TRUE;
               pthread_cond_signal(&This->moveSig);
            } else This->slaves[slaveNum].moveErr = FALSE;

            if(This->slaves[slaveNum].update){
               
               output_buff_ptr = This->slaves[slaveNum].outUserBuff;
               input_buff_ptr = This->slaves[slaveNum].inUserBuff;

               // Update outputs from user buffers to IOmap
               output_map_ptr = ec_slave[slaveNum].outputs;
               for(int i = 1 ; i <= This->slaves[slaveNum].outSizes[0] ; i++){ 
                  if(i != This->slaves[slaveNum].coeCtrlPos) This->cpyData(output_map_ptr, output_buff_ptr, This->slaves[slaveNum].outSizes[i]); // Copy data from user's input to IOmap. (Skipping Ctrl Word used by DS402Controller::Controller)
                  output_map_ptr += This->slaves[slaveNum].outSizes[i];
                  output_buff_ptr += This->slaves[slaveNum].outSizes[i];
               }

               // Update inputs from user buffers to IOmap
               input_map_ptr = ec_slave[slaveNum].inputs;
               for(int i = 1 ; i <= This->slaves[slaveNum].inSizes[0] ; i++){
                  This->cpyData(input_buff_ptr, input_map_ptr, This->slaves[slaveNum].inSizes[i]); // Copy input data to user's buffer
                  input_map_ptr += This->slaves[slaveNum].inSizes[i];
                  input_buff_ptr += This->slaves[slaveNum].inSizes[i];
               }   

               
               
            }
            else {
               if(This->slaves[slaveNum].mode == profPos || This->slaves[slaveNum].mode == homing) This->slaves[slaveNum].coeCtrlWord &= ~0b0010000; // Clear move bit. In homing : start_homing, In profPos : new_setpoint, In intPos : Interpolate
            }

            // Release caller of Update()
            This->slaves[slaveNum].update = FALSE;
            pthread_cond_signal(&This->IOUpdated);
         }
         
         

         pthread_mutex_unlock(&This->control);
         // CONTROL UNLOCKED
      }
      
      
      ec_send_processdata();
   }
   return nullptr;
}

void* DS402Controller::ecat_Controller(void* THIS)
{
   printf("ECAT: Controller Spawned\n");  

   // Local variables
   uint16 prevcStatus = -1;
   uint currentgroup, noDCCount;
   DS402Controller* This = (DS402Controller*)THIS;

   

   while(1)
   {
      // CONTROL LOCKED
      pthread_mutex_lock(&This->control);
      if(This->masterState == ms_shutdown){
         pthread_mutex_unlock(&This->control);  
         break;
      }
      if(This->diffDCtime == 0) noDCCount++;
      else noDCCount = 0;
      if(noDCCount > 20) This->masterState = ms_stop;
      
      
      for(int slaveNum = 0 ; slaveNum < ec_slavecount ; slaveNum++){

         // CANopen state machine
         switch(This->slaves[slaveNum].coeStatus & 0b01101111){
            case QUICKSTOP :
            {
               This->slaves[slaveNum].coeCurrentState = cs_QuickStop;
               if(!This->slaves[slaveNum].quickStop) {
                  This->slaves[slaveNum].coeStateTransition = cst_EnableOp;
                  This->slaves[slaveNum].coeCtrlWord |= 0b100; // Disable quickstop
               } 
               else This->slaves[slaveNum].coeStateTransition = cst_TrigQuickStop;
               break;
            }
            default :
            {
               switch(This->slaves[slaveNum].coeStatus & 0b01001111){
                  case NOTRDY2SWCH & 0b01001111: 
                  {
                     This->slaves[slaveNum].coeCurrentState = cs_NotReady;
                     This->slaves[slaveNum].coeStateTransition = cst_Shutdown; // Ready to switch
                     break;
                  }
                  case SWCHDISABLED & 0b01001111: 
                  {
                     This->slaves[slaveNum].coeCurrentState = cs_SwitchDisabled;
                     This->slaves[slaveNum].coeStateTransition = cst_Shutdown; // Ready to switch
                     This->slaves[slaveNum].coeCtrlWord = (This->slaves[slaveNum].coeCtrlWord & ~0b00001111) | 0b110; // Ready2Switch/Shutdown : 0bx---x110
                     break;
                  }
                  case RDY2SWITCH & 0b01001111:
                  { //Stop
                     This->slaves[slaveNum].coeCurrentState = cs_Ready; 
                     This->slaves[slaveNum].coeStateTransition = cst_SwitchOn; // Switch on
                     if((This->masterState == ms_enable) || (This->masterState = ms_disable)) This->slaves[slaveNum].coeCtrlWord = (This->slaves[slaveNum].coeCtrlWord & ~0b00001111) | 0b0111; // Switch-On : 0bx---x111
                     break;
                  }
                  case SWITCHEDON  & 0b01001111:
                  { // Disable
                     This->slaves[slaveNum].coeCurrentState = cs_SwitchedOn;
                     if((This->masterState == ms_stop) || (This->masterState == ms_shutdown)) {
                        This->slaves[slaveNum].coeStateTransition = cst_Shutdown; // Switch off
                        This->slaves[slaveNum].coeCtrlWord = (This->slaves[slaveNum].coeCtrlWord & ~0b00001111) | 0b0110; // Ready2Switch : 0bx---x110
                     }
                     else if(This->masterState == ms_enable){
                        This->slaves[slaveNum].coeStateTransition = cst_EnableOp; // Operation enable
                        This->slaves[slaveNum].coeCtrlWord = (This->slaves[slaveNum].coeCtrlWord & ~0b00001111) | 0b1111; // Operation Enabled : 0bx---1111
                     }
                     break;
                  }
                  case OP_ENABLED  & 0b11011111:
                  { // Enable
                     This->slaves[slaveNum].coeCurrentState = cs_OpEnabled;
                     if (This->masterState != ms_enable) {
                        This->slaves[slaveNum].coeStateTransition = cst_DisableOp; // Disable operation
                        This->slaves[slaveNum].coeCtrlWord = (This->slaves[slaveNum].coeCtrlWord & ~0b00001111) | 0b0111; // Operation Disabled : 0bx---0111
                     }
                     break;
                  }
                  case FAULT :
                  {
                     This->slaves[slaveNum].coeCurrentState = cs_Fault;
                     This->slaves[slaveNum].coeStateTransition = cst_DisableOp;
                     This->slaves[slaveNum].coeCtrlWord = (This->slaves[slaveNum].coeCtrlWord & ~0b00001111) | 0b0111; // Operation Disabled : 0bx---0111
                     break;
                  }
                  case FAULTREACT :
                  {
                     This->slaves[slaveNum].coeCurrentState = cs_FaultReaction;
                     This->slaves[slaveNum].coeStateTransition = cst_Shutdown;
                     break;
                  }
                  default :
                  {
                     This->slaves[slaveNum].coeCurrentState = cs_Unknown;
                     This->slaves[slaveNum].coeStateTransition = cst_Shutdown;
                     This->slaves[slaveNum].coeCtrlWord = (This->slaves[slaveNum].coeCtrlWord & ~0b00001111) | 0b110; // Shutddown : 0bx---x110
                     break;
                  }
               }
            }
         }
         #if DEBUG_MODE
         if(prevcStatus != This->slaves[slaveNum].coeStatus){
            printf("\nCoE State: %-24s (0x%04x)\tCoE Control: %-24s (0x%04x)\n", This->coeStateReadable[This->slaves[slaveNum].coeCurrentState], This->slaves[slaveNum].coeStatus, This->coeCtrlReadable[This->slaves[slaveNum].coeStateTransition], This->slaves[slaveNum].coeCtrlWord);
            prevcStatus = This->slaves[slaveNum].coeStatus;
         }
         #endif
      }

      if(This->inOP && (This->wrkCounter < This->expectedWKC) || ec_group[currentgroup].docheckstate)
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
        
      
      
      #if DEBUG_MODE
      pthread_mutex_lock(&This->debug);
      
      printf("\r");
      printf("WKC: %2i(%2i)\t", This->wrkCounter, This->expectedWKC);
      printf("DiffDC: %12" PRIi64 "\t", This->diffDCtime);
      printf("inSyncCount: %4i\t", This->inSyncCount);
      //printf("coeStatus: 0x%04"PRIx16"\t", This->coeStatus);
      //printf("CtrlWord: 0x%04"PRIx16"\t", This->coeCtrlWord);

      if(This->gl_toff*((This->gl_toff>0) - (This->gl_toff<0)) > (CYCLE_NS / 2)) printf("\tClock slipping! toff = %" PRIi64 "  CYCLE_NS/2 = %d\n", This->gl_toff, CYCLE_NS / 2);
      pthread_mutex_unlock(&This->debug);
      fflush(stdout);
      #endif

      pthread_mutex_unlock(&This->control);
      // CONTROL UNLOCKED

      pthread_cond_signal(&This->stateUpdated);

      osal_usleep(50000); // 100ms
      
   }
   
   return nullptr;
}

bool DS402Controller::ecat_Init(char *ifname){
   if (ec_init(ifname))
   {
      printf("ECAT: ec_init on %s succeeded.\n",this->ifname);
      /* find and auto-config slaves */

      if( ec_config_init(FALSE) > 0 ) {
         initialized = TRUE;
         return TRUE;
      }
      else
         printf("ECAT: No slaves found!\n");
   }
   else
      printf("ECAT: No socket connection on %s. Execute as root!\n", this->ifname);

      return FALSE;
}

bool DS402Controller::ecat_Start(void* usrControl, int size){

   printf("ECAT: Starting EtherCAT Master\n");

   uint32 sdoBuff;
   int sdoBuffSize;
   int sdosNotConfigured = 1;
   int largestOut = 0, largestIn = 0, totalBytes = 0;

   if(!this->initialized){
      printf("ECAT: Not initialized. Calling ecat_Init()\n");
      return FALSE;
   }
   if(this->PDOAssignments.size() == 0){
      printf("ECAT: PDOs not configured. Call confSlavePDOs()\n");
      return FALSE;
   }
   if(this->PDOAssignments.size() != ec_slavecount){
      printf("ECAT: Not all PDOs configured. Call confSlavePDOs() for each slave. Use slaveCount() for number of found devices.\n");
      return FALSE;
   }
      
   // Set to stop state
   this->masterState = ms_stop;


   // Configure PDO assignments
   printf("ECAT: %d slaves found. Configuring PDO assigments...\n",ec_slavecount);


   for(int slaveNum = 1 ; slaveNum <= ec_slavecount ; slaveNum++){
      
      for(int i = 1; sdosNotConfigured != 0 && i <=10 ; i++){
         if(i != 1) osal_usleep(100000); // If looping, wait 100ms
         sdosNotConfigured = 0;

         // Assign PDOs
         sdoBuff = 0;
         ec_SDOwrite(slaveNum, rxPDOEnable, FALSE, 1, &sdoBuff, EC_TIMEOUTRXM); // Disable rxPDO
         ec_SDOwrite(slaveNum, txPDOEnable, FALSE, 1, &sdoBuff, EC_TIMEOUTRXM); // Disable txPDO

         for(int i = 0 ; i < this->PDOAssignments[slaveNum - 1].pdos ; i++){
            sdoBuff = this->PDOAssignments[slaveNum - 1].rxPDO[i];
            ec_SDOwrite(slaveNum, rxPDOAssign1, i + 1, FALSE, 2, &sdoBuff, EC_TIMEOUTRXM); // Assign rxPDO1
            sdoBuff = this->PDOAssignments[slaveNum - 1].txPDO[i];
            ec_SDOwrite(slaveNum, txPDOAssign1, i + 1, FALSE, 2, &sdoBuff, EC_TIMEOUTRXM); // Assign txPDO1
         }

         sdoBuff = this->PDOAssignments[slaveNum - 1].pdos;
         ec_SDOwrite(slaveNum, rxPDOEnable, FALSE, 1, &sdoBuff, EC_TIMEOUTRXM); // Enable rxPDO
         ec_SDOwrite(slaveNum, txPDOEnable, FALSE, 1, &sdoBuff, EC_TIMEOUTRXM); // Enable txPDO

         // Verify Assignments
         sdoBuffSize = 1;
         sdoBuff = 0;
         ec_SDOread(slaveNum, rxPDOEnable, FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTRXM); // Check rxPDO enabled
         if(sdoBuff != 1) sdosNotConfigured++;
         sdoBuffSize = 1;
         sdoBuff = 0;
         ec_SDOread(slaveNum, txPDOEnable, FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTRXM); // Check txPDO enabled
         if(sdoBuff != 1) sdosNotConfigured++;

         for(int i = 0 ; i < this->PDOAssignments[slaveNum - 1].pdos ; i++){
            sdoBuffSize = 2;
            sdoBuff = 0;
            ec_SDOread(slaveNum, rxPDOAssign1, i + 1, FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTRXM); // Read rxPDO1
            if(sdoBuff != this->PDOAssignments[slaveNum - 1].rxPDO[i]) sdosNotConfigured++;

            sdoBuffSize = 2;
            sdoBuff = 0;
            ec_SDOread(slaveNum, txPDOAssign1, i + 1, FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTRXM); // Read txPDO1
            if(sdoBuff != this->PDOAssignments[slaveNum - 1].txPDO[i]) sdosNotConfigured++;
         }

         if(sdosNotConfigured != 0) printf("\rECAT: PDO assignments failed to configure %2d time(s). Retrying...  ", i);
      }
      if(sdosNotConfigured == 0) printf("PDO's configured!\n");
      else {
         printf("PDO's failed to configure.\n");
         return FALSE;
      }

      // Configure Homing settings
      sdoBuff = 0;
      ec_SDOwrite(slaveNum, HM_AUTOMOVE , FALSE, 1, &sdoBuff, EC_TIMEOUTRXM); // Disable automove

      sdoBuff = 6;
      ec_SDOwrite(slaveNum, QSTOP_OPT , FALSE, 2, &sdoBuff, EC_TIMEOUTRXM); // Set Quickstop option
      
   }


   ecx_context.manualstatechange = 1;
   ec_config_map(&this->IOmap);
   

   for(int slaveNum = 1 ; slaveNum <= ec_slavecount ; slaveNum++){
      if(ec_slave[slaveNum].Obytes > largestOut) largestOut = ec_slave[slaveNum].Obytes;
      if(ec_slave[slaveNum].Ibytes > largestIn) largestIn = ec_slave[slaveNum].Ibytes;
      totalBytes += ec_slave[slaveNum].Obytes;
      totalBytes += ec_slave[slaveNum].Ibytes;
   }
   if(size != totalBytes) {
      printf("Input struct is of incorrect size. Is %i. Needs to be %i", size, totalBytes);
      return FALSE;
   }

   // Allocate slave data
   this->slaves = new ecat_slave [ec_slavecount]();

   for(int slaveNum = 1 ; slaveNum <= ec_slavecount ; slaveNum++){
      slaves[slaveNum - 1].outSizes = new uint8 [largestOut + 1];
      slaves[slaveNum - 1].inSizes  = new uint8 [largestIn  + 1];

      readPDOAssignments(slaveNum, 0x1C12, this->slaves[slaveNum - 1].outSizes, &sdoBuff, &(this->slaves[slaveNum - 1].coeCtrlPos), &(this->slaves[slaveNum - 1].coeStatusPos));
      this->slaves[slaveNum - 1].numOfPDOs += sdoBuff;
      readPDOAssignments(slaveNum, 0x1C13, this->slaves[slaveNum - 1].inSizes, &sdoBuff, &(this->slaves[slaveNum - 1].coeCtrlPos), &(this->slaves[slaveNum - 1].coeStatusPos));
      this->slaves[slaveNum - 1].numOfPDOs += sdoBuff;
   }

   // Find IOmap info and build user buff info
   for(int slaveNum = 1 ; slaveNum <= ec_slavecount ; slaveNum++){
   
      // Populate User Buffer PDO maps
      this->slaves[slaveNum - 1].outUserBuff = (uint8*)usrControl + (ec_slave[slaveNum].outputs - this->IOmap);
      this->slaves[slaveNum - 1].inUserBuff = (uint8*)usrControl + (ec_slave[slaveNum].inputs - this->IOmap);

      // Find where the DS402 CoE control word is in IOmap
      this->slaves[slaveNum - 1].coeCtrlMapPtr = ec_slave[slaveNum].outputs;
      for(int i = 1; i < this->slaves[slaveNum - 1].coeCtrlPos; i++){
         this->slaves[slaveNum - 1].coeCtrlMapPtr += this->slaves[slaveNum - 1].outSizes[i]; 
      }

      // Find where the DS402 CoE status word is in IOmap
      this->slaves[slaveNum - 1].coeStatusMapPtr = ec_slave[slaveNum].inputs;
      for(int i = 1; i < this->slaves[slaveNum - 1].coeStatusPos; i++){
         this->slaves[slaveNum - 1].coeStatusMapPtr += this->slaves[slaveNum - 1].inSizes[i]; 
      }
   }

   
   // Config DC (In PREOP)
   printf("ECAT: Configuring DC. Should be in PRE-OP. ");
   printf(ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE * 4) == EC_STATE_PRE_OP ? "State: PRE_OP\n" : "ERR! NOT IN PRE_OP!\n") ;
   printf(ec_configdc() ? "ECAT: Found slaves with DC.\n" : "ECAT: Did not find slaves with DC.\n") ;
   ec_dcsync0(1, FALSE, CYCLE_NS, 0);
   
   
   
   // Syncing (In SAFEOP)
   printf("ECAT: Slaves mapped, state to SAFE_OP... ");
   ec_slave[0].state = EC_STATE_SAFE_OP;
   ec_writestate(0);
   printf(ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4) == EC_STATE_SAFE_OP ? "State: SAFE_OP\n" : "ERR! NOT IN SAFE_OP!\n") ;
   this->expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
   printf("ECAT: Calculated workcounter %d\n", this->expectedWKC);
   
   /*for(int i = 0 ; i < 100 ; i++){ // Send 10,000 cycles of process data to tune other slaves
      ec_send_processdata();
      wkc = ec_receive_processdata(EC_TIMEOUTRET);
      printf("\rSending 100 process cycles: %5d WKC : %d", i, wkc);
   }*/

   // Initialize threading resources
   pthread_mutex_init(&this->control, NULL);
   pthread_mutex_init(&this->debug, NULL);
   pthread_cond_init(&this->IOUpdated, NULL);
   pthread_cond_init(&this->stateUpdated, NULL);
   pthread_cond_init(&this->moveSig, NULL);

   /* Enable talker to handle slave PDOs in OP */
   pthread_create(&this->talker, NULL, &DS402Controller::ecat_Talker, this);
   pthread_create(&this->controller, NULL, &ecat_Controller, this);
   printf( "ECAT: Talker started! Synccount needs to reach : %" PRIi64 "\n", (uint64)SYNC_AQTIME_NS / CYCLE_NS);
   pthread_mutex_lock(&this->control);
   while(this->inSyncCount < (uint64)(SYNC_AQTIME_NS / CYCLE_NS)){
      pthread_mutex_unlock(&this->control);
      osal_usleep(100000); // 100ms
      pthread_mutex_lock(&this->control);
   }
   pthread_mutex_unlock(&this->control);
   printf("\nECAT: Master within sync window. Enabling sync0 generation!\n");
   ec_dcsync0(1, TRUE, CYCLE_NS, 0); //Enable sync0 generation


   // Go into OPERATIONAL
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
      for(int i = 1; i<=ec_slavecount ; i++)
      {
         ec_dcsync0(i+1, FALSE, 0, 0); // SYNC0,1
         if(ec_slave[i].state != EC_STATE_OPERATIONAL)
         {
            printf("ECAT: Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                  i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
         }
      }
   }

   

   return FALSE;
}

// Blocks for timeout time
int DS402Controller::Update(uint slave, bool move, int timeout_ms){
   int err = 0, slaveNum;
   struct timespec timeout, temptime;
   bool allUpdated = FALSE;

   clock_gettime(CLOCK_REALTIME, &timeout);
   add_timespec(&timeout, (int64)timeout_ms * 1000000 * 100);

   pthread_mutex_lock(&this->control);

   if(slave != 0) slaveNum = slave - 1;
   else slaveNum = 0;
   do{ // Set update flag (And possibly bit 4)

      this->slaves[slaveNum].update = TRUE;
      if(move && (slaves[slaveNum].mode == homing || slaves[slaveNum].mode == profPos || slaves[slaveNum].mode == intPos) ) 
         this->slaves[slaveNum].coeCtrlWord |= 0b0010000; // Should only be used if in homing, interpolated or profPos mode.

      slaveNum++;
   } while((slaveNum < ec_slavecount) && (slave == 0));

   allUpdated = FALSE;
   while(!allUpdated && err == 0){ // Check and make sure all slave update flags were recognized.

      err = pthread_cond_timedwait(&this->IOUpdated, &this->control, &timeout); //Wait for talker thread to confirm update to IOmap

      allUpdated = TRUE;
      if(slave != 0) slaveNum = slave - 1;
      else slaveNum = 0;
      do{ 
         if (this->slaves[slaveNum].update == TRUE){
            allUpdated = FALSE;
            break;
         }

         slaveNum++;
      }while((slaveNum < ec_slavecount) && (slave == 0));

   }

   allUpdated = FALSE;
   while(!allUpdated && err == 0 && move){ // Check to see if update was acknowledged or error

      err = pthread_cond_timedwait(&this->moveSig, &this->control, &timeout); //Wait for talker thread to confirm update to IOmap

      allUpdated = TRUE;
      if(slave != 0) slaveNum = slave - 1;
      else slaveNum = 0;
      do{
         if (this->slaves[slaveNum].moveAck == FALSE && this->slaves[slaveNum].mode != intPos){
            allUpdated = FALSE;
            break;
         }
         if(this->slaves[slaveNum].moveErr) {
            return DS402_MOVEERR;
            break;
         }

         slaveNum++;
      }while((slaveNum < ec_slavecount) && (slave == 0));
         
   }
   
   pthread_mutex_unlock(&this->control);
   return err; // Return err
}
 
 // Blocks for max of 1 sec;
bool DS402Controller::State(ecat_masterStates reqState){

   struct timespec timeout;
   uint sdoBuffSize, sdoBuff;
   int err = 0;
   bool allStatesChanged = FALSE;
   ecat_coeStates waitingFor;
   ecat_masterStates oldState;

   pthread_mutex_lock(&this->control);
   if(this->masterState != reqState){
         if(reqState != ms_shutdown){

            oldState = this->masterState;
            this->masterState = reqState;

            clock_gettime(CLOCK_REALTIME, &timeout);
            timeout.tv_sec += 1; // 1 sec timeout
            
            switch(reqState){ //Wait for talker thread to confirm update
                  case ms_stop :
                     waitingFor = cs_Ready;
                  break;
                  case ms_disable :
                     waitingFor = cs_SwitchedOn;
                  break;
                  case ms_enable :
                     waitingFor = cs_OpEnabled;
                  break;
                  default :
                     err = -1;
                  break;
            }

            while(!allStatesChanged && err == 0){
               allStatesChanged = TRUE;
               for(int slaveNum = 0 ; slaveNum < ec_slavecount ; slaveNum++){
                  if (this->slaves[slaveNum].coeCurrentState != waitingFor){
                     
                     if(this->slaves[slaveNum].coeCurrentState == cs_Fault){
                        pthread_mutex_unlock(&this->control);
                        this->masterState = oldState; // Reset state
                        return FALSE; // If a slave is in fault, return 
                     }

                     allStatesChanged = FALSE;
                     break;
                  }
               }
               err = pthread_cond_timedwait(&this->stateUpdated, &this->control, &timeout);
            }
         }
        else{// Shutdown threads
            this->masterState = ms_shutdown;
            pthread_mutex_unlock(&this->control);
            pthread_join(this->talker, NULL);
            pthread_join(this->controller, NULL);

            printf("\nECAT: Requesting init state for all slaves\n");
            ec_slave[0].state = EC_STATE_INIT;
            /* request INIT state for all slaves */
            ec_writestate(0);

            printf("ECAT: End simple test, close socket\n");

            pthread_mutex_destroy(&this->control);
            pthread_mutex_destroy(&this->debug);

            pthread_cond_destroy(&this->IOUpdated);
            pthread_cond_destroy(&this->stateUpdated);
            pthread_cond_destroy(&this->moveSig);

            /* stop SOEM, close socket */
            ec_close();
        }
    }

   pthread_mutex_unlock(&this->control);
   return err == 0;

}

bool DS402Controller::Enable(){
   return DS402Controller::State(ms_enable);
}

bool DS402Controller::Disable(){
   return DS402Controller::State(ms_disable);
}

bool DS402Controller::Stop(){
   return DS402Controller::State(ms_stop);
}

bool DS402Controller::Shutdown(){
   return DS402Controller::State(ms_shutdown);
}

// Blocking until quick stop engaged
bool DS402Controller::QuickStop(uint slave, bool enableQuickStop){ 
   pthread_mutex_lock(&this->control);
   if(enableQuickStop){
      this->slaves[slave-1].quickStop = TRUE;
      while(this->slaves[slave].coeCurrentState != cs_QuickStop){
         pthread_cond_wait(&this->stateUpdated, &this->control);
      }
   } else {
      this->slaves[slave-1].quickStop = FALSE;
      while(this->slaves[slave].coeCurrentState != cs_OpEnabled){
         pthread_cond_wait(&this->stateUpdated, &this->control);
      }
   }
   pthread_mutex_unlock(&this->control);
   return TRUE;
}

// Can block for 1 sec
bool DS402Controller::setOpMode(uint slave, ecat_OpModes reqMode){

   int sdoBuffSize, sdoBuff, slaveNum;
   struct timespec timeout, curtime;
   clock_gettime(CLOCK_MONOTONIC, &timeout);
   timeout.tv_sec += 1;

   
   if(DS402Controller::State(ms_disable)){ // Don't assign if not disabled 
      
      pthread_mutex_lock(&this->control);
      
      if(slave != 0) slaveNum = slave - 1;
      else slaveNum = 0;
      do{

         if(this->slaves[slaveNum].mode != reqMode){  
   
            do{
               sdoBuffSize = 1;
               sdoBuff = reqMode;
               ec_SDOwrite(slaveNum, REQOPMODE, FALSE, sdoBuffSize, &sdoBuff, EC_TIMEOUTRXM); // Assign Operational Mode

               sdoBuffSize = 1; // Check if drive has acknowledged
               sdoBuff = 0;
               ec_SDOread(slaveNum, ACTOPMODE, FALSE, &sdoBuffSize, &sdoBuff, EC_TIMEOUTRXM);

               clock_gettime(CLOCK_MONOTONIC, &curtime);
               if(curtime.tv_sec > timeout.tv_sec){
                  pthread_mutex_unlock(&this->control);
                  return FALSE;
               }

            } while(sdoBuff != reqMode && curtime.tv_sec < timeout.tv_sec);

            this->slaves[slaveNum].mode = reqMode;
         }

         slaveNum++;
      }while((slaveNum < ec_slavecount) && (slave == 0));

      pthread_mutex_unlock(&this->control);
      DS402Controller::State(ms_enable);
   }

   
   
   return TRUE;
}

bool DS402Controller::ConfProfPosImm(uint slave, bool moveImmediate_u){
   pthread_mutex_lock(&this->control);
   /*
      Configure Profile Position settings
   sdoBuff = 10;
   ec_SDOwrite(1, MT_ACC , FALSE, 4, &sdoBuff, EC_TIMEOUTRXM); // Set acceleration for profile position mode
   sdoBuff = 10;
   ec_SDOwrite(1, MT_DEC , FALSE, 4, &sdoBuff, EC_TIMEOUTRXM); // Set acceleration for profile position mode
               sdoBuff = 10;
   ec_SDOwrite(1, MT_V, FALSE, 4, &sdoBuff, EC_TIMEOUTRXM); // Set acceleration for profile position mode
   */
  int slaveNum;
   if(slave != 0) slaveNum = slave - 1;
   else slaveNum = 0;
   do{
      if(moveImmediate_u && this->slaves[slaveNum].mode == profPos) this->slaves[slaveNum].coeCtrlWord |= 0b00100000;
      else this->slaves[slaveNum].coeCtrlWord &= ~0b00100000;
      slaveNum++;
   } while((slaveNum < ec_slavecount) && (slave == 0));
   pthread_mutex_unlock(&this->control);
   
   return TRUE;
}

void DS402Controller::confSlavePDOs(uint slave, uint16 rxPDO1, uint16 txPDO1){
   _confSlavePDOs(slave, 1, rxPDO1, txPDO1, 0, 0, 0, 0, 0, 0);
}

void DS402Controller::confSlavePDOs(uint slave, uint16 rxPDO1, uint16 txPDO1, uint16 rxPDO2, uint16 txPDO2){
   _confSlavePDOs(slave, 2, rxPDO1, txPDO1, rxPDO2, txPDO2, 0, 0, 0, 0);
}

void DS402Controller::confSlavePDOs(uint slave, uint16 rxPDO1, uint16 txPDO1, uint16 rxPDO2, uint16 txPDO2, uint16 rxPDO3, uint16 txPDO3){
   _confSlavePDOs(slave, 3, rxPDO1, txPDO1, rxPDO2, txPDO2, rxPDO3, txPDO3, 0, 0);
}

void DS402Controller::confSlavePDOs(uint slave, uint16 rxPDO1, uint16 txPDO1, uint16 rxPDO2, uint16 txPDO2, uint16 rxPDO3, uint16 txPDO3, uint16 rxPDO4, uint16 txPDO4){
   _confSlavePDOs(slave, 4, rxPDO1, txPDO1, rxPDO2, txPDO2, rxPDO3, txPDO3, rxPDO4, txPDO4);
}

void DS402Controller::_confSlavePDOs(uint slave, uint8 num, uint16 rxPDO1, uint16 txPDO1, uint16 rxPDO2, uint16 txPDO2, uint16 rxPDO3, uint16 txPDO3, uint16 rxPDO4, uint16 txPDO4){
   int slaveNum;
   
   if(slave != 0) slaveNum = slave;
   else slaveNum = 1;
   do{

      if(this->PDOAssignments.size() < slaveNum) this->PDOAssignments.resize(slaveNum, slavePDOs{0,{},{}});

      this->PDOAssignments[slaveNum - 1].pdos = num;

      this->PDOAssignments[slaveNum - 1].rxPDO[0] = rxPDO1;
      this->PDOAssignments[slaveNum - 1].txPDO[0] = txPDO1;
      this->PDOAssignments[slaveNum - 1].rxPDO[1] = rxPDO2;
      this->PDOAssignments[slaveNum - 1].txPDO[1] = txPDO2;
      this->PDOAssignments[slaveNum - 1].rxPDO[2] = rxPDO3;
      this->PDOAssignments[slaveNum - 1].txPDO[2] = txPDO3;
      this->PDOAssignments[slaveNum - 1].rxPDO[3] = rxPDO4;
      this->PDOAssignments[slaveNum - 1].txPDO[3] = txPDO4;
      
      slaveNum++;
   } while((slaveNum <= ec_slavecount) && (slave == 0));
   
}

// Can block for timeout_ms
int DS402Controller::Home(uint slave, int HOME_MODE, int HOME_DIR, int speed, int acceleration, int HOME_DIST, int HOME_P, int timeout_ms){

   uint sdoBuff, err = 0, slaveNum;
   ecat_OpModes prevMode;

   pthread_mutex_lock(&this->control);

   if(slave == 0) slaveNum = 0;
   else slaveNum = slave - 1;
   while(slaveNum < ec_slavecount){ // Configure Homing parameters.

      prevMode = this->slaves[slaveNum].mode;

      sdoBuff = HOME_MODE;
      ec_SDOwrite(slaveNum + 1, HM_MODE , FALSE, 1, &sdoBuff, EC_TIMEOUTRXM);
      
      // HOME.DIR
      sdoBuff = HOME_DIR;
      ec_SDOwrite(slaveNum + 1, HM_DIR , FALSE, 1, &sdoBuff, EC_TIMEOUTRXM); 

      // HOME.V
      sdoBuff = speed;
      ec_SDOwrite(slaveNum + 1, HM_V , FALSE, 1, &sdoBuff, EC_TIMEOUTRXM); 

      // HOME.ACC/HOME.DEC
      sdoBuff = acceleration;
      ec_SDOwrite(slaveNum + 1, HMACCEL , FALSE, 4, &sdoBuff, EC_TIMEOUTRXM);

      // HOME.DIST
      sdoBuff = HOME_DIST;
      ec_SDOwrite(slaveNum + 1, HM_DIST , FALSE, 4, &sdoBuff, EC_TIMEOUTRXM);

      // HOME.P
      sdoBuff = HOME_P;
      ec_SDOwrite(slaveNum + 1, HM_P , FALSE, 4, &sdoBuff, EC_TIMEOUTRXM); 

      if(slave == 0) slaveNum++;
      else break;
   }
   pthread_mutex_unlock(&this->control);

   if(!DS402Controller::setOpMode(slave, homing)) return FALSE;

   if(DS402Controller::Update(slave, TRUE, timeout_ms) != 0) return FALSE;

   if(!DS402Controller::setOpMode(slave, prevMode)) return FALSE;

   return TRUE;
}

// Returns TRUE if fault
bool DS402Controller::readFault(uint slave){
   int slaveNum;

   pthread_mutex_lock(&this->control);

   if(slave == 0) slaveNum = 0;
   else slaveNum = slave - 1;
   while(slaveNum < ec_slavecount){

      if(this->slaves[slave - 1].coeCurrentState == cs_Fault){
         pthread_mutex_unlock(&this->control);
         return TRUE;
      }

      if(slave == 0) slaveNum++;
      else break;
   } 

   pthread_mutex_unlock(&this->control);
   return FALSE;
}

bool DS402Controller::clearFault(uint slave, bool persistClear){

   pthread_mutex_lock(&this->control);

   int err = 0, slaveNum;
   bool noFaults;
   struct timespec timeout;
   clock_gettime(CLOCK_MONOTONIC, &timeout);
   timeout.tv_sec += 1;

   // Loop through all slaves and set clear fault bit
   if(slave == 0) slaveNum = 0;
   else slaveNum = slave - 1;
   while(slaveNum < ec_slavecount){
      this->slaves[slaveNum].coeCtrlWord |= 0b10000000; // Clear Fault : 0b1---xxxx
      if(slave == 0) slaveNum++;
      else break;
   } 

   // Loop through all slaves and check that all are out of fault.
   while(!noFaults && err == 0) {
      noFaults = TRUE;
      if(slave == 0) slaveNum = 0;
      else slaveNum = slave - 1;
      while(slaveNum < ec_slavecount){
         if(this->slaves[slaveNum].coeCurrentState == cs_Fault){
            noFaults = FALSE;
            break;
         }
         if(slave == 0) slaveNum++;
         else break;
      } 
      err = pthread_cond_timedwait(&this->stateUpdated, &this->control, &timeout);
   }

   if(slave == 0) slaveNum = 0;
   else slaveNum = slave - 1;
   while(slaveNum < ec_slavecount){
      if(!persistClear) this->slaves[slave - 1].coeCtrlWord &= ~0b10000000; // Clear Fault : 0b1---xxxx
      if(slave == 0) slaveNum++;
      else break;
   } 
   

   pthread_mutex_unlock(&this->control);

   return err == 0;
}

