
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/queue.h>
#include <inttypes.h>


#include "AKDEcatController.h"
#include "ethercat.h"

#define USECS_PER_SEC     1000000

int us_sleep(uint32_t usec)
{
   struct timespec ts;
   ts.tv_sec = usec / USECS_PER_SEC;
   ts.tv_nsec = (usec % USECS_PER_SEC) * 1000;
   return nanosleep(&ts, nullptr);
}

int main(int argc, char *argv[])
{
   printf("Starting simpleMtrCtrlx2 wrapper\n");

   

   if (argc > 1)
   {
      AKDController master1;
      int err = 0;
      std::string ifname = argv[1];

      
      struct __attribute__((__packed__)){
         //0x1725
         //rxPDOs
         uint16_t  ctrlWord; 
         uint32_t  targetPos;
         uint32_t  digOutputs;
         uint16_t  tqFdFwd;         
         uint16_t  maxTorque;
         
         //0x1B20
         //txPDOs
         int32_t   posActual;
         int32_t   posFdback2;
         int32_t   velActual;
         uint32_t  digInputs;
         int32_t   followErr;
         uint32_t  latchPos;
         uint16_t  coeStatus;
         int16_t   tqActual;
         uint16_t  latchStatus;
         int16_t   analogInput;
      } s1;

      struct __attribute__((__packed__)){
         //0x1725
         //rxPDOs
         uint16_t  ctrlWord; 
         uint32_t  targetPos;
         uint32_t  digOutputs;
         uint16_t  tqFdFwd;         
         uint16_t  maxTorque;
         
         //0x1B20
         //txPDOs
         int32_t   posActual;
         int32_t   posFdback2;
         int32_t   velActual;
         uint32_t  digInputs;
         int32_t   followErr;
         uint32_t  latchPos;
         uint16_t  coeStatus;
         int16_t   tqActual;
         uint16_t  latchStatus;
         int16_t   analogInput;
      } s2;


      #if AKD_ECAT_DEBUG_MODE
         printf("\nLocked memory in main(PID: %d) before Init: %dkB\n", getpid(), master1.getLockedMem());
      #endif

      if(!master1.ecat_Init(ifname)) return -1;

      #if AKD_ECAT_DEBUG_MODE
         printf("\nLocked memory in main after Init: %dkB\n", master1.getLockedMem());
      #endif

      master1.confSlavePDOs(1, &s1, sizeof(s1), 0x1725, 0,0,0, 0x1B20, 0,0,0);
      master1.confSlavePDOs(2, &s2, sizeof(s2), 0x1725, 0,0,0, 0x1B20, 0,0,0);

      master1.confUnits(1, 1, 360);
      master1.confUnits(2, 1, 360);

      master1.confMotionTask(0, 2000, 10000, 10000);

      master1.confProfPos(0, true, false);

      if (!master1.ecat_Start()) return -2;
      us_sleep(10000);

      printf("\nReading fault...\n");
      if(master1.readFault(0)){
         printf("\nWaiting for fault to clear!\n");
         do{
            master1.clearFault(0,false);

         }while(master1.readFault(0));
         printf("\nFault cleared!\n");
      }

      //us_sleep(50000000); // 50 sec

      printf("\nEnabling...\n");
      if(!master1.Enable()){
         printf("\nEnable failed\n");
         return -3;
      }
      printf("\nEnabled!\n");

      s1.maxTorque = 1000;
      s2.maxTorque = 1000;

      if(!master1.setOpMode(0, profPos)){
         printf("\nMode switch failed\n");
         return -4;
      }
      printf("\nMode switched!\n");

      err = master1.Home(0, 0, 0, 6000, 1000, 500, 0, 0);
      if(err != true){
         printf("\nFailed to home. %i\n", err);
         return -5;
      }
      printf("\nHomed\n");

      us_sleep(5000000); // 5 sec

      
         s1.targetPos = 180*30;
         s2.targetPos = 180*7;
      for(int i = 0; i < 3; i++){
         s1.targetPos = -s1.targetPos;
         s2.targetPos = -s2.targetPos;
         printf("\nSetting Slave 1 pos = %d\n", s1.targetPos);
         printf("\nSetting Slave 2 pos = %d\n", s2.targetPos);
         master1.Update(0, true, 500000); // 5 sec
         printf("\nSetpoint set\n");
         master1.waitForTarget(0,0);
         us_sleep(5000000);
      }
      
      /*
      master1.QuickStop(1, true); printf("Quickstop!\n");
      us_sleep(5000000); // 5 sec
      printf("Re-enable\n");
      master1.QuickStop(1, false);
      */

      master1.Update(0, false, 5000); // 5 sec
      printf("Ending\n");

      master1.Shutdown();
      
   }
   else
   {
      ec_adaptert * adapter = nullptr;
      printf("Usage: simple_test ifname1\nifname = eth0 for example\n");

      printf ("\nAvailable adapters:\n");
      adapter = ec_find_adapters ();
      while (adapter != nullptr)
      {
         printf ("    - %s  (%s)\n", adapter->name, adapter->desc);
         adapter = adapter->next;
      }
      ec_free_adapters(adapter);
   }


   printf("End program\n");
   return 0;
}
