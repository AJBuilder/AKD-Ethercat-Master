
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/queue.h>
#include <inttypes.h>
#include <pthread.h>

#include "AKDEcatController.h"
#include "ethercat.h"


int main(int argc, char *argv[])
{
   printf("Starting simpleMtrCtrl wrapper for acrEcat\n");


   

   if (argc > 1)
   {
      AKDController master1;
      int err = 0;
      
      struct __attribute__((__packed__)){
         //0x1725
         //rxPDOs
         uint16   ctrlWord; 
         uint32   targetPos;
         uint32   digOutputs;
         uint16   tqFdFwd;         
         uint16   maxTorque;
         
         //0x1B20
         //txPDOs
         int32    posActual;
         int32    posFdback2;
         int32    velActual;
         uint32   digInputs;
         int32    followErr;
         uint32   latchPos;
         uint16   coeStatus;
         int16    tqActual;
         uint16   latchStatus;
         int16    analogInput;
      } s1;



      
      if(!master1.ecat_Init(argv[1])) return -1;
      master1.confSlavePDOs(1, &s1, sizeof(s1), 0x1725, 0,0,0, 0x1B20, 0,0,0);
      master1.confUnits(1, 1, 360);
      master1.confMotionTask(1, 2000, 10000, 10000);
      master1.confProfPos(1, TRUE, FALSE);
      
      //master1.confSlavePDOs(2, &s2, sizeof(s1), 0x1725, 0x1B20);

      if (!master1.ecat_Start()) return -2;
      osal_usleep(10000);

      if(master1.readFault(1)){
         printf("\nWaiting for fault to clear!\n");
         do{
            master1.clearFault(1,FALSE);

         }while(master1.readFault(1));
         printf("\nFault cleared!\n");
      }

      if(!master1.Enable()){
         printf("\nEnable failed\n");
         return -3;
      }
      printf("\nEnabled!\n");

      s1.maxTorque = 1000;

      if(!master1.setOpMode(1, profPos)){
         printf("\nMode switch failed\n");
         return -4;
      }


      printf("\nMode switched!\n");

      err = master1.Home(1, 0, 0, 6000, 1000, 0, 0, 0);
      if(err != TRUE){
         printf("\nFailed to home. %i\n", err);
         return -5;
      }
      printf("\nHomed\n");

       osal_usleep(5000000); // 5 sec

      
      //s1.digOutputs = 3 << 0;
         s1.targetPos = 180*7;
      for(int i = 0; i < 3; i++){
         s1.targetPos = -s1.targetPos;
         printf("\nSetting pos = %d\n", s1.targetPos);
         master1.Update(1, TRUE, 500000); // 5 sec
         printf("\nSetpoint set\n");
         master1.waitForTarget(1,0);
         osal_usleep(5000);
      }
      
      /*
      master1.QuickStop(1, TRUE); printf("Quickstop!\n");
      osal_usleep(5000000); // 5 sec
      printf("Re-enable\n");
      master1.QuickStop(1, FALSE);
      */

      master1.Update(1, TRUE, 5000); // 5 sec
      printf("Ending\n");

      master1.Shutdown();
      
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
