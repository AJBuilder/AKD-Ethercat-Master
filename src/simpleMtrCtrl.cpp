
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/queue.h>
#include <inttypes.h>
#include <pthread.h>

#include "DS402EcatController.h"
#include "ethercat.h"


int main(int argc, char *argv[])
{
   printf("Starting simpleMtrCtrl wrapper for acrEcat\n");


   
   int currentgroup = 0;

   if (argc > 1)
   {
      DS402Controller master1;

      

      struct __attribute__((__packed__)){

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
         uint32   latchPos;
         uint16   coeStatus;
         int16    tqActual;
         uint16   latchStatus;
         int16    analogInput;
      } PDOs = {0};


      master1.confSlavePDOs(1, 0x1725, 0x1B20);
      if(!master1.ecat_Init(argv[1])) return -1;

      if (!master1.ecat_Start(&PDOs, sizeof(PDOs))) return -2;
      osal_usleep(10000);

      if(master1.readFault(1)){
         printf("\nWaiting for fault to clear!\n");
         do{
            master1.clearFault(1,FALSE);

         }while(master1.readFault(1));
         printf("\nFault cleared!\n");
      }

      while(!master1.Enable()){
         printf("\nEnable failed\n");
      }
      printf("\nEnabled!\n");

      PDOs.maxTorque = 1000;

      while(!master1.setOpMode(0, profPos)){
         printf("\nMode switch failed\n");
      }
      printf("\nMode switched!\n");
      if(master1.Home(0, 0, 0, 60, 1000, 0, 0, 1000)){
         printf("\nHomed\n");
      }else{
         printf("\nFailed to home\n");
      }
       osal_usleep(5000000); // 5 sec

      PDOs.targetPos = 360;
      master1.Update(1, TRUE, 5000); // 5 sec
      printf("\nSetpoint set\n");
      osal_usleep(50000000); // 50 sec
      
      master1.QuickStop(1, TRUE); printf("Quickstop!\n");
      osal_usleep(5000000); // 5 sec
      printf("Re-enable\n");
      master1.QuickStop(1, FALSE);
      

      master1.Update(1, TRUE, 5000); // 5 sec
      printf("Ending\n");
      osal_usleep(5000000); // 5 sec

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
