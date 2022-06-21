
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


      master1.ecat_Init(argv[1], &PDOs, sizeof(PDOs), rxPDOFixed, txPDOFixed);
      osal_usleep(10000);

      if(master1.readFault()){
         printf("\nWaiting for fault to clear!\n");
         do{
            master1.clearFault(FALSE);

         }while(master1.readFault());
         printf("\nFault cleared!\n");
      }

      while(!master1.Enable()){
         printf("\nEnable failed\n");
      }
      while(!master1.setOpMode(profPos)){
         printf("\nMode switch failed\n");
      }
      
      master1.Home(0, 0, 60, 1000, 0, 0, 1000);
      
      printf("\nHomed\n");


      PDOs.maxTorque = 1000;
      //PDOs.targetVel = 60*1000;
      /*for(int i = 0, pos = 0; i < 10; i++, pos += 100){
         PDOs.targetPos = pos;
         ecat_Update(TRUE);
         osal_usleep(1000000);
      }*/

      PDOs.targetPos = 360;
      master1.Update(TRUE, 5000); // 5 sec
      printf("\nSetpoint set\n");
      osal_usleep(5000000); // 5 sec
      
      master1.QuickStop(TRUE); printf("Quickstop!\n");
      osal_usleep(5000000); // 5 sec
      printf("Re-enable\n");
      master1.QuickStop(FALSE);
      

      master1.Update(TRUE, 5000); // 5 sec
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
