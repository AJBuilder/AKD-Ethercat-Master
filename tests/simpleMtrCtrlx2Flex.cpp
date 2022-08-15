
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/queue.h>
#include <inttypes.h>


#include "AKDEcatController.h"
#include "soem/ethercat.h"

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
            //rxPDOs
            uint16_t  ctrlWord;     // 0x6040, 0
            uint16_t  maxTorque;    // 0x6072, 0
            uint32_t  targetPos;    // 0x607A, 0
            uint32_t  targetVel;    // 0x60FF, 0
            
            //txPDOs
            uint16_t  coeStatus;    // 0x6041, 0
            uint32_t  actualPos;    // 0x6064, 0
            uint32_t  actualVel;    // 0x606C, 0

        } s1;

        struct __attribute__((__packed__)){
            //rxPDOs
            uint16_t  ctrlWord;     // 0x6040, 0
            uint16_t  maxTorque;    // 0x6072, 0
            uint32_t  targetPos;    // 0x607A, 0
            uint32_t  targetVel;    // 0x60FF, 0
            
            //txPDOs
            uint16_t  coeStatus;    // 0x6041, 0
            uint32_t  actualPos;    // 0x6064, 0
            uint32_t  actualVel;    // 0x606C, 0
        } s2;

        AKDController::ecat_pdoEntry_t tx[3] = {{0x6041,0}, {0x6064,0}, {0x606C,0}};
        AKDController::ecat_pdoEntry_t rx[4] = {{0x6040,0}, {0x6072,0},{0x607A,0},{0x60FF,0}};

        #if AKD_ECAT_DEBUG_MODE
            printf("\nLocked memory in main(PID: %d) before Init: %dkB\n", getpid(), master1.getLockedMem());
        #endif

        if(!master1.ecat_Init(ifname.c_str())) return -1;

        #if AKD_ECAT_DEBUG_MODE
            printf("\nLocked memory in main after Init: %dkB\n", master1.getLockedMem());
        #endif

        master1.confSlavePDOs(1, &s1, sizeof(s1), 0x1600, 0x1601, 0x1602, 0x1603, 0x1a00, 0x1a01, 0x1a02, 0x1a03);
        master1.confSlavePDOs(2, &s2, sizeof(s2), 0x1600, 0x1601, 0x1602, 0x1603, 0x1a00, 0x1a01, 0x1a02, 0x1a03);

        if(!master1.confSlaveEntries(0, rx, sizeof(rx)/sizeof(AKDController::ecat_pdoEntry_t), tx, sizeof(tx)/sizeof(AKDController::ecat_pdoEntry_t))){
            printf("Couldn't confSlaveEntries.\n");
            return -1;
        }

        

        if(!master1.confUnits(1, 30, 360)) {
            printf("Couldn't confUnits on 1\n");
            return -1;
        }

        if(!master1.confUnits(2, 7, 360)) {
            printf("Couldn't confUnits on 2\n");
            return -2;
        }

        if(!master1.confMotionTask(0, 500, 6000, 6000)) {
            printf("Couldn't confMotionTask\n");
            return -3;
        }

        if(!master1.setOpMode(0, AKDController::ecat_OpModes::profPos)) {
            printf("Couldn't setOpMode\n");
            return -3;
        }

        if(!master1.confProfPos(0, false, false)) {
            printf("Couldn't confProfPos\n");
            return -4;
        }

        if (!master1.ecat_Start()) {
            printf("Couldn't start\n");
            return -5;
        }
        us_sleep(10000);

        printf("\nReading fault...\n");
        if(master1.readFault(0)){
            printf("\nWaiting for fault to clear!\n");
            do{
            master1.clearFault(0,false);

            }while(master1.readFault(0));
            printf("\nFault cleared!\n");
        }

        printf("\nEnabling...\n");
        if(!master1.Enable()){
            printf("\nEnable failed\n");
            return -6;
        }
        printf("\nEnabled!\n");

        s1.maxTorque = 3000;
        s2.maxTorque = 3000;

        /*if(!master1.writeObject(0, 0x6081, 0, 4000)){ // Set MT.V (Velocity)
            printf("\nFailed to set velocity for position test.\n");
            return -6;
        }*/

        err = master1.Home(0, 0, 0, 6000, 1000, 500, 0, 0);
        if(err != true){
            printf("\nFailed to home. %i\n", err);
            return -8;
        }
        printf("\nHomed\n");

        us_sleep(5000000); // 5 sec

        
            s1.targetPos = 180;
            s2.targetPos = 180;
        for(int i = 0; i < 3; i++){
            s1.targetPos += s1.targetPos;
            s2.targetPos += s2.targetPos;
            printf("\nSetting Slave 1 pos = %d\n", s1.targetPos);
            printf("\nSetting Slave 2 pos = %d\n", s2.targetPos);
            master1.Update(0, true, 500000); // 5 sec
            printf("\nSetpoint set, waiting for target.\n");
            master1.waitForTarget(0,0);
            printf("\nTarget reached!\n");
            us_sleep(5000000);
        }
        
        printf("\nPosition test finished.\n");
        if(!master1.setOpMode(0, AKDController::ecat_OpModes::profVel)){
            printf("\nCouldn't switch to velocity mode.\n");
            return -9;
        }

            s1.targetVel = 500;
            s2.targetVel = 500;
        for(int i = 0; i < 3; i++){
            s1.targetVel = -s1.targetVel;
            s2.targetVel = -s2.targetVel;
            printf("\nSetting Slave 1 vel = %d\n", s1.targetVel);
            printf("\nSetting Slave 2 vel = %d\n", s2.targetVel);
            master1.Update(0, true, 500000); // 5 sec
            //printf("\nVelocity set, waiting for target.\n");
            //master1.waitForTarget(0,0);
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
