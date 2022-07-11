#ifndef DEBUG_MODE
#define DEBUG_MODE  TRUE

#define DEBUG_BUFF_SIZE 50
#define DEBUG_BUFF_WIDTH 200

#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/queue.h>
#include <inttypes.h>
#include <pthread.h>
#include <vector>

#include "ethercat.h"

    
    enum ecat_OpModes:int8{profPos = 1, profVel = 3, profTor = 4, homing = 6, intPos = 7, syncPos = 8};

class AKDController{
    public:

    enum ecat_masterStates:int8{ms_shutdown, ms_stop, ms_disable, ms_enable};
    
    bool ecat_Init(char *ifname);
    bool ecat_Start();

    bool Enable();
    bool Disable();
    bool Stop();
    bool Shutdown();

    bool clearFault(uint slave, bool persistClear);
    bool readFault(uint slave);
    bool readFaultList();

    bool Home(uint slave, int mode, int dir, int speed, int acc, int dist, int pos, int timeout_ms);
    int  Update(uint slave, bool move, int timeout_ms);
    bool QuickStop(uint slave, bool enableQuickStop);
    bool waitForTarget(uint slave, uint timeout_ms);

    void confSlavePDOs(uint slave, void* usrControl, int size, uint16 rxPDO1, uint16 rxPDO2, uint16 rxPDO3, uint16 rxPDO4, uint16 txPDO1, uint16 txPDO2, uint16 txPDO3, uint16 txPDO4);
    bool confProfPos(uint slave, bool moveImmediate, bool moveRelative);
    bool confMotionTask(uint slave, uint vel, uint acc, uint dec);
    bool confDigOutputs(uint slave, bool enableOut1, bool enableOut2, uint8 out1Mode, uint8 out2Mode);
    bool confUnits(uint slave, uint32 motorRev, uint32 shaftRev);
    bool setOpMode(uint slave, ecat_OpModes reqMode);
    
    private:

    // Const
    char *ifname;
    uint slaveCount, configuredSlaves;

    // PDO buffers
    uint8 IOmap[4096];//[4096];
    
    
    // Slave Data Struct Array
    class ecat_slave{
        public:
        /*ecat_slave(){
            coeCtrlWord = (DEFAULT_MOVIMM << 5) | (DEFAULT_MOVREL << 6);
            rxPDO = (struct mappings_t){0};
            txPDO = (struct mappings_t){0};
            quickStopOption = DEFAULT_QSCODE;
            digOut1Mode = DEFAULT_DIGOUT;
            digOut2Mode = DEFAULT_DIGOUT;
            update = FALSE;
            quickStop = FALSE;
            mode = (ecat_OpModes)UNINIT_OPMODE;
        }*/
        // PDO Data
        uint8 *outUserBuff, *inUserBuff;
        uint16 *coeCtrlMapPtr, *coeStatusMapPtr;
        int coeCtrlOffset, coeStatusOffset;

        // PDO Assign
        struct mappings_t{
            uint8  numOfPDOs;
            uint16 mapObject[4];
            uint8  bytes;
        }rxPDO, txPDO;
        uint8 totalBytes;

        // Config

        // Slave Control Signals
        bool update, quickStop;
        
        uint16 coeCtrlWord, coeStatus;

        // Profile Control
        ecat_OpModes mode;
        bool moveFin, moveAck, moveErr;
    } *slaves;

    // Control Signals
    bool inOP;
    uint inSyncCount;
    int8 wrkCounter, expectedWKC;
    uint64 diffDCtime;
    ecat_masterStates masterState = ms_shutdown;

    

    // Threading
    pthread_attr_t rt_attr;
    pthread_mutex_t debug, control;
    pthread_cond_t IOUpdated, stateUpdated;
    pthread_t talker, controller;

    #if DEBUG_MODE
    char coeStatusReadable[9][23] = {
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
    char coeCtrlReadable[7][18] = {
        "Shutdown",                //0
        "Switch-On",               //1
        "Disable Voltage",         //2
        "Quick Stop",              //3
        "Disable Operation",       //4
        "Enable Operation",        //5
        "Fault Reset"              //6
    };

    // Debug 
    int64 gl_toff, gl_delta;
    uint8 gl_integral;
    uint buffHead = 0, buffTail = 0;
    char debugBuffer[DEBUG_BUFF_SIZE][DEBUG_BUFF_WIDTH] = {0};

    void addToDebugBuff(char *str);
    void printDebugBuff();
    char* getReadableStatus(uint16 status);
    char* getReadableCtrl(uint16 ctrl);
    

    #endif

    // Utility Methods
    bool State(ecat_masterStates reqState);
    bool ec_sync(int64 reftime, uint64 cycletime , int64 *offsettime, int64 dist, int64 window, int64 *d, int64 *i);
    

    // Main Methods
    static void* ecat_Talker(void* THIS);
    static void* ecat_Controller(void* THIS);
};
