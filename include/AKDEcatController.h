#ifndef AKD_ECAT_DEBUG_MODE
    #define AKD_ECAT_DEBUG_MODE false
#endif

#define DEBUG_BUFF_SIZE 50
#define DEBUG_BUFF_WIDTH 200

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>


    

class AKDController{
    public:

    enum class ecat_OpModes {profPos = 1, profVel = 3, profTor = 4, homing = 6, intPos = 7, syncPos = 8};
    enum class ecat_masterStates{ms_shutdown, ms_stop, ms_disable, ms_enable};
    struct ecat_pdoEntry_t{
        uint16_t index;
        uint8_t  subIndex;
    };
    
    bool ecat_Init(const char* ifname);
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

    void confSlavePDOs(uint slave, const void* usrControl, int size, uint16_t rxPDO1, uint16_t rxPDO2, uint16_t rxPDO3, uint16_t rxPDO4, uint16_t txPDO1, uint16_t txPDO2, uint16_t txPDO3, uint16_t txPDO4);
    bool confSlaveEntries(uint slave, ecat_pdoEntry_t *rxEntries, int numOfRx, ecat_pdoEntry_t *txEntries, int numOfTx);

    bool confProfPos(uint slave, bool moveImmediate, bool moveRelative);
    bool confMotionTask(uint slave, uint vel, uint acc, uint dec);
    bool confDigOutputs(uint slave, bool enableOut1, bool enableOut2, uint8_t out1Mode, uint8_t out2Mode);
    bool confUnits(uint slave, uint32_t motorRev, uint32_t shaftRev);
    bool setOpMode(uint slave, ecat_OpModes reqMode);
    
    #if AKD_ECAT_DEBUG_MODE

        int getLockedMem();

    #endif

    private:

    // Const
    char* ifname;
    uint slaveCount, configuredSlaves;

    // PDO buffers
    uint8_t IOmap[4096];//[4096];
    
    
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
            mode = (ecat_OpModes_t)UNINIT_OPMODE;
        }*/
        // PDO Data
        uint8_t *outUserBuff, *inUserBuff;
        uint16_t *coeCtrlMapPtr, *coeStatusMapPtr;
        int coeCtrlOffset, coeStatusOffset;

        // PDO Assign
        struct mappings_t{
            uint8_t  numOfPDOs;
            uint16_t mapObject[4];
            uint8_t  bytes;
        }rxPDO, txPDO;
        uint8_t totalBytes;

        // Config

        // Slave Control Signals
        bool update, quickStop;
        
        uint16_t coeCtrlWord, coeStatus;

        // Profile Control
        ecat_OpModes mode;
        bool moveFin, moveAck, moveErr;
    } *slaves;

    // Control Signals
    bool inOP;
    uint inSyncCount;
    int8_t wrkCounter, expectedWKC;
    int64_t diffDCtime;
    ecat_masterStates masterState = ecat_masterStates::ms_shutdown;

    

    // Threading
    pthread_attr_t rt_attr;
    pthread_mutex_t debug, control;
    pthread_cond_t IOUpdated, stateUpdated;
    pthread_t talker, controller;

    #if AKD_ECAT_DEBUG_MODE
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
    int64_t gl_toff, gl_delta;
    uint64_t gl_integral;
    uint buffHead = 0, buffTail = 0;
    char debugBuffer[DEBUG_BUFF_SIZE][DEBUG_BUFF_WIDTH] = {0};

    void addToDebugBuff(char *str);
    void printDebugBuff();
    char* getReadableStatus(uint16_t status);
    char* getReadableCtrl(uint16_t ctrl);
    

    #endif

    // Utility Methods
    bool State(ecat_masterStates reqState);
    bool ec_sync(int64_t reftime, uint64_t cycletime , int64_t *offsettime, int64_t dist, int64_t window, int64_t *d, int64_t *i);
    
    

    // Main Methods
    static void* ecat_Talker(void* THIS);
    static void* ecat_Controller(void* THIS);
};
