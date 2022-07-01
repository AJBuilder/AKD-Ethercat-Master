//////////////////////////// Timing ////////////////////////////
#define NSEC_PER_SEC 1000000000
#define EC_TIMEOUTMON 500

//////////////////// PDO Assignment Objects ////////////////////
#define rxPDOEnable 0x1C12, 0
#define rxPDOAssign1 0x1C12

#define txPDOEnable 0x1C13, 0
#define txPDOAssign1 0x1C13

///////////////// Control and Status Objects //////////////////
#define COControl    0x6040
#define MnfStatus    0x1002
#define COStatus     0x6041

#define QSTOP_OPT    0x605A, 0
#define REQOPMODE    0x6060, 0
#define ACTOPMODE    0x6061, 0
#define MOTOR_RATIO  0x6091, 1
#define SHAFT_RATIO  0x6091, 2

/////////////////////// Homing Objects ///////////////////////
#define HM_MODE          0x50CB, 0
#define HM_DIR           0x50C5, 0 
#define HM_SET           0x35F0, 0 // Start home
#define HM_ACC           0x3502, 0
#define HM_DEC           0x3524, 0 
#define HMACCEL          0x609A, 0 // Bidirectional acceleration
#define HM_AUTOMOVE      0x36D7, 0 // Automove after startup
#define HM_DIST          0x3484, 0 // Aditional move after home
#define HM_P             0x607C, 0 // Encoder offset after home

// Homeing Mode : Limitswitch
#define HM_V             0x6099, 1

// Homing Mode : Position
#define HM_FEEDRATE      0x6099, 2
#define HM_TPOSWND       0x5406, 0

// Homing Mode : Hardstop
#define HM_IPEAKACTIVE   0x5403, 0
#define HM_IPEAK         0x35E2, 0
#define HM_PERRTHRESH    0x3482, 0

/////////////////////// Profile Control Objects ///////////////////////
#define MT_V            0x6081, 0
#define MT_ACC          0x6083, 0
#define MT_DEC          0x6084, 0
#define PF_MAXCURRENT   0x6073, 0

// CoE States (0x6041)
#define NOTRDY2SWCH  0b00000000
#define SWCHDISABLED 0b01000000
#define RDY2SWITCH   0b00100001
#define SWITCHEDON   0b00100011
#define OP_ENABLED   0b00100111
#define FAULT        0b00001000
#define FAULTREACT   0b00001111
#define QUICKSTOP    0b00000111

// CoE Control (0x6040)
#define CTRL_SHUTDOWN_bit   0b0001
#define CTRL_DIS_VOL_bit    0b0010
#define CTRL_QSTOP_bit      0b0100
#define CTRL_ENABLED_bit    0b1000
#define CTRL_FAULTRESET_bit 0b10000000

////// Config //////
//Talker
#define CYCLE_NS (8*1000*1000) //8ms
#define SYNC_WINDOW_NS (100*1000) //.1ms
#define SYNC_AQTIME_NS (1000*1000*1000) // 1000ms
#define SYNC_DIST (2000*1000) // 2ms

//Controller
#define CNTRL_CYCLEMS   500

#define UNINIT_OPMODE   0
#define UNINIT_RXPDO    0
#define UNINIT_TXPDO    0
#define DEFAULT_QSCODE  6
#define DEFAULT_MOVIMM  FALSE
#define DEFAULT_MOVREL  FALSE
#define DEFAULT_DIGOUT  0
#define DEFAULT_HMAUTOMOVE 0


// Operation
#define DEBUG_MODE TRUE

////////////////////// Error Codes //////////////////////

// Generic (errno.h)
#define ECAT_ETIMEDOUT 110

// Update()
#define AKD_MOVEERR -1


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

    int  Home(uint slave, int HOME_MODE, int HOME_DIR, int speed, int acceleration, int HOME_DIST, int HOME_P, int timeout_ms);
    int  Update(uint slave, bool move, int timeout_ms);
    bool QuickStop(uint slave, bool enableQuickStop);
    bool waitForTarget(uint slave, uint timeout_ms);

    void confSlavePDOs(uint slave, void* usrControl, int size, uint16 rxPDO1, uint16 rxPDO2, uint16 rxPDO3, uint16 rxPDO4, uint16 txPDO1, uint16 txPDO2, uint16 txPDO3, uint16 txPDO4);
    bool confProfPos(uint slave, bool moveImmediate, bool moveRelative);
    bool confMotionTask(uint slave, uint vel, uint acc, uint dec);
    void confDigOutputs(uint slave, uint32 bitmask, uint8 out1Mode, uint8 out2Mode);
    bool confUnits(uint slave, uint32 motorRev, uint32 shaftRev);
    bool setOpMode(uint slave, ecat_OpModes reqMode);

    

    
    private:

    // Const
    uint slaveCount;

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
        uint8 digOutBitmask, digOut1Mode, digOut2Mode;
        uint16 quickStopOption;

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
    ecat_masterStates masterState;

    

    // Threading
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
    char debugBuffer[20][150] = {0};

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
