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
#define REQOPMODE    0x6060, 0
#define ACTOPMODE    0x6061, 0
#define MnfStatus    0x1002
#define COStatus     0x6041
#define QSTOP_OPT    0x605A, 0

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

// Config
#define CYCLE_NS (8*1000*1000) //8ms
#define SYNC_WINDOW_NS (100*1000) //.1ms
#define SYNC_AQTIME_NS (1000*1000*1000) // 1000ms
#define SYNC_DIST (2000*1000) // 2ms

#define DEFAULT_OPMODE  profPos
#define DEFAULT_RXPDO   0x1701
#define DEFAULT_TXPDO   0x1B01

// Operation
#define DEBUG_MODE TRUE

////////////////////// Error Codes //////////////////////

// Generic (errno.h)
#define DS402_ETIMEDOUT 110

// Update()
#define DS402_MOVEERR -1


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/queue.h>
#include <inttypes.h>
#include <pthread.h>
#include <vector>

#include "ethercat.h"

    
    enum ecat_OpModes:int8{profPos = 1, profVel = 3, profTor = 4, homing = 6, intPos = 7, syncPos = 8};

class DS402Controller{
    public:

    enum ecat_masterStates:int8{ms_shutdown, ms_stop, ms_disable, ms_enable};
    enum ecat_coeStates:int8{cs_NotReady = 0, cs_SwitchDisabled = 1, cs_Ready = 2, cs_SwitchedOn = 3, cs_OpEnabled = 4, cs_Fault = 5, cs_FaultReaction = 6, cs_QuickStop = 7, cs_Unknown = 8};
    enum ecat_coeStateTrans:int8{cst_Shutdown = 0, cst_SwitchOn = 1, cst_DisableVolt = 2, cst_TrigQuickStop = 3, cst_DisableOp = 4, cst_EnableOp = 5, cst_ResetFault = 6};

    bool ecat_Init(char *ifname);
    bool ecat_Start(void* usrControl, int size);
    bool Enable();
    bool Disable();
    bool Stop();
    bool Shutdown();
    bool setOpMode(uint slave, ecat_OpModes reqMode);
    bool ConfProfPosImm(uint slave, bool moveImmediate_u);
    bool clearFault(uint slave, bool persistClear);
    bool readFault(uint slave);
    bool readFaultList();
    int  Home(uint slave, int HOME_MODE, int HOME_DIR, int speed, int acceleration, int HOME_DIST, int HOME_P, int timeout_ms);
    int  Update(uint slave, bool move, int timeout_ms);
    bool QuickStop(uint slave, bool enableQuickStop);
    void confSlavePDOs(uint slave, uint16 rxPDO1, uint16 txPDO1);
    void confSlavePDOs(uint slave, uint16 rxPDO1, uint16 txPDO1, uint16 rxPDO2, uint16 txPDO2);
    void confSlavePDOs(uint slave, uint16 rxPDO1, uint16 txPDO1, uint16 rxPDO2, uint16 txPDO2, uint16 rxPDO3, uint16 txPDO3);
    void confSlavePDOs(uint slave, uint16 rxPDO1, uint16 txPDO1, uint16 rxPDO2, uint16 txPDO2, uint16 rxPDO3, uint16 txPDO3, uint16 rxPDO4, uint16 txPDO4);
    

    
    private:
       
    // Init
    char* ifname;
    bool initialized;

    // PDO buffers
    uint8 IOmap[4096];//[4096];
    
    
    // Slave Data Struct Array
    struct ecat_slave{

        // PDO Data
        uint8 *outUserBuff, *inUserBuff;
        int coeCtrlPos, coeStatusPos;
        uint numOfPDOs;
        uint8 *outSizes, *inSizes; //Pointer to array. Index 0 specifies number of entries
        uint8 *coeCtrlMapPtr, *coeStatusMapPtr;

        // PDO Assign
        uint16 rxPDO, txPDO;

        // Slave Control Signals
        bool update, quickStop, moveImmediate;
        
        uint16 coeCtrlWord, coeStatus;
        ecat_coeStates coeCurrentState;
        ecat_coeStateTrans coeStateTransition;

        // Profile Control
        ecat_OpModes mode;
        bool moveAck, moveErr;
    } *slaves;
    struct slavePDOs{
        uint8 pdos;
        uint16 rxPDO[4];
        uint16 txPDO[4];
    };
    std::vector<slavePDOs> PDOAssignments;

    // Control Signals
    bool inOP;
    uint inSyncCount;
    int8 wrkCounter, expectedWKC;
    uint64 diffDCtime;
    ecat_masterStates masterState;

    // Debug 
    int64 gl_toff, gl_delta;
    uint8 gl_integral;

    // Threading
    pthread_mutex_t debug, control;
    pthread_cond_t IOUpdated;
    pthread_cond_t moveSig, stateUpdated;
    pthread_t talker, controller;

    char coeStateReadable[9][23] = {
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

    // Utility Methods
    int cpyData(void* dest, void* source, int bytes);
    void add_timespec(struct timespec *ts, int64 addtime);
    void readPDOAssignments(uint16 Slave, uint16 PDOassign, uint8* sizeList, uint* pdoAssignments, int* ctrlIndex, int* statIndex);
    bool ec_sync(int64 reftime, uint64 cycletime , int64 *offsettime, int64 dist, int64 window, int64 *d, int64 *i);
    bool State(ecat_masterStates reqState);
    void _confSlavePDOs(uint slave, uint8 num, uint16 rxPDO1, uint16 txPDO1, uint16 rxPDO2, uint16 txPDO2, uint16 rxPDO3, uint16 txPDO3, uint16 rxPDO4, uint16 txPDO4);

    // Main Methods
    static void* ecat_Talker(void* THIS);
    static void* ecat_Controller(void* THIS);


    
    
};
