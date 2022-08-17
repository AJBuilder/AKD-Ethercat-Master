# Tutorial
## Overview
The AKD Ethercat Master is a library made to interface and control the AKD series drives over Ethercat. No timing requirements are required of the user. Multiple masters can be opened on different ethernet ports. Must be run as sudo.

The Simple Open Ethercat Master (SOEM) library is required as a dependecy. The [repo](https://github.com/OpenEtherCATsociety/SOEM) is included under /extern and is built with AKD Ethercat Master.

Most of the drive configuration can be done over Ethercat. Any CANopen accessible parameters can be written and read. These parameters can be found in the Ethercat manual.

Manuals
- [User Manual](https://www.kollmorgen.com/sites/default/files/akd/documentation/user%20guide//AKD%20User%20Guide%20EN%20Rev%20J.pdf)
- [Ethercat Manual](https://www.kollmorgen.com/sites/default/files/public_downloads/AKD%20EtherCat%20Communications%20Manual%20EN%20%28REV%20P%29.pdf)
- [CANopen Manual](https://www.kollmorgen.com/sites/default/files/public_downloads/AKD%20CANopen%20Communications%20Manual%20EN%20%28REV%20U%29.pdf)
## Initialization and Setup
A master object must first be declared. This is done by using the AKDController class.

Example:
```
AKDController master1;
```
There are two main functions that must always be called— ecat_Init, and ecat_Start. ecat_Init should be called before any slave configuration. ecat_Start should be called after most slave configuration.

ecat_Init initializes a master on a specified ethernet port, sets up slave communication, and initializes data structures. The ecat_Init declaration is:
```
bool ecat_Init(const char* ifname);
```
"ifname" is the name of the specified ethernet port. (The one listed by ifconfig) Returns whether initialization was successful.

ecat_Start actually starts cyclical communication. Only then will PDO data be transfered. Motor power will also be enabled.The ecat_Start declaration is:
```
bool ecat_Start();
```
Returns whether startup was successful.

Example:
```
if(!master1.ecat_Init(ifname.c_str())){
    printf("Couldn't init\n");
    return -1;
}
...
Some slave configuration and setup.
...

if (!master1.ecat_Start()) {
    printf("Couldn't start\n");
    return -2;
}
```

The only configuration function that must be called is confSlavePDOs. This function is necessary because it links the user's buffer to the master. The confSlavePDOs declaration is:

```
void confSlavePDOs(uint slave, const void* usrControl, int size, uint16_t rxPDO1, uint16_t rxPDO2, uint16_t rxPDO3, uint16_t rxPDO4, uint16_t txPDO1, uint16_t txPDO2, uint16_t txPDO3, uint16_t txPDO4);
```
Example:
```
master1.confSlavePDOs(1, &s1Buff, sizeof(s1Buff), 0x1725, 0,0,0, 0x1B20, 0,0,0);
master1.confSlavePDOs(2, &s2Buff, sizeof(s2Buff), 0x1725, 0,0,0, 0x1B20, 0,0,0);
```
confSlavePDOs needs both a pointer to the a buffer, and its size. Requiring the size of the buffer to be passed is the least "idiot-proof" part of this user buffer approach. This user buffer is the data structure used to write and read data from PDOs. This structure must match the PDO entry mapping of the slave. Inputs(rxPDOs) are arranged first, outputs(txPDOs) are arranged second. **Common practice : reference to input/output is from the slave's perspective** ie. Data to the slave is first(input), data from the slave is second(output).

Example:
```
struct __attribute__((__packed__)){
    //rxPDO1
    uint16_t  someEntryInput1;
    uint16_t  someEntryInput2;
    uint8_t  someEntryInput3;
    //rxPDO2
    uint32_t  someEntryInput4;
    
    //txPDO1
    uint32_t  someEntryOutput1;
    uint16_t  someEntryOutput2;
    //txPDO2
    uint32_t  someEntryOutput3;
} slave1UserBuff;
```

A few things to note:
- **The struct must be packed!**
- Each entry must be sized appropriately.
- Inputs first, outputs second.
- A PDO's entries must be ordered according to their PDO's order in addition to being in order within their respective PDO.
- The CANopen control and status words must exist in the mapping.

 confSlavePDOs will also record what PDO assignments to write to the slave when ecat_Start is called. When an assignment argument is 0, no assignment configuration is written.

 confSlaveEntries can also be used in tandem with confSlavePDOs to map entries to the drives flexible PDOs. In order for flexible PDOs to work, confSlavePDOs must  assign all flexible PDO mapping objects. (0x1600, 0x1601, 0x1a00, etc) The declaration is:
 ```
bool confSlaveEntries(uint slave, ecat_pdoEntry_t *rxEntries, int numOfRx, ecat_pdoEntry_t *txEntries, int numOfTx);
 ```
 confSlaveEntries will auto-fit object entries to PDOs. All that is needed is a list of input and output entries. There are some limitations to these flexible mappings. However, confSlaveEntries will detect if the given entries exceed these limitations. These limitations can be found in the AKD ethercat manual (Pg 44).

 Entries are specified using the member struct ecat_pdoEntry_t. This struct simply has the object index and subindex. 

 Example:
 ```
 AKDController::ecat_pdoEntry_t tx[3] = {{0x6041,0}, {0x6064,0}, {0x606C,0}};
AKDController::ecat_pdoEntry_t rx[4] = {{0x6040,0}, {0x6072,0},{0x607A,0},{0x60FF,0}}; 
 ```

## Operation

Basic operation consists of calling Update() to update the user buffer, do some application processing, write new values to the user buffer, then calling Update() to update the slave with new inputs. Updating the user buffer will copy inputs from the buffer to the slave and copy outputs from the slave to the buffer. Update's declaration is:
```
int  Update(uint slave, bool move, int timeout_ms);
```
"move" tells the master whether to process a new setpoint or simply update the user buffer. As of now, "move" is only fully tested in homing and profile position mode.

Example:
```
slave1UserBuff.targetPos = 180;
slave2UserBuff.targetPos = 180;

for(int i = 0; i < 3; i++){
    slave1UserBuff.targetPos = -slave1UserBuff.targetPos;
    slave2UserBuff.targetPos = -slave2UserBuff.targetPos;

    master1.Update(0, true, 500000); // 5 sec timeout

    master1.waitForTarget(0,0);
    us_sleep(5000000);
}
```
waitForTarget is another important command during operation. This command simply blocks the caller until the current setpoint has been reached. The declaration is:
```
bool waitForTarget(uint slave, uint timeout_ms);
```

As of now, there is no way to simply check if the target has been reached without blocking the caller. A very low timeout value could be used instead. However, this will fail if multiple setpoints are queued, since there is no functionality to count how many times a target has been reached.

The operation mode of the slave can also be configured using setOpMode. It can be set before or during operation. The declaration is:
```
bool setOpMode(uint slave, ecat_OpModes reqMode);
```
eat_OpModes is a member enum class that has the corresponding numerical operation modes for each mode.

The End?