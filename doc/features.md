# Features
## Implemented and Tested

- No timing requirements for calling process
- Uses an asynchronously updated buffer
- Multiple slaves supported
- Velocity Profile Control
- Position profile control
- Can queue setpoint(s) without blocking
- Wait for target to be reached
- Set PDO assignments
- Set entries for flexible PDO mappings
- Detects unsuitable mappings
- Auto fits entries into PDOs
- Homing
- Only zero current position tested
- Digital I/O
- Disable/enable outputs
- Set output mode
- Set output value
- Change Ethercat slave modes
- Enable (Operation Enable)
- Disable (Operation Disable/Switched On)
- Stop (Ready to Switch On)
- Shutdown (Close down master)
- Reads when fault detected
- Clears faults

## Needs tested
- Interpolated Position Control
- Torque Position Control
- Cyclical Position Control
- Different homing modes. Only zero current position is tested.
- How different user functions behave when motor faults. (waitForTarget, Update, etc.)

## Needs added
- Set Feed Rates
- Toggle modulus position mode
- Axis control mode? Feed, degrees, etc
- Fault handling
- Write and read to EEPROM (Pretty sure this is possible)
- Save parameters from memory
- Load parameters from memory

## Could be added or investigated
- Read fault code for user to handle
- Even when configuring units, values are scaled awkwardly. ex 500 rpm -> 583 rpm with 7/360 units during test
- Be able to setup user buffer without configuring PDOS. (In case the current assignments want to be used?)

