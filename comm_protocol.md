# CAN ID Structure

EXT CAN Frame, 4 Bytes:

|         | MSB (High) |          |         | LSB (Low)             |
| ------- | ---------- | -------- | ------- | --------------------- |
| Btyes   | Empty      | Robot ID | Unit ID | Device ID in the Unit |
| Example | 0x00       | 0x01     | 0x01    | 0x01                  |

# CAN Message Frame
| data(uint8_t)   | msg[0] | msg[1] | msg[2]      | msg[3]   | msg[4~7]                  |
| --------------- | ------ | ------ | ----------- | -------- | ------------------------- |
| frame structure | header | header | Instruction | Register | Data: msg[4]=H ~ msg[7]=L |
| Example         | 0xCC   | 0xCC   | Operation   | Register | ---                       |
*Operation: READ = 0x02; WRITE = 0x03.

# ASRSDM Control Table

The register is a structured set of data that defines the configuration, status, and control parameters of an unit. Below is an overview of the key components:

## CAN Control Table Overview

## Register

| Address | Name                  | Description                    | Access | Default Value | Type/Range |
| ------- | --------------------- | ------------------------------ | ------ | ------------- | ---------- |
| 0x00    | Model Number          | Unique identifier              | R      | Varies        |            |
| 0x01    | Firmware Version      | Version of the firmware        | R      | Varies        |            |
| 0x02    | ID                    | CAN ID                         | RW     | 1             |            |
|         | Heartbeat             | System check heartneat         | RW     |               |            |
| 0x03    | Baud Rate             | Communication speed            | RW     | 57600         |            |
| 0x04    | Return Delay Time     | Response delay time            | RW     | 250           |            |
| 0x05    | CW Angle Limit        | Clockwise angle limit          | RW     | 0             |            |
| 0x06    | CCW Angle Limit       | Counter-clockwise angle limit  | RW     | 1023          |            |
| 0x07    | Temperature Limit     | Maximum operating temperature  | RW     | 70°C          |            |
| 0x08    | Voltage Limit         | Operating voltage range        | RW     | 12V           |            |
| 0x09    | Max Torque            | Maximum torque limit           | RW     | 1023          |            |
| 0x0A    | Status Return Level   | Status packet return level     | RW     | 2             |            |
| 0x0B    | Alarm LED             | LED for alarm indication       | RW     | 0             |            |
| 0x0C    | Shutdown              | Shutdown conditions            | RW     | 0             |            |
| 0x0D    | Multi-turn Offset     | Offset for multi-turn mode     | RW     | 0             |            |
| 0x0E    | Resolution Divider    | Resolution adjustment          | RW     | 1             |            |
| 0x0F    | Torque Enable         | Motor torque on/off            | RW     | 0             |            |
| 0x10    | LED                   | LED on/off                     | RW     | 0             |            |
| 0x11    | PID Gain              | PID control gain               | RW     | Default       |            |
| 0x12    | Goal Position         | Target position                | RW     | 0             |            |
| 0x13    | Moving Speed          | Speed of movement              | RW     | 0             |            |
| 0x14    | Torque Limit          | Limit for torque               | RW     | 1023          |            |
| 0x15    | Present Position      | Current position               | R      | 0             |            |
| 0x16    | Present Speed         | Current speed                  | R      | 0             |            |
| 0x17    | Present Load          | Current load                   | R      | 0             |            |
| 0x18    | Present Voltage       | Current voltage                | R      | 0             |            |
| 0x19    | Present Temperature   | Current temperature            | R      | 0             |            |
| 0x1A    | Registered            | Instruction registered flag    | R      | 0             |            |
| 0x1B    | Moving                | Movement status                | R      | 0             |            |
| 0x1C    | Lock                  | Lock EEPROM                    | RW     | 0             |            |
| 0x1D    | Punch                 | Minimum current threshold      | RW     | 32            |            |
| 0x1E    | Realtime Tick         | Timer tick for real-time ops   | R      | 0             |            |
| 0x1F    | Current               | Current consumption            | R      | 0             |            |
| 0x20    | Velocity I Gain       | Integral gain for velocity     | RW     | 0             |            |
| 0x21    | Velocity P Gain       | Proportional gain for velocity | RW     | 0             |            |
| 0x22    | Position D Gain       | Derivative gain for position   | RW     | 0             |            |
| 0x23    | Position I Gain       | Integral gain for position     | RW     | 0             |            |
| 0x24    | Position P Gain       | Proportional gain for position | RW     | 0             |            |
| 0x25    | Feedforward 2nd Gain  | 2nd order feedforward gain     | RW     | 0             |            |
| 0x26    | Feedforward 1st Gain  | 1st order feedforward gain     | RW     | 0             |            |
| 0x27    | Bus Watchdog          | Communication watchdog timer   | RW     | 0             |            |
| 0x28    | Goal PWM              | Target PWM value               | RW     | 0             |            |
| 0x29    | Goal Current          | Target current value           | RW     | 0             |            |
| 0x2A    | Goal Velocity         | Target velocity                | RW     | 0             |            |
| 0x2B    | Profile Acceleration  | Acceleration profile           | RW     | 0             |            |
| 0x2C    | Profile Velocity      | Velocity profile               | RW     | 0             |            |
| 0x2D    | Goal Position         | Target position                | RW     | 0             |            |
| 0x2E    | Realtime Tick         | Timer tick for real-time ops   | R      | 0             |            |
| 0x2F    | Moving                | Movement status                | R      | 0             |            |
| 0x30    | Present PWM           | Current PWM value              | R      | 0             |            |
| 0x31    | Present Current       | Current consumption            | R      | 0             |            |
| 0x32    | Present Velocity      | Current velocity               | R      | 0             |            |
| 0x33    | Present Position      | Current position               | R      | 0             |            |
| 0x34    | Velocity Trajectory   | Velocity trajectory            | R      | 0             |            |
| 0x35    | Position Trajectory   | Position trajectory            | R      | 0             |            |
| 0x36    | Present Input Voltage | Current input voltage          | R      | 0             |            |
| 0x37    | Present Temperature   | Current temperature            | R      | 0             |            |
| 0x38    | Backup Ready          | Backup data ready flag         | R      | 0             |            |
| 0x39    | Indirect Address 1    | Indirect address mapping       | RW     | 0             |            |
| 0x3A    | Indirect Address 2    | Indirect address mapping       | RW     | 0             |            |
| 0x3B    | Indirect Address 3    | Indirect address mapping       | RW     | 0             |            |
| 0x3C    | Indirect Address 4    | Indirect address mapping       | RW     | 0             |            |
| 0x3D    | Indirect Address 5    | Indirect address mapping       | RW     | 0             |            |
| 0x3E    | Indirect Address 6    | Indirect address mapping       | RW     | 0             |            |
| 0x3F    | Indirect Address 7    | Indirect address mapping       | RW     | 0             |            |
| 0x40    | Indirect Address 8    | Indirect address mapping       | RW     | 0             |            |
| 0x41    | Indirect Address 9    | Indirect address mapping       | RW     | 0             |            |
| 0x42    | Indirect Address 10   | Indirect address mapping       | RW     | 0             |            |
| 0x43    | Indirect Address 11   | Indirect address mapping       | RW     | 0             |            |
| 0x44    | Indirect Address 12   | Indirect address mapping       | RW     | 0             |            |
| 0x45    | Indirect Address 13   | Indirect address mapping       | RW     | 0             |            |
| 0x46    | Indirect Address 14   | Indirect address mapping       | RW     | 0             |            |
| 0x47    | Indirect Address 15   | Indirect address mapping       | RW     | 0             |            |
| 0x48    | Indirect Address 16   | Indirect address mapping       | RW     | 0             |            |
| 0x49    | Indirect Address 17   | Indirect address mapping       | RW     | 0             |            |
| 0x4A    | Indirect Address 18   | Indirect address mapping       | RW     | 0             |            |
| 0x4B    | Indirect Address 19   | Indirect address mapping       | RW     | 0             |            |
| 0x4C    | Indirect Address 20   | Indirect address mapping       | RW     | 0             |            |
| 0x4D    | Indirect Address 21   | Indirect address mapping       | RW     | 0             |            |
| 0x4E    | Indirect Address 22   | Indirect address mapping       | RW     | 0             |            |
| 0x4F    | Indirect Address 23   | Indirect address mapping       | RW     | 0             |            |
| 0x50    | Indirect Address 24   | Indirect address mapping       | RW     | 0             |            |
| 0x51    | Indirect Address 25   | Indirect address mapping       | RW     | 0             |            |
| 0x52    | Indirect Address 26   | Indirect address mapping       | RW     | 0             |            |
| 0x53    | Indirect Address 27   | Indirect address mapping       | RW     | 0             |            |
| 0x54    | Indirect Address 28   | Indirect address mapping       | RW     | 0             |            |
| 0x55    | Indirect Address 29   | Indirect address mapping       | RW     | 0             |            |
| 0x56    | Indirect Address 30   | Indirect address mapping       | RW     | 0             |            |
| 0x57    | Indirect Address 31   | Indirect address mapping       | RW     | 0             |            |
| 0x58    | Indirect Address 32   | Indirect address mapping       | RW     | 0             |            |
| 0x59    | Indirect Address 33   | Indirect address mapping       | RW     | 0             |            |
| 0x5A    | Indirect Address 34   | Indirect address mapping       | RW     | 0             |            |
| 0x5B    | Indirect Address 35   | Indirect address mapping       | RW     | 0             |            |
| 0x5C    | Indirect Address 36   | Indirect address mapping       | RW     | 0             |            |
| 0x5D    | Indirect Address 37   | Indirect address mapping       | RW     | 0             |            |
| 0x5E    | Indirect Address 38   | Indirect address mapping       | RW     | 0             |            |
| 0x5F    | Indirect Address 39   | Indirect address mapping       | RW     | 0             |            |
| 0x60    | Indirect Address 40   | Indirect address mapping       | RW     | 0             |            |
| 0x61    | Indirect Address 41   | Indirect address mapping       | RW     | 0             |            |
| 0x62    | Indirect Address 42   | Indirect address mapping       | RW     | 0             |            |
| 0x63    | Indirect Address 43   | Indirect address mapping       | RW     | 0             |            |
| 0x64    | Indirect Address 44   | Indirect address mapping       | RW     | 0             |            |
| 0x65    | Indirect Address 45   | Indirect address mapping       | RW     | 0             |            |
| 0x66    | Indirect Address 46   | Indirect address mapping       | RW     | 0             |            |
| 0x67    | Indirect Address 47   | Indirect address mapping       | RW     | 0             |            |
| 0x68    | Indirect Address 48   | Indirect address mapping       | RW     | 0             |            |
| 0x69    | Indirect Address 49   | Indirect address mapping       | RW     | 0             |            |
| 0x6A    | Indirect Address 50   | Indirect address mapping       | RW     | 0             |            |
| 0x6B    | Indirect Address 51   | Indirect address mapping       | RW     | 0             |            |
| 0x6C    | Indirect Address 52   | Indirect address mapping       | RW     | 0             |            |
| 0x6D    | Indirect Address 53   | Indirect address mapping       | RW     | 0             |            |
| 0x6E    | Indirect Address 54   | Indirect address mapping       | RW     | 0             |            |
| 0x6F    | Indirect Address 55   | Indirect address mapping       | RW     | 0             |            |
| 0x70    | Indirect Address 56   | Indirect address mapping       | RW     | 0             |            |
| 0x71    | Indirect Address 57   | Indirect address mapping       | RW     | 0             |            |
| 0x72    | Indirect Address 58   | Indirect address mapping       | RW     | 0             |            |
| 0x73    | Indirect Address 59   | Indirect address mapping       | RW     | 0             |            |
| 0x74    | Indirect Address 60   | Indirect address mapping       | RW     | 0             |            |
| 0x75    | Indirect Address 61   | Indirect address mapping       | RW     | 0             |            |
| 0x76    | Indirect Address 62   | Indirect address mapping       | RW     | 0             |            |
| 0x77    | Indirect Address 63   | Indirect address mapping       | RW     | 0             |            |
| 0x78    | Indirect Address 64   | Indirect address mapping       | RW     | 0             |            |
| 0x79    | Indirect Address 65   | Indirect address mapping       | RW     | 0             |            |
| 0x7A    | Indirect Address 66   | Indirect address mapping       | RW     | 0             |            |
| 0x7B    | Indirect Address 67   | Indirect address mapping       | RW     | 0             |            |
| 0x7C    | Indirect Address 68   | Indirect address mapping       | RW     | 0             |            |
| 0x7D    | Indirect Address 69   | Indirect address mapping       | RW     | 0             |            |
| 0x7E    | Indirect Address 70   | Indirect address mapping       | RW     | 0             |            |
| 0x7F    | Indirect Address 71   | Indirect address mapping       | RW     | 0             |            |
| 0x80    | Indirect Address 72   | Indirect address mapping       | RW     | 0             |            |
| 0x81    | Indirect Address 73   | Indirect address mapping       | RW     | 0             |            |
| 0x82    | Indirect Address 74   | Indirect address mapping       | RW     | 0             |            |
| 0x83    | Indirect Address 75   | Indirect address mapping       | RW     | 0             |            |
| 0x84    | Indirect Address 76   | Indirect address mapping       | RW     | 0             |            |
| 0x85    | Indirect Address 77   | Indirect address mapping       | RW     | 0             |            |
| 0x86    | Indirect Address 78   | Indirect address mapping       | RW     | 0             |            |
| 0x87    | Indirect Address 79   | Indirect address mapping       | RW     | 0             |            |
| 0x88    | Indirect Address 80   | Indirect address mapping       | RW     | 0             |            |
| 0x89    | Indirect Address 81   | Indirect address mapping       | RW     | 0             |            |
| 0x8A    | Indirect Address 82   | Indirect address mapping       | RW     | 0             |            |
| 0x8B    | Indirect Address 83   | Indirect address mapping       | RW     | 0             |            |
| 0x8C    | Indirect Address 84   | Indirect address mapping       | RW     | 0             |            |
| 0x8D    | Indirect Address 85   | Indirect address mapping       | RW     | 0             |            |
| 0x8E    | Indirect Address 86   | Indirect address mapping       | RW     | 0             |            |
| 0x8F    | Indirect Address 87   | Indirect address mapping       | RW     | 0             |            |
| 0x90    | Indirect Address 88   | Indirect address mapping       | RW     | 0             |            |
| 0x91    | Indirect Address 89   | Indirect address mapping       | RW     | 0             |            |
| 0x92    | Indirect Address 90   | Indirect address mapping       | RW     | 0             |            |

## Notes
- **Access Types**: 
    - `R`: Read-only
    - `RW`: Read/Write
- Always refer to the specific Dynamixel model's documentation for exact details, as the control table may vary.

## References
- [Dynamixel Official Documentation](https://emanual.robotis.com/)