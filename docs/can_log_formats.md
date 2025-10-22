# CAN Log Format Implementation Guide

## Overview
This guide provides exact specifications for implementing three CAN logging formats: GVRET, CRTD, and BLF (Vector Binary Log Format).

---

## 1. GVRET Format

### Description
GVRET is a simple comma-separated ASCII format created for the GVRET project. Human-readable and easy to parse.

### Format Specification
```
timestamp,id,extended,bus,length,byte0,byte1,byte2,...,byteN\r\n
```

### Field Definitions
| Field | Type | Description |
|-------|------|-------------|
| timestamp | integer | Milliseconds since start of logging |
| id | hex | CAN identifier (no 0x prefix) |
| extended | integer | 0 = 11-bit ID, 1 = 29-bit ID |
| bus | integer | CAN bus number (0, 1, 2, etc.) |
| length | integer | Number of data bytes (0-8) |
| byteN | hex | Data bytes (no 0x prefix, uppercase recommended) |

### Line Ending
- Windows: `\r\n` (CRLF)
- Unix/Linux: `\n` (LF) also acceptable

### Example Output
```
0,7DF,0,0,8,02,01,0D,55,55,55,55,55
15,7E8,0,0,8,10,14,49,02,01,31,46,4D
23,7E8,0,0,8,21,43,55,31,4C,39,46,30
45,18FEF100,1,1,8,FF,FF,FF,FF,FF,00,64,00
```

### Implementation Template (C++)
```cpp
void writeGVRET(FILE* file, uint32_t timestamp_ms, uint32_t id, 
                bool extended, uint8_t bus, uint8_t length, 
                const uint8_t* data) {
    // Write timestamp, id, extended, bus, length
    fprintf(file, "%u,%X,%d,%d,%d", 
            timestamp_ms, id, extended ? 1 : 0, bus, length);
    
    // Write data bytes
    for (int i = 0; i < length; i++) {
        fprintf(file, ",%02X", data[i]);
    }
    
    // Line ending
    fprintf(file, "\r\n");
}
```

### Implementation Template (Python)
```python
def write_gvret(file, timestamp_ms, can_id, extended, bus, length, data):
    """Write a CAN frame in GVRET format"""
    # Format data bytes as hex
    data_str = ','.join(f'{b:02X}' for b in data[:length])
    
    # Write line
    line = f"{timestamp_ms},{can_id:X},{1 if extended else 0},{bus},{length}"
    if length > 0:
        line += f",{data_str}"
    line += "\r\n"
    
    file.write(line)
```

---

## 2. CRTD Format

### Description
CRTD (CAN/CAN-FD Reverse Engineering Tool Data) is a space-separated format used by SavvyCAN and other reverse engineering tools.

### Format Specification
```
timestamp R[bits] id byte0 byte1 byte2 ... byteN\r\n
```

### Field Definitions
| Field | Type | Description |
|-------|------|-------------|
| timestamp | float | Seconds since start (with decimals) |
| R[bits] | literal | R11 for standard, R29 for extended |
| id | hex | CAN identifier (uppercase, no 0x prefix) |
| byteN | hex | Data bytes (space-separated, uppercase) |

### Important Notes
- Timestamp is in **seconds** (not milliseconds)
- Use at least 6 decimal places for timestamp precision
- No commas, only spaces as separators
- No length field (implied by number of bytes)

### Example Output
```
0.000000 R11 7DF 02 01 0D 55 55 55 55 55
0.015000 R11 7E8 10 14 49 02 01 31 46 4D
0.023000 R11 7E8 21 43 55 31 4C 39 46 30
0.045000 R29 18FEF100 FF FF FF FF FF 00 64 00
1.234567 R11 100 DE AD BE EF
```

### Optional Header
Some tools support an optional header:
```
// CAN Log File
// Generated: 2024-01-15 10:30:45
// Format: CRTD v2.0
```

### Implementation Template (C++)
```cpp
void writeCRTD(FILE* file, double timestamp_sec, uint32_t id, 
               bool extended, uint8_t length, const uint8_t* data) {
    // Write timestamp and frame type
    fprintf(file, "%.6f R%d %X", 
            timestamp_sec, extended ? 29 : 11, id);
    
    // Write data bytes
    for (int i = 0; i < length; i++) {
        fprintf(file, " %02X", data[i]);
    }
    
    // Line ending
    fprintf(file, "\r\n");
}
```

### Implementation Template (Python)
```python
def write_crtd(file, timestamp_sec, can_id, extended, length, data):
    """Write a CAN frame in CRTD format"""
    # Determine frame type
    frame_type = "R29" if extended else "R11"
    
    # Format data bytes
    data_str = ' '.join(f'{b:02X}' for b in data[:length])
    
    # Write line
    line = f"{timestamp_sec:.6f} {frame_type} {can_id:X}"
    if length > 0:
        line += f" {data_str}"
    line += "\r\n"
    
    file.write(line)
```

---

## 3. BLF Format (Vector Binary Log Format)

### Description
BLF is Vector Informatik's binary logging format. It's the industry standard for professional CAN logging with excellent compression and performance.

### ⚠️ Important Warning
BLF is a **complex binary format** with:
- Multiple versions (BLF2, BLF3, BLF4)
- Compression support (zlib)
- Many object types (CAN, CAN-FD, LIN, FlexRay, etc.)
- Checksums and headers

**Recommendation:** Use Vector's official BLF library rather than implementing from scratch.

### Official Implementation Options

#### Option 1: Vector BLF Library (Recommended)
Vector provides official libraries:
- **C++ API**: Part of Vector's XL Driver Library
- **Python**: `python-can` library with BLF support
- Download: https://www.vector.com/

#### Option 2: Open-Source Library
- **Library**: `vector_blf` (Python)
- **Install**: `pip install vector-blf`
- **GitHub**: https://github.com/Murmele/vector_blf

### Basic BLF Structure (For Reference)

```
[File Header]
  - Signature: "LOGG"
  - Version info
  - Start timestamp
  
[Object Headers + Data]
  Each object:
    - Object header (base size, type, flags)
    - Object-specific data
    - Optional: compression
    
[Object Types for CAN]
  - CAN_MESSAGE (0x0001)
  - CAN_MESSAGE2 (0x0056) 
  - CAN_FD_MESSAGE (0x0064)
  - CAN_ERROR_FRAME
```

### Implementation Template (Python - Using vector_blf)

```python
from vector_blf import BLFWriter, CANMessage
from datetime import datetime

def create_blf_logger(filename):
    """Create a BLF file for logging"""
    writer = BLFWriter(filename)
    return writer

def write_can_to_blf(writer, timestamp_ns, channel, can_id, 
                     extended, length, data):
    """Write a CAN message to BLF file"""
    msg = CANMessage()
    msg.timestamp = timestamp_ns  # Nanoseconds since 1970-01-01
    msg.channel = channel         # 1, 2, 3, etc.
    msg.id = can_id
    msg.flags = 0x01 if extended else 0x00  # Extended flag
    msg.dlc = length
    msg.data = bytes(data[:length])
    
    writer.write(msg)

# Example usage
writer = create_blf_logger("canlog.blf")
write_can_to_blf(writer, 1234567890000000, 1, 0x123, False, 8, 
                 [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08])
writer.close()
```

### Implementation Template (C++ - Using Vector XL Library)

```cpp
#include "vxlapi.h"  // Vector XL Driver Library

// Initialize
XLhandle xlHandle;
XLaccess xlChannelMask;
XLevent xlEvent;

// Create BLF file
XL_OpenLogFile(&xlHandle, "canlog.blf", 
               XL_CREATE_FILE, XL_BL_TYPE_BLF);

// Write CAN message
void writeCANtoBLF(uint64_t timestamp_ns, uint8_t channel,
                   uint32_t id, bool extended, uint8_t dlc,
                   const uint8_t* data) {
    memset(&xlEvent, 0, sizeof(xlEvent));
    
    xlEvent.tag = XL_CAN_EV_TAG_RX_OK;
    xlEvent.timeStamp = timestamp_ns;
    xlEvent.tagData.canRxOkMsg.canId = id;
    xlEvent.tagData.canRxOkMsg.msgFlags = 
        extended ? XL_CAN_MSG_FLAG_EXTENDED : 0;
    xlEvent.tagData.canRxOkMsg.dlc = dlc;
    memcpy(xlEvent.tagData.canRxOkMsg.data, data, dlc);
    
    XL_WriteLogEvent(xlHandle, &xlEvent);
}

// Close file
XL_CloseLogFile(xlHandle);
```

### BLF Timestamp Format
```
- Unit: Nanoseconds (10^-9 seconds)
- Epoch: January 1, 1970 00:00:00 UTC
- Type: 64-bit unsigned integer
- Range: Allows dates from 1970 to ~2554

Example conversion:
Seconds since 1970: 1705320000
Nanoseconds: 1705320000000000000
```

---

## Format Comparison Table

| Feature | GVRET | CRTD | BLF |
|---------|-------|------|-----|
| **Type** | Text | Text | Binary |
| **Timestamp** | ms (int) | seconds (float) | ns (int64) |
| **File Size** | Medium | Medium | Small (compressed) |
| **Parse Speed** | Fast | Fast | Very Fast |
| **Human Readable** | Yes | Yes | No |
| **Tool Support** | Limited | Good | Excellent |
| **Metadata** | Minimal | Minimal | Extensive |
| **Complexity** | Simple | Simple | Complex |

---

## Complete Example: Multi-Format Logger

```cpp
class CANLogger {
public:
    enum Format { GVRET, CRTD, BLF };
    
    CANLogger(const char* filename, Format fmt) {
        format = fmt;
        if (fmt == BLF) {
            // Use Vector library
            initBLF(filename);
        } else {
            file = fopen(filename, "w");
        }
        start_time = getCurrentTime();
    }
    
    void logFrame(uint32_t id, bool extended, uint8_t bus,
                  uint8_t length, const uint8_t* data) {
        switch(format) {
            case GVRET:
                writeGVRET(file, getElapsedMS(), id, extended, 
                          bus, length, data);
                break;
            case CRTD:
                writeCRTD(file, getElapsedSec(), id, extended,
                         length, data);
                break;
            case BLF:
                writeCANtoBLF(getElapsedNS(), bus, id, extended,
                             length, data);
                break;
        }
    }
    
private:
    Format format;
    FILE* file;
    uint64_t start_time;
    
    uint32_t getElapsedMS() { 
        return (getCurrentTime() - start_time) / 1000000; 
    }
    double getElapsedSec() { 
        return (getCurrentTime() - start_time) / 1000000000.0; 
    }
    uint64_t getElapsedNS() { 
        return getCurrentTime() - start_time; 
    }
};
```

---

## Testing Your Implementation

### Test Cases

**Standard Frame (11-bit ID)**
```
ID: 0x123
Extended: false
Bus: 0
DLC: 8
Data: 01 02 03 04 05 06 07 08
Time: 1.234567 seconds

Expected GVRET:
1234,123,0,0,8,01,02,03,04,05,06,07,08

Expected CRTD:
1.234567 R11 123 01 02 03 04 05 06 07 08
```

**Extended Frame (29-bit ID)**
```
ID: 0x18FEF100
Extended: true
Bus: 1
DLC: 4
Data: AA BB CC DD
Time: 5.678901 seconds

Expected GVRET:
5678,18FEF100,1,1,4,AA,BB,CC,DD

Expected CRTD:
5.678901 R29 18FEF100 AA BB CC DD
```

**Zero-Length Frame**
```
ID: 0x100
Extended: false
Bus: 0
DLC: 0
Data: (none)
Time: 0.000000 seconds

Expected GVRET:
0,100,0,0,0

Expected CRTD:
0.000000 R11 100
```

---

## Validation Tools

- **GVRET**: Load in SavvyCAN (supports both)
- **CRTD**: Load in SavvyCAN, CANgaroo
- **BLF**: Load in Vector CANalyzer, CANoe, SavvyCAN

---

## Common Pitfalls

### GVRET
- ❌ Including "0x" prefix on hex values
- ❌ Using lowercase hex digits (uppercase preferred)
- ❌ Wrong timestamp units (must be milliseconds)

### CRTD  
- ❌ Using milliseconds instead of seconds
- ❌ Insufficient timestamp precision (use 6+ decimals)
- ❌ Using commas instead of spaces
- ❌ Writing R29 for standard frames or R11 for extended

### BLF
- ❌ Attempting to implement from scratch
- ❌ Wrong timestamp epoch or units
- ❌ Not handling endianness correctly
- ❌ Missing compression support

---

## Recommendations

**For Your Logger:**

1. **Primary Format: CRTD**
   - Easy to implement
   - Wide tool support
   - Good for most users

2. **Secondary Format: GVRET**  
   - Includes explicit bus field
   - Slightly easier to parse programmatically

3. **Professional Format: BLF**
   - Use Vector's library
   - Best performance and features
   - Required for professional customers

**Implementation Priority:**
1. Start with CRTD (1-2 hours)
2. Add GVRET (30 minutes)
3. Integrate BLF library if needed (1 day)

