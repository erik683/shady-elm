# Test Suite for Shady Elm CAN Bridge

This directory contains test scripts for the Shady Elm CAN bridge project.

## Test Files

### `test_mscan_config.py`
Comprehensive test suite for Ford MS-CAN configuration and functionality.

**What it tests:**
- MS-CAN frame parsing (11-bit IDs, 125 kHz)
- GVRET binary protocol conversion
- GVRET CSV and CRTD text format output
- Configuration file loading and validation
- Mock serial communication simulation
- TCP server and GVRET protocol handling
- Config integration with main script
- Script execution with MS-CAN config
- Error conditions and edge cases
- Security validation (path traversal prevention)

### `test_hscan_config.py`
Comprehensive test suite for STN1170 HS-CAN configuration and functionality.

**What it tests:**
- HS-CAN frame parsing (11-bit IDs, 500 kHz)
- GVRET binary protocol conversion
- GVRET CSV and CRTD text format output
- Configuration file loading, flattening, and alias resolution
- Mock serial communication simulation
- TCP/GVRET command handling
- End-to-end file-only GVRET CSV log generation

**Usage:**
```bash
python tests/test_mscan_config.py
python tests/test_hscan_config.py
```

**Expected Output:**
- All tests should pass with `[OK]` status
- Shows parsed CAN frames in multiple formats
- Validates MS-CAN specific settings (STN protocol 51, 125 kHz, 11-bit raw CAN)
- Validates HS-CAN specific settings (STN protocol 31, 500 kHz, 11-bit raw CAN)
- Demonstrates mock STN1170 communication
- Confirms GVRET protocol compatibility

## Test Results

The test suite validates that:
1. ✅ MS-CAN frames are parsed correctly from STN1170 output
2. ✅ GVRET binary protocol conversion works
3. ✅ Multiple output formats (GVRET CSV, CRTD) function properly
4. ✅ Configuration file is properly structured for MS-CAN
5. ✅ Mock serial communication simulates real STN1170 behavior
6. ✅ TCP server and GVRET protocol handle SavvyCAN requests
7. ✅ Config integration works with the main script
8. ✅ Script can be executed with MS-CAN configuration
9. ✅ Error conditions are handled properly
10. ✅ Security vulnerabilities are prevented

## Running Tests

From the project root directory:
```bash
# Run the MS-CAN test suite
python tests/test_mscan_config.py

# Run the HS-CAN test suite
python tests/test_hscan_config.py

# Run the main bridge with MS-CAN config
python savvycan_bridge.py --config STN1170_MSCAN_125000 -p COM3

# Run the main bridge with HS-CAN config
python savvycan_bridge.py --config STN1170_HSCAN_500000 -p COM3
```

## Test Coverage

The test suite covers:
- **Frame Parsing**: Both spaced and non-spaced STN1170 output formats
- **Protocol Conversion**: ELM327 → GVRET binary → SavvyCAN
- **Configuration**: JSON config loading, flattening, and validation
- **Communication**: Mock serial port simulation
- **Network**: TCP server and GVRET protocol handling
- **Integration**: End-to-end script execution testing
- **Error Handling**: Invalid inputs, malformed data, and edge cases
- **Security**: Path traversal prevention and input validation

This ensures the MS-CAN configuration is production-ready for Ford vehicles.
