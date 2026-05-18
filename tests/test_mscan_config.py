#!/usr/bin/env python3
"""
Test script for MS-CAN configuration
Simulates STN1170 output for Ford MS-CAN (125kHz, 11-bit) and tests parsing
Tests the main script functionality with mock serial data
"""

import os
import queue
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock, PropertyMock, patch

# Add parent directory to path to import the bridge module
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import savvycan_bridge


CONFIG_PROFILE = "STN1170_MSCAN_125000"

# Sample MS-CAN frames from a Ford vehicle
# MS-CAN uses 11-bit IDs and 125 kHz bitrate
# Common Ford MS-CAN IDs: 0x201, 0x215, 0x3B3, 0x3B5, etc.
test_frames = [
    # Format: Raw line from STN1170 as it would appear on serial
    "201 8 00 00 7D 00 27 10 FF FF",  # Typical HVAC frame (spaced format)
    "215 8 00 00 00 00 00 00 00 00",  # Climate control
    "3B3 8 00 00 00 00 00 00 00 00",  # Body control module
    "3B5 8 00 00 00 00 00 00 00 00",  # Instrument cluster
    "420 4 12 34 56 78",              # Shorter frame
    "20100007D002710FFFF",            # Non-spaced format
    "2158000000000000000",            # Non-spaced format
]


def require(condition, message):
    if not condition:
        raise AssertionError(message)
    print(f"[OK] {message}")


def load_mscan_config():
    config_path = savvycan_bridge.resolve_config_path(CONFIG_PROFILE)
    config = savvycan_bridge.load_config(config_path)
    return config_path, config


def test_elm_parser():
    print("=" * 60)
    print("Testing MS-CAN Frame Parsing")
    print("=" * 60)

    for i, test_line in enumerate(test_frames, 1):
        print(f"\nTest Frame {i}:")
        print(f"  Raw Input:  '{test_line}'")

        # Parse the frame
        frame_id, data, is_extended, bus = savvycan_bridge.parse_elm_frame(test_line, default_protocol=7)

        require(frame_id is not None, f"Frame {i} parsed")
        assert frame_id is not None

        require(is_extended is not None, f"Frame {i} extension flag is present")
        assert is_extended is not None

        require(data is not None, f"Frame {i} payload is present")
        assert data is not None

        require(bus is not None, f"Frame {i} bus is present")
        assert bus is not None

        # Narrow types for the type checker
        is_extended = bool(is_extended)
        bus = int(bus)

        require(not is_extended, f"Frame {i} is standard 11-bit")
        require(bus == 0, f"Frame {i} uses bus 0")
        require(len(data) <= 8, f"Frame {i} payload length is classic CAN compatible")

        # Build GVRET binary frame to verify it works
        gvret_frame = savvycan_bridge.build_gvret_can_frame(frame_id, bus, data, is_extended)
        require(gvret_frame.startswith(bytes([savvycan_bridge.GVRET_COMMAND_ID, 0x00])),
                f"Frame {i} builds a GVRET binary frame")

        # Format as GVRET CSV
        ts_ms = int(time.time() * 1000)
        gvret_csv = savvycan_bridge.format_gvret_csv_line(ts_ms, frame_id, is_extended, bus, data)
        require(len(gvret_csv.strip().split(",")) == 14, f"Frame {i} formats as GVRET CSV")

        # Format as CRTD
        ts_sec = time.time()
        crtd = savvycan_bridge.format_crtd_line(ts_sec, frame_id, is_extended, bus, data)
        require(" R11 " in crtd, f"Frame {i} formats as CRTD R11")

def test_config_loading():
    print("\n" + "=" * 60)
    print("Testing MS-CAN Config File")
    print("=" * 60)

    config_path, config = load_mscan_config()
    print(f"\nLoaded config: {config_path}")

    require(config["can"]["protocol"] == 7, "Protocol is 7 for 125 kbps 11-bit MS-CAN")
    require(config["device"]["bus_bitrate"] == 125000, "Bus bitrate is 125000")
    require(config["can"]["standard_only"] is True, "standard_only is enabled")
    require(config["can"]["extended_only"] is False, "extended_only is disabled")
    require(config["performance"]["batch_size"] == 3, "Batch size matches MS-CAN profile")

def test_protocol_commands():
    """Show the AT/ST commands that will be sent for MS-CAN"""
    print("\n" + "=" * 60)
    print("Expected STN1170 Initialization Commands for MS-CAN")
    print("=" * 60)
    print("\nCommands that will be sent to STN1170:")
    print("  1. ATZ             - Reset device")
    print("  2. ATE 0           - Echo off")
    print("  3. ATL 0           - Linefeeds off")
    print("  4. ATS 0           - Spaces off")
    print("  5. ATH 1           - Headers on (show CAN IDs)")
    print("  6. ATCAF 0         - CAN Auto Formatting off")
    print("  7. ATCFC0          - CAN Flow Control off")
    print("  8. ATCSM 1         - Silent mode (monitor only)")
    print("  9. STP 7           - Protocol 7 (ISO 15765-4, 11-bit, 125 kbaud) [KEY]")
    print(" 10. STPTO 20        - Protocol timeout 20ms")
    print(" 11. STCMM 1         - CAN Monitor Mode normal")
    print(" 12. STMA            - Start monitoring")
    print("\nKey Setting: STP 7 = ISO 15765-4 CAN (11-bit ID, 125 kbaud)")
    print("This is the critical difference from HS-CAN (STP 6 = 500 kbaud)")

def test_mock_serial_communication():
    print("\n" + "=" * 60)
    print("Testing Mock Serial Communication")
    print("=" * 60)

    # Create mock serial object
    mock_ser = Mock()
    mock_ser.in_waiting = 0
    serial_chunks = iter([f"{frame}\r\n".encode("ascii") for frame in test_frames[:3]])
    mock_ser.read.side_effect = lambda size: next(serial_chunks, b"")

    # Create queues for communication
    tcp_queue = queue.Queue()
    stop_event = threading.Event()

    print("\nSimulating STN1170 MS-CAN data stream...")

    # Start serial reader in a thread
    serial_thread = threading.Thread(
        target=savvycan_bridge.serial_reader,
        args=(mock_ser, tcp_queue, stop_event),
        kwargs={
            "quiet": True,
            "batch_size": 3,
            "batch_timeout": 0.01,
            "extended_only": False,
            "standard_only": True,
            "output_format": "binary",
            "protocol": 7,  # MS-CAN
        },
    )

    serial_thread.start()
    time.sleep(0.1)  # Let it process some data
    stop_event.set()
    serial_thread.join(timeout=2)
    require(not serial_thread.is_alive(), "Serial reader thread stopped")
    require(tcp_queue.qsize() == 3, "Mock serial stream produced three GVRET frames")

def test_tcp_server_functionality():
    print("\n" + "=" * 60)
    print("Testing TCP Server and GVRET Protocol")
    print("=" * 60)

    # Test GVRET command handling
    print("\nTesting GVRET command responses...")

    # Mock client socket
    mock_socket = Mock()
    mock_socket.sendall = Mock()

    # Test device info request (0x07)
    device_info_cmd = bytes([0xF1, 0x07])
    serial_queue = queue.Queue()

    savvycan_bridge.gvret_command_handler(mock_socket, device_info_cmd, serial_queue)
    require(mock_socket.sendall.called, "GVRET device info command handled correctly")

    # Test CAN bus params request (0x06)
    mock_socket.reset_mock()
    can_params_cmd = bytes([0xF1, 0x06])

    savvycan_bridge.gvret_command_handler(mock_socket, can_params_cmd, serial_queue)
    require(mock_socket.sendall.called, "GVRET CAN bus params command handled correctly")

def test_config_integration():
    print("\n" + "=" * 60)
    print("Testing Config Integration with Main Script")
    print("=" * 60)

    # Test config path resolution
    config_path = savvycan_bridge.resolve_config_path(CONFIG_PROFILE)
    expected_path = os.path.join("configs", "STN1170_MSCAN_125000.json")

    require(config_path.endswith(expected_path), "Config path resolution works correctly")

    # Test config loading and flattening
    config = savvycan_bridge.load_config(config_path)
    flat_config = savvycan_bridge.flatten_config(config)

    # Check key settings are in flattened config
    require(flat_config.get("protocol") == 7, "Protocol setting correctly flattened")
    require(flat_config.get("bus_bitrate") == 125000, "Bus bitrate setting correctly flattened")

def test_script_execution():
    print("\n" + "=" * 60)
    print("Testing Script Execution with MS-CAN Config")
    print("=" * 60)

    # Test argument parsing with config
    print("\nTesting argument parsing with MS-CAN config...")

    # Mock the argument parser to avoid actual execution
    with patch("sys.argv", ["savvycan_bridge.py", "--config", CONFIG_PROFILE, "--port", "COM99"]):
        # This will test the config loading part without actually running the bridge
        pre_parser = savvycan_bridge.argparse.ArgumentParser(add_help=False)
        pre_parser.add_argument("--config", help="Config file path or profile name")
        pre_args, _ = pre_parser.parse_known_args()

        config_path = savvycan_bridge.resolve_config_path(pre_args.config)
        require(os.path.exists(config_path), "Script can resolve MS-CAN config successfully")

        runtime_config = {}
        runtime_config.update(savvycan_bridge.load_config(config_path))
        flat_config = savvycan_bridge.flatten_config(runtime_config)

        require(len(flat_config) > 0, "Script can flatten MS-CAN config settings")

def test_error_conditions():
    print("\n" + "=" * 60)
    print("Testing Error Conditions and Edge Cases")
    print("=" * 60)

    # Test invalid frame parsing
    invalid_frames = [
        "",  # Empty string
        "   ",  # Whitespace only
        "INVALID",  # Invalid format
        "XYZ123",  # Invalid hex
        "1234567890123456789012345678901234567890",  # Too long
    ]

    print("\nTesting invalid frame parsing...")
    for i, frame in enumerate(invalid_frames, 1):
        result = savvycan_bridge.parse_elm_frame(frame)
        require(result == (None, None, None, None), f"Invalid frame {i} correctly rejected")

    # Test config path security
    print("\nTesting config path security...")
    try:
        # This should fail with path traversal attempt
        savvycan_bridge.resolve_config_path("../../../etc/passwd.json")
    except ValueError:
        require(True, "Path traversal attack correctly prevented")
    except Exception as e:
        require(True, f"Path traversal prevented with different exception: {e}")
    else:
        raise AssertionError("Path traversal attack not prevented")

    # Test config loading with invalid JSON
    print("\nTesting invalid config handling...")
    try:
        # This will try to load a non-existent config
        savvycan_bridge.load_config("nonexistent.json")
    except FileNotFoundError:
        require(True, "Non-existent config file correctly raises FileNotFoundError")
    except Exception as e:
        require(True, f"Non-existent config handled: {e}")
    else:
        raise AssertionError("Non-existent config file should raise exception")

def test_log_generation_and_validation():
    print("\n" + "=" * 60)
    print("Testing Log Generation and Validation")
    print("=" * 60)

    fd, log_file = tempfile.mkstemp(prefix="mscan_output_", suffix=".gvret")
    os.close(fd)
    os.remove(log_file)

    # Mock CLI arguments for the bridge
    args = SimpleNamespace(
        config=CONFIG_PROFILE,
        port="COM_MOCK",
        output_file=log_file,
        file_only=True,
        format="gvret",
        log_file=None,
        quiet=True,
        debug=False,
        list_ports=False,
        env_check=False,
        disable_flow_control=True,
        force_flow_control=None,
        test_flow_control=False,
        skip_init=True,
    )

    # Load and flatten config to set defaults on args
    config_path = savvycan_bridge.resolve_config_path(args.config)
    config = savvycan_bridge.load_config(config_path)
    defaults = savvycan_bridge.flatten_config(config)
    for key, value in defaults.items():
        if not hasattr(args, key):
            setattr(args, key, value)

    # Mock serial object with test data
    mock_serial_instance = MagicMock()

    # Format test frames as they would come from the serial port
    serial_data = [f"{frame}\r".encode("ascii") for frame in test_frames]

    # Use a generator to provide a continuous stream of data
    def serial_data_generator():
        for item in serial_data:
            yield item
        while not savvycan_bridge.shutdown_event.is_set():
            time.sleep(0.01)  # Prevent busy-waiting
            yield b""  # Yield empty bytes until shutdown

    data_gen = serial_data_generator()
    # The lambda function will call next() on the generator for each read
    mock_serial_instance.read.side_effect = lambda size: next(data_gen)
    type(mock_serial_instance).in_waiting = PropertyMock(
        side_effect=lambda: 0 if savvycan_bridge.shutdown_event.is_set() else (len(serial_data[0]) if serial_data else 0)
    )
    mock_serial_instance.read_until.return_value = b'>'
    mock_serial_instance.is_open = True

    try:
        # Use patch to replace serial.Serial with our mock
        with patch("serial.Serial", return_value=mock_serial_instance):
            with patch("savvycan_bridge.is_port_available", return_value=True):
                with patch("savvycan_bridge.setup_signal_handlers", return_value=None):
                    print(f"\nRunning bridge in file-only mode, output to '{log_file}'...")
                    bridge_thread = threading.Thread(target=lambda: savvycan_bridge.run_bridge(args))
                    bridge_thread.start()
                    time.sleep(1.0)
                    savvycan_bridge.shutdown_event.set()
                    bridge_thread.join(timeout=2.0)

        require(os.path.exists(log_file), "GVRET CSV log file was created")
        with open(log_file, "r", encoding="utf-8") as f:
            lines = f.readlines()

        # 1. Check for header
        expected_header = "Time Stamp,ID,Extended,Dir,Bus,LEN,D1,D2,D3,D4,D5,D6,D7,D8\n"
        require(lines and lines[0] == expected_header, "GVRET CSV header is correct")

        # 2. Check number of data lines
        # -1 for the header line
        require(len(lines) - 1 == len(test_frames), "GVRET CSV log contains every test frame")

        # 3. Check format of a sample line
        require(all(len(line.strip().split(",")) == 14 for line in lines[1:]),
                "GVRET CSV data lines have the expected columns")
    finally:
        # Clean up the log file
        if os.path.exists(log_file):
            os.remove(log_file)
        savvycan_bridge.shutdown_event.clear()  # Reset for other tests

def main():
    """Run all tests"""
    print("\n" + "=" * 60)
    print("Ford MS-CAN Configuration Test Suite")
    print("=" * 60)
    
    # Test 1: Parse MS-CAN frames
    test_elm_parser()
    
    # Test 2: Verify config file
    test_config_loading()
    
    # Test 3: Show protocol commands
    test_protocol_commands()
    
    # Test 4: Mock serial communication
    test_mock_serial_communication()
    
    # Test 5: TCP server functionality
    test_tcp_server_functionality()
    
    # Test 6: Config integration
    test_config_integration()
    
    # Test 7: Script execution
    test_script_execution()
    
    # Test 8: Error conditions
    test_error_conditions()

    # Test 9: Log generation and validation
    test_log_generation_and_validation()
    
    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)
    print("\n[OK] MS-CAN frame parsing works correctly")
    print("[OK] GVRET binary protocol conversion works")
    print("[OK] GVRET CSV and CRTD text formats work")
    print("[OK] Configuration file is properly structured")
    print("[OK] Mock serial communication tested")
    print("[OK] TCP server and GVRET protocol tested")
    print("[OK] Config integration with main script verified")
    print("[OK] Error conditions and edge cases tested")
    print("[OK] Log generation and validation tested")
    print("\nThe STN1170_MSCAN_125000.json config is ready for production use!")
    print("Run: python savvycan_bridge.py --config STN1170_MSCAN_125000 -p COM3")
    print("Legacy alias also works: --config STN1170_MSCAN_Ford")

if __name__ == '__main__':
    main()
