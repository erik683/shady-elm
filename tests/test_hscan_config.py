#!/usr/bin/env python3
"""
Test script for HS-CAN configuration.
Simulates STN1170 output for high speed 500 kbps 11-bit CAN and verifies that the
STN1170_HSCAN_500000 profile works with the bridge.
"""

import os
import queue
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock, PropertyMock, patch

# Add parent directory to path to import the bridge module.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import savvycan_bridge


CONFIG_PROFILE = "STN1170_HSCAN_500000"

# Representative 11-bit HS-CAN / OBD-II style frames.
test_frames = [
    "7E8 8 02 41 0C 1A F8 00 00 00",  # OBD response, spaced with DLC
    "7E0 2 01 0C",                    # OBD request, short frame
    "130 8 10 20 30 40 50 60 70 80",  # Generic HS-CAN body/powertrain frame
    "7DF 8 02 01 00 00 00 00 00 00",  # Functional OBD request
    "7E802410C1AF8000000",            # Packed OBD response
]


def require(condition, message):
    if not condition:
        raise AssertionError(message)
    print(f"[OK] {message}")


def load_hscan_config():
    config_path = savvycan_bridge.resolve_config_path(CONFIG_PROFILE)
    config = savvycan_bridge.load_config(config_path)
    return config_path, config


def test_hscan_frame_parsing():
    print("=" * 60)
    print("Testing HS-CAN Frame Parsing")
    print("=" * 60)

    for i, raw_frame in enumerate(test_frames, 1):
        frame_id, data, is_extended, bus = savvycan_bridge.parse_elm_frame(raw_frame, default_protocol=6)

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

        gvret_frame = savvycan_bridge.build_gvret_can_frame(frame_id, bus, data, is_extended)
        require(gvret_frame.startswith(bytes([savvycan_bridge.GVRET_COMMAND_ID, 0x00])),
                f"Frame {i} builds a GVRET binary frame")

        csv_line = savvycan_bridge.format_gvret_csv_line(
            int(time.time() * 1000), frame_id, is_extended, bus, data
        )
        require(len(csv_line.strip().split(",")) == 14, f"Frame {i} formats as GVRET CSV")

        crtd_line = savvycan_bridge.format_crtd_line(time.time(), frame_id, is_extended, bus, data)
        require(" R11 " in crtd_line, f"Frame {i} formats as CRTD R11")


def test_config_loading():
    print("\n" + "=" * 60)
    print("Testing HS-CAN Config File")
    print("=" * 60)

    config_path, config = load_hscan_config()
    print(f"\nLoaded config: {config_path}")

    require(config["can"]["protocol"] == 6, "Protocol is 6 for 500 kbps 11-bit HS-CAN")
    require(config["device"]["bus_bitrate"] == 500000, "Bus bitrate is 500000")
    require(config["can"]["standard_only"] is True, "standard_only is enabled")
    require(config["can"]["extended_only"] is False, "extended_only is disabled")
    require(config["performance"]["batch_size"] == 5, "Batch size matches HS-CAN profile")


def test_legacy_alias_resolution():
    print("\n" + "=" * 60)
    print("Testing HS-CAN Config Alias")
    print("=" * 60)

    alias_path = savvycan_bridge.resolve_config_path("STN1170_HSCAN")
    expected_suffix = os.path.join("configs", "STN1170_HSCAN_500000.json")
    require(alias_path.endswith(expected_suffix), "Legacy STN1170_HSCAN alias resolves to 500 kbps profile")


def test_config_flattening():
    print("\n" + "=" * 60)
    print("Testing HS-CAN Config Flattening")
    print("=" * 60)

    _, config = load_hscan_config()
    flat_config = savvycan_bridge.flatten_config(config)
    require(flat_config.get("protocol") == 6, "Flattened config includes protocol")
    require(flat_config.get("bus_bitrate") == 500000, "Flattened config includes bus bitrate")
    require(flat_config.get("batch_timeout") == 0.005, "Flattened config includes batch timeout")


def test_mock_serial_communication():
    print("\n" + "=" * 60)
    print("Testing HS-CAN Mock Serial Communication")
    print("=" * 60)

    mock_ser = Mock()
    mock_ser.in_waiting = 0
    serial_chunks = iter([f"{frame}\r\n".encode("ascii") for frame in test_frames[:3]])
    mock_ser.read.side_effect = lambda size: next(serial_chunks, b"")

    tcp_queue = queue.Queue()
    stop_event = threading.Event()
    serial_thread = threading.Thread(
        target=savvycan_bridge.serial_reader,
        args=(mock_ser, tcp_queue, stop_event),
        kwargs={
            "quiet": True,
            "batch_size": 3,
            "batch_timeout": 0.005,
            "extended_only": False,
            "standard_only": True,
            "output_format": "binary",
            "protocol": 6,
        },
    )

    serial_thread.start()
    time.sleep(0.1)
    stop_event.set()
    serial_thread.join(timeout=2)

    require(not serial_thread.is_alive(), "Serial reader thread stopped")
    require(tcp_queue.qsize() == 3, "Mock serial stream produced three GVRET frames")


def test_gvret_command_responses():
    print("\n" + "=" * 60)
    print("Testing GVRET Command Responses")
    print("=" * 60)

    mock_socket = Mock()
    serial_queue = queue.Queue()

    savvycan_bridge.gvret_command_handler(mock_socket, bytes([0xF1, 0x07]), serial_queue)
    require(mock_socket.sendall.called, "GVRET device info command sends a response")

    mock_socket.reset_mock()
    savvycan_bridge.gvret_command_handler(mock_socket, bytes([0xF1, 0x06]), serial_queue)
    require(mock_socket.sendall.called, "GVRET CAN bus params command sends a response")


def test_log_generation_and_validation():
    print("\n" + "=" * 60)
    print("Testing HS-CAN Log Generation")
    print("=" * 60)

    fd, log_file = tempfile.mkstemp(prefix="hscan_output_", suffix=".gvret")
    os.close(fd)
    os.remove(log_file)

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

    config_path = savvycan_bridge.resolve_config_path(args.config)
    defaults = savvycan_bridge.flatten_config(savvycan_bridge.load_config(config_path))
    for key, value in defaults.items():
        if not hasattr(args, key):
            setattr(args, key, value)

    mock_serial_instance = MagicMock()
    serial_data = [f"{frame}\r".encode("ascii") for frame in test_frames]

    def serial_data_generator():
        for item in serial_data:
            yield item
        while not savvycan_bridge.shutdown_event.is_set():
            time.sleep(0.01)
            yield b""

    data_gen = serial_data_generator()
    mock_serial_instance.read.side_effect = lambda size: next(data_gen)
    type(mock_serial_instance).in_waiting = PropertyMock(
        side_effect=lambda: 0 if savvycan_bridge.shutdown_event.is_set() else (len(serial_data[0]) if serial_data else 0)
    )
    mock_serial_instance.read_until.return_value = b'>'
    mock_serial_instance.is_open = True

    try:
        with patch("serial.Serial", return_value=mock_serial_instance):
            with patch("savvycan_bridge.is_port_available", return_value=True):
                with patch("savvycan_bridge.setup_signal_handlers", return_value=None):
                    bridge_thread = threading.Thread(target=lambda: savvycan_bridge.run_bridge(args))
                    bridge_thread.start()
                    time.sleep(1.0)
                    savvycan_bridge.shutdown_event.set()
                    bridge_thread.join(timeout=2.0)

        require(os.path.exists(log_file), "GVRET CSV log file was created")
        with open(log_file, "r", encoding="utf-8") as f:
            lines = f.readlines()

        expected_header = "Time Stamp,ID,Extended,Dir,Bus,LEN,D1,D2,D3,D4,D5,D6,D7,D8\n"
        require(lines and lines[0] == expected_header, "GVRET CSV header is correct")
        require(len(lines) - 1 == len(test_frames), "GVRET CSV log contains every test frame")
        require(all(len(line.strip().split(",")) == 14 for line in lines[1:]),
                "GVRET CSV data lines have the expected columns")
    finally:
        if os.path.exists(log_file):
            os.remove(log_file)
        savvycan_bridge.shutdown_event.clear()


def main():
    print("\n" + "=" * 60)
    print("HS-CAN Configuration Test Suite")
    print("=" * 60)

    test_hscan_frame_parsing()
    test_config_loading()
    test_legacy_alias_resolution()
    test_config_flattening()
    test_mock_serial_communication()
    test_gvret_command_responses()
    test_log_generation_and_validation()

    print("\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)
    print("[OK] HS-CAN frame parsing works")
    print("[OK] STN1170_HSCAN_500000 config loads and flattens")
    print("[OK] Legacy HS-CAN alias still resolves")
    print("[OK] Mock serial and GVRET output paths work")
    print("\nRun: python savvycan_bridge.py --config STN1170_HSCAN_500000 -p COM3")


if __name__ == "__main__":
    main()
