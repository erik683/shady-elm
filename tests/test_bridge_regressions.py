#!/usr/bin/env python3
"""Regression tests for bridge behavior that affects real adapter sessions."""

import os
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import savvycan_bridge


def test_config_profile_defaults_are_applied_to_cli_args():
    savvycan_bridge.runtime_config.clear()

    args = savvycan_bridge.parse_cli_args([
        "--config", "STN1170_MSCAN_125000",
        "-p", "COM_MOCK",
    ])

    assert args.protocol == 7
    assert args.bus_bitrate == 125000
    assert args.batch_size == 3
    assert args.standard_only is True


def test_cli_args_override_config_profile_defaults():
    savvycan_bridge.runtime_config.clear()

    args = savvycan_bridge.parse_cli_args([
        "--config", "STN1170_MSCAN_125000",
        "-p", "COM_MOCK",
        "--protocol", "6",
        "--batch-size", "9",
    ])

    assert args.protocol == 6
    assert args.batch_size == 9


def test_gvret_binary_frame_uses_length_delimited_layout():
    payload = bytes([0xAA, 0xBB, 0xCC])

    frame = savvycan_bridge.build_gvret_can_frame(0x123, 0, payload, False)

    assert frame[:2] == bytes([savvycan_bridge.GVRET_COMMAND_ID, 0x00])
    assert len(frame) == 2 + 4 + 4 + 1 + len(payload)
    assert frame[-len(payload):] == payload


def test_file_only_mode_enters_monitoring_before_reading():
    savvycan_bridge.shutdown_event.clear()
    fd, output_path = tempfile.mkstemp(prefix="file_only_", suffix=".bin")
    os.close(fd)

    args = SimpleNamespace(
        debug=False,
        log_file=None,
        quiet=True,
        port="COM_MOCK",
        baud=115200,
        timeout=1.0,
        bytesize=8,
        parity="N",
        stopbits=1,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
        disable_flow_control=True,
        force_flow_control=None,
        test_flow_control=False,
        skip_init=True,
        file_only=True,
        output_file=output_path,
        format="binary",
        batch_size=5,
        batch_timeout=0.005,
        extended_only=False,
        standard_only=True,
        protocol=6,
    )

    def idle_reader(_ser, _queue, stop_event, **_kwargs):
        while not stop_event.is_set() and not savvycan_bridge.shutdown_event.is_set():
            time.sleep(0.01)

    mock_serial = MagicMock()
    mock_serial.timeout = 1.0
    mock_serial.in_waiting = 0

    try:
        with patch("serial.Serial", return_value=mock_serial):
            with patch("savvycan_bridge.is_port_available", return_value=True):
                with patch("savvycan_bridge.setup_signal_handlers", return_value=None):
                    with patch("savvycan_bridge.start_monitoring", return_value=True) as start_mock:
                        with patch("savvycan_bridge.serial_reader", side_effect=idle_reader):
                            thread = threading.Thread(target=lambda: savvycan_bridge.run_bridge(args))
                            thread.start()
                            time.sleep(0.1)
                            savvycan_bridge.shutdown_event.set()
                            thread.join(timeout=2.0)

        assert not thread.is_alive()
        start_mock.assert_called_once_with(mock_serial, args)
    finally:
        savvycan_bridge.shutdown_event.clear()
        if os.path.exists(output_path):
            os.remove(output_path)
