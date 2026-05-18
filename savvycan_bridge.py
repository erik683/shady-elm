#!/usr/bin/env python3
"""
ELM327/STN to GVRET Bridge for SavvyCAN

This module provides a robust bridge between ELM327 and STN1170 CAN adapters
and SavvyCAN analysis software. It supports multiple CAN protocols, output formats,
and connection modes.

Features:
- Multi-protocol CAN support (HS-CAN, MS-CAN, extended frames)
- GVRET binary protocol for SavvyCAN compatibility
- Multiple output formats (binary, GVRET CSV, CRTD)
- TCP server mode for real-time analysis
- File-only logging mode
- Configurable adapter initialization
- Flow control probing and management
- CAN frame filtering and batching

Supported Adapters:
- ELM327: Standard OBD-II CAN adapters
- STN1170: Advanced CAN adapter with extended features

Key fixes vs previous version:
- Correct PySerial usage (no read_until(timeout=...)).
- Consistent GVRET-CSV header and row schema (includes Dir).
- No text headers in binary logs; proper headers only for text formats.
- Init sequence clarified; questionable ATCSM removed, better fallbacks.
- File mirroring respects stream format; no mixing ASCII headers with binary.
- Less console spam by default; env check behind --env-check.

Author: [Your Name]
Version: 1.1.0
"""

import argparse
import socket
import threading
import time
import queue
import signal
import sys
import logging
import os
import json

try:
    import serial
except ImportError:
    serial = None

# High-resolution timestamp base
TIME_BASE = time.perf_counter()

# Config loaded from JSON (optional)
runtime_config = {}

GVRET_VERSION = 1
GVRET_COMMAND_ID = 0xF1  # All GVRET frames begin with 0xF1

shutdown_event = threading.Event()

CONFIG_PROFILE_ALIASES = {
    'STN1170_HSCAN': 'STN1170_HSCAN_500000',
    'STN1170_MSCAN_Ford': 'STN1170_MSCAN_125000',
}


def signal_handler(signum, frame):
    logging.info(f"Signal {signum} received; shutting down...")
    shutdown_event.set()


def setup_signal_handlers():
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    if hasattr(signal, 'SIGHUP'):
        signal.signal(signal.SIGHUP, signal_handler)


def resolve_config_path(value: str) -> str:
    """
    Resolve a config path safely, preventing directory traversal attacks.

    Args:
        value: Config filename or path

    Returns:
        str: Safe absolute path to config file
    """
    config_dir = os.path.join(os.path.dirname(__file__), 'configs')

    if not value:
        return os.path.join(config_dir, 'STN1170_HSCAN_500000.json')

    base, ext = os.path.splitext(value)

    if ext.lower() == '.json':
        # For explicit .json files, ensure they stay within configs directory
        config_dir = os.path.join(os.path.dirname(__file__), 'configs')
        # Normalize the path and check it stays within config_dir
        full_path = os.path.abspath(os.path.join(config_dir, value))
        if not full_path.startswith(os.path.abspath(config_dir)):
            raise ValueError(f"Config path {value} is outside allowed directory")
        return full_path
    else:
        # For profile names, construct path within configs directory
        value = CONFIG_PROFILE_ALIASES.get(value, value)
        safe_path = os.path.join(config_dir, f'{value}.json')
        return os.path.abspath(safe_path)


def load_config(path: str) -> dict:
    """
    Load and validate a JSON configuration file.

    Args:
        path: Path to the JSON config file

    Returns:
        dict: Parsed configuration dictionary

    Raises:
        FileNotFoundError: If config file doesn't exist
        json.JSONDecodeError: If JSON is malformed
        ValueError: If config structure is invalid
    """
    try:
        with open(path, 'r', encoding='utf-8') as f:
            config = json.load(f)

        # Basic validation - ensure it's a dictionary
        if not isinstance(config, dict):
            raise ValueError(f"Config file {path} must contain a JSON object")

        # Log successful loading
        logging.debug(f"Loaded config from {path}")
        return config

    except FileNotFoundError:
        raise FileNotFoundError(f"Configuration file not found: {path}")
    except json.JSONDecodeError as e:
        raise ValueError(f"Invalid JSON in config file {path}: {e}")
    except Exception as e:
        raise ValueError(f"Error loading config file {path}: {e}")


def flatten_config(cfg: dict) -> dict:
    """
    Flatten a nested configuration dictionary into a single level.

    Args:
        cfg: Nested configuration dictionary

    Returns:
        dict: Flattened configuration with dot-notation keys removed

    Note:
        Only includes non-null values from nested dictionaries.
        Top-level non-dict values are ignored.
    """
    flat = {}
    for section in cfg.values():
        if isinstance(section, dict):
            flat.update({k: v for k, v in section.items() if v is not None})
    return flat


def build_gvret_can_frame(frame_id, bus, data, is_extended=False):
    """
    Build a binary GVRET CAN frame for transmission to SavvyCAN.

    The GVRET protocol uses a compact binary format for efficient transmission
    of CAN frames over TCP connections.

    Frame Format:
    [F1][00][timestamp_us:4][id:4][bus_dlc:1][data:0-8]

    Args:
        frame_id (int): CAN frame identifier (11-bit or 29-bit)
        bus (int): CAN bus number (0-15)
        data (bytes): CAN frame data payload (0-8 bytes)
        is_extended (bool): True for 29-bit extended frames

    Returns:
        bytes: Complete GVRET binary frame

    Note:
        Extended frames have bit 31 set in the ID field.
        DLC is automatically calculated from data length (clamped to 8 bytes).
    """
    ts_us = int((time.perf_counter() - TIME_BASE) * 1_000_000) & 0xFFFFFFFF
    gvret_id = (frame_id | (1 << 31)) if is_extended else frame_id

    frame = bytearray([GVRET_COMMAND_ID, 0x00])
    frame += ts_us.to_bytes(4, 'little')
    frame += gvret_id.to_bytes(4, 'little')
    dlc = min(len(data), 8)
    frame.append(((bus & 0x0F) << 4) | (dlc & 0x0F))
    frame.extend(data[:dlc])
    return bytes(frame)


def format_gvret_csv_line(ts_ms: int, can_id: int, is_ext: bool, bus: int, data: bytes) -> str:
    """
    GVRET CSV (as understood by SavvyCAN):
    Time Stamp,ID,Extended,Dir,Bus,LEN,D1,D2,D3,D4,D5,D6,D7,D8
    Dir: 0 = RX (from bus), 1 = TX
    """
    ext = 1 if is_ext else 0
    dlc = min(len(data), 8)
    # Pad to 8 columns
    payload = [f"{b:02X}" for b in data[:dlc]] + ([""] * (8 - dlc))
    cols = [str(ts_ms), f"{can_id:X}", str(ext), "0", str(bus & 0x0F), str(dlc)] + payload
    return ",".join(cols) + "\r\n"


def format_crtd_line(ts_sec: float, can_id: int, is_ext: bool, bus: int, data: bytes) -> str:
    """
    CRTD (candump-like text):
    <timestamp> R11|R29 <ID> <data bytes>
    """
    frame_type = "R29" if is_ext else "R11"
    hex_id = f"{can_id:X}"
    data_str = " ".join(f"{b:02X}" for b in data[:8])
    base = f"{ts_sec:.6f} {frame_type} {hex_id}"
    return (base + (" " + data_str if data_str else "")) + "\r\n"


def parse_elm_frame(line: str, default_protocol: int = 6):
    """
    Parse a single line of ELM327/STN adapter output into CAN frame components.

    This function handles various output formats from different CAN adapters,
    including both spaced and packed hexadecimal formats.

    Supported Input Formats:
    - Spaced: "7E8 8 02 41 0C ..." (with DLC field)
    - Packed: "7E802410C..." (no spaces)
    - RX prefix: "RX: 7E8 8 ..." (with receive indicator)
    - TX prefix: "TX: 7E8 8 ..." (with transmit indicator)

    Args:
        line (str): Raw line from adapter serial output
        default_protocol (int): CAN protocol number (6=11-bit, 33/34=29-bit)

    Returns:
        tuple: (frame_id, data_bytes, is_extended, bus) or (None, None, None, None) on error

        - frame_id (int or None): CAN frame identifier
        - data_bytes (bytes or None): Frame data payload
        - is_extended (bool or None): True for 29-bit extended frames
        - bus (int or None): CAN bus number (always 0 for single-bus adapters)

    Note:
        - Ignores status messages (NO DATA, BUS BUSY, etc.)
        - Handles protocol-based header length detection
        - Clamps data to 8 bytes maximum
        - Strips RX/TX prefixes and DLC fields as needed
    """
    s = line.strip()
    if not s:
        return (None, None, None, None)

    # Common noise/status
    if s.startswith(('>', 'OK', '?')):
        return (None, None, None, None)
    if s in ('NO DATA', 'STOPPED', 'SEARCHING', 'BUS BUSY', 'CAN ERROR', 'ERROR'):
        return (None, None, None, None)

    parts = s.split()
    if parts and parts[0] in ('RX', 'TX'):
        s = ' '.join(parts[1:])
        parts = s.split()

    # Guess header length by protocol (11 vs 29-bit)
    is_ext_proto = default_protocol in (33, 34, 8, 9)
    header_len = 8 if is_ext_proto else 3

    header = ""
    data_hex = ""

    if ' ' in s:
        # spaced form
        header = parts[0]
        # If second token looks like decimal DLC (e.g., "8" or "07"),
        # keep it only if purely decimal; hex letters mean it's part of payload.
        data_tokens = parts[1:]
        if data_tokens and data_tokens[0].isdigit() and len(data_tokens[0]) <= 2:
            # skip DLC token
            data_tokens = data_tokens[1:]
        data_hex = "".join(tok for tok in data_tokens)
    else:
        # packed hex
        if len(s) < header_len:
            return (None, None, None, None)
        header = s[:header_len]
        data_hex = s[header_len:]

    try:
        frame_id = int(header, 16)
        is_extended = len(header) > 3
        if len(data_hex) & 1:
            data_hex = data_hex[:-1]  # drop stray nibble
        if len(data_hex) > 16:
            return (None, None, None, None)
        data = bytes.fromhex(data_hex) if data_hex else b''
        return (frame_id, data, is_extended, 0)
    except Exception:
        return (None, None, None, None)


def _read_until_prompt(ser: serial.Serial, prompt=b'>', timeout_s: float = 1.0):
    """Read until prompt using a temporary serial timeout."""
    if ser is None:
        return b""
    old = ser.timeout
    try:
        ser.timeout = timeout_s
        return ser.read_until(prompt)
    finally:
        ser.timeout = old


def test_flow_control(ser: serial.Serial, mode: str) -> bool:
    """
    Probe flow control modes. Non-fatal if unknown on this adapter.
    """
    logging.info(f"Testing flow control: {mode}")
    table = {
        'none':        [b'ATCFC0\r'],
        'software':    [b'ATCFC1\r'],       # XON/XOFF
        'hardware_cts':[b'STFCSD 115200\r'],# STN-specific
        'hardware_dtr':[b'STFCSR 115200\r'],# STN-specific
    }
    cmds = table.get(mode)
    if not cmds:
        return False
    for cmd in cmds:
        ser.write(cmd)
        time.sleep(0.1)
        rsp = _read_until_prompt(ser)
        if b'?' in rsp:
            logging.debug(f"{cmd.strip().decode(errors='ignore')} -> unsupported")
            return False
    return True


def serial_reader(ser: serial.Serial,
                  out_queue: queue.Queue,
                  stop_event: threading.Event,
                  *,
                  quiet=False,
                  batch_size=5,
                  batch_timeout=0.005,
                  extended_only=False,
                  standard_only=False,
                  output_format: str = 'binary',
                  protocol: int = 6):
    if not quiet:
        logging.info("Serial reader started.")
    buf = ""
    last_flush = time.time()
    batch = []

    while not stop_event.is_set() and not shutdown_event.is_set():
        try:
            n = ser.in_waiting or 1
            chunk = ser.read(n).decode('ascii', errors='ignore')
            for ch in chunk:
                if ch in ('\r', '\n'):
                    line = buf.strip()
                    if line and not line.startswith(('>', 'OK')):
                        fid, data, is_ext, bus = parse_elm_frame(line, protocol)
                        if fid is not None:
                            if extended_only and not is_ext:
                                buf = ""
                                continue
                            if standard_only and is_ext:
                                buf = ""
                                continue

                            if output_format == 'binary':
                                batch.append(build_gvret_can_frame(fid, bus or 0, data or b'', is_ext))
                            else:
                                now = time.perf_counter() - TIME_BASE
                                if output_format == 'gvret':
                                    ts_ms = int(now * 1000)
                                    line_out = format_gvret_csv_line(ts_ms, fid, is_ext, bus or 0, data or b'')
                                elif output_format == 'crtd':
                                    ts_sec = now
                                    line_out = format_crtd_line(ts_sec, fid, is_ext, bus or 0, data or b'')
                                else:
                                    line_out = None
                                if line_out is not None:
                                    batch.append(line_out.encode('ascii'))
                            if len(batch) >= batch_size:
                                for f in batch:
                                    out_queue.put(f)
                                batch.clear()
                    buf = ""
                else:
                    buf += ch

            if batch and (time.time() - last_flush) > batch_timeout:
                for f in batch:
                    out_queue.put(f)
                batch.clear()
                last_flush = time.time()

        except (serial.SerialException, UnicodeDecodeError):
            logging.error("Serial reader error/disconnect")
            stop_event.set()
            break

    if not quiet:
        logging.info("Serial reader stopped.")


def tcp_writer(client_socket: socket.socket,
               out_queue: queue.Queue,
               stop_event: threading.Event,
               *,
               stream_format: str,
               output_file_path: str = None,
               send_lock=None):
    """
    Sends frames to TCP client and optionally mirrors to a file (with proper header).
    """
    logging.info("TCP writer started.")
    fh = None
    wrote_header = False

    if output_file_path:
        try:
            fh = open(output_file_path, 'ab')
            logging.info(f"Mirroring to file: {output_file_path}")
        except Exception as e:
            logging.error(f"File open failed: {e}")
            fh = None

    def _maybe_write_header():
        nonlocal wrote_header
        if wrote_header or fh is None:
            return
        if stream_format == 'gvret':
            header = "Time Stamp,ID,Extended,Dir,Bus,LEN,D1,D2,D3,D4,D5,D6,D7,D8\r\n"
            fh.write(header.encode('ascii'))
        elif stream_format == 'crtd':
            hdr = "// CAN Log File\n// Format: CRTD\n// Generated: " + time.strftime("%Y-%m-%d %H:%M:%S") + "\n"
            fh.write(hdr.encode('ascii'))
        # binary: no header
        wrote_header = True
        fh.flush()

    while not stop_event.is_set() and not shutdown_event.is_set():
        try:
            frame = out_queue.get(timeout=1)
            if send_lock is not None:
                with send_lock:
                    client_socket.sendall(frame)
            else:
                client_socket.sendall(frame)
            if fh is not None:
                _maybe_write_header()
                fh.write(frame)
        except queue.Empty:
            continue
        except (socket.timeout, BrokenPipeError, ConnectionResetError) as e:
            logging.warning(f"TCP writer socket issue: {e}")
            stop_event.set()
            break
        except socket.error as e:
            logging.error(f"TCP writer error: {e}")
            stop_event.set()
            break

    if fh is not None:
        try:
            fh.flush()
            fh.close()
        except Exception:
            pass
    logging.info("TCP writer stopped.")


def serial_writer(ser: serial.Serial,
                  serial_queue: queue.Queue,
                  stop_event: threading.Event,
                  *,
                  serial_delay: float = 0.005):
    logging.info("Serial writer started.")
    while not stop_event.is_set() and not shutdown_event.is_set():
        try:
            cmd_list = serial_queue.get(timeout=1)
            for cmd in cmd_list:
                ser.write(cmd)
                time.sleep(serial_delay)
                _ = _read_until_prompt(ser, b'>', 1.0)
        except queue.Empty:
            continue
        except serial.SerialException as e:
            logging.error(f"Serial writer error: {e}")
            stop_event.set()
            break
    logging.info("Serial writer stopped.")


def _locked_sendall(sock: socket.socket, payload: bytes, send_lock=None):
    try:
        if send_lock is not None:
            with send_lock:
                sock.sendall(payload)
        else:
            sock.sendall(payload)
    except socket.error:
        pass


def gvret_command_handler(client_socket: socket.socket, data: bytes, serial_queue: queue.Queue, send_lock=None):
    """
    Handle GVRET protocol commands from SavvyCAN.
    0x00: TX frame request from client
    0x01: Time sync
    0x06: CAN bus params
    0x07: Device info
    0x09: Comm validation
    0x0C: Num buses
    0x0D: Extended buses (none here)
    """
    if not data or data[0] != GVRET_COMMAND_ID:
        return

    cmd = data[1]
    logging.debug(f"GVRET cmd 0x{cmd:02X}")

    if cmd == 0x00:
        # Expect: F1 00 [ID4][BUS][LEN][DATA..LEN][opt 00]
        if len(data) < 8:
            return
        frame_id = int.from_bytes(data[2:6], 'little')
        dlc = data[7] & 0x0F
        payload = data[8:8+dlc]

        is_extended = bool(frame_id & (1 << 31))
        if is_extended:
            frame_id &= ~(1 << 31)
        hdr = f"{frame_id:08X}" if is_extended else f"{frame_id:03X}"
        # Bracket the TX with monitor stop/start so the adapter is willing
        # to transmit, then returns to passive capture.
        at_cmds = [
            b'STMA 0\r',
            f"ATSH{hdr}\r".encode('ascii'),
            f"{payload.hex()}\r".encode('ascii'),
            b'STMA\r',
        ]
        serial_queue.put(at_cmds)
        return

    if cmd == 0x01:
        ts_us = int(time.time() * 1_000_000) & 0xFFFFFFFF
        _locked_sendall(client_socket, bytes([GVRET_COMMAND_ID, 0x01]) + ts_us.to_bytes(4, 'little'), send_lock)
        return

    if cmd == 0x06:
        bitrate = int(runtime_config.get('device', {}).get('bus_bitrate', 500000))
        bb = bitrate.to_bytes(4, 'little')
        _locked_sendall(client_socket,
                        bytes([GVRET_COMMAND_ID, 0x06, 0x01, bb[0], bb[1], bb[2], bb[3],
                               0x00, 0x00, 0x00, 0x00, 0x00]),
                        send_lock)
        return

    if cmd == 0x07:
        build_num = 1234
        _locked_sendall(client_socket,
                        bytes([GVRET_COMMAND_ID, 0x07, build_num & 0xFF, (build_num >> 8) & 0xFF, 1, 1, 0]),
                        send_lock)
        return

    if cmd == 0x09:
        _locked_sendall(client_socket, bytes([GVRET_COMMAND_ID, 0x09]), send_lock)
        return

    if cmd == 0x0C:
        _locked_sendall(client_socket, bytes([GVRET_COMMAND_ID, 0x0C, 1]), send_lock)
        return

    if cmd == 0x0D:
        _locked_sendall(client_socket, bytes([GVRET_COMMAND_ID, 0x0D] + [0x00]*15), send_lock)
        return


def process_gvret_buffer(buf: bytearray, client_socket: socket.socket, serial_queue: queue.Queue, send_lock=None) -> bytearray:
    while len(buf) > 1:
        if buf[0] != GVRET_COMMAND_ID:
            buf = buf[1:]
            continue
        cmd = buf[1]
        frame_len = 0

        if cmd in (0x01, 0x06, 0x07, 0x09, 0x0C, 0x0D):
            frame_len = 2
        elif cmd == 0x00:
            if len(buf) < 8:
                break
            dlc = buf[7] & 0x0F
            need = 8 + dlc
            if len(buf) < need:
                break
            frame_len = need
            if len(buf) >= need + 1 and buf[need] == 0x00:
                frame_len += 1
        else:
            # Unknown; drop leading byte to resync
            buf = buf[1:]
            continue

        if len(buf) >= frame_len:
            pkt = bytes(buf[:frame_len])
            gvret_command_handler(client_socket, pkt, serial_queue, send_lock)
            buf = buf[frame_len:]
        else:
            break
    return buf


def tcp_reader(client_socket: socket.socket, serial_queue: queue.Queue, stop_event: threading.Event, send_lock=None):
    logging.info("TCP reader started.")
    buf = bytearray()
    while not stop_event.is_set() and not shutdown_event.is_set():
        try:
            data = client_socket.recv(1024)
            if not data:
                logging.info("TCP client disconnected.")
                stop_event.set()
                break
            buf.extend(data)
            buf = process_gvret_buffer(buf, client_socket, serial_queue, send_lock)
        except socket.timeout:
            continue
        except socket.error as e:
            logging.error(f"TCP reader error: {e}")
            stop_event.set()
            break
    logging.info("TCP reader stopped.")


def is_port_available(port: str, baud: int) -> bool:
    if serial is None:
        return False
    try:
        test = serial.Serial(port=port, baudrate=baud, timeout=0.1)
        test.close()
        return True
    except (serial.SerialException, OSError):
        return False


def clear_elm_buffers(ser: serial.Serial, timeout: float = 0.5):
    try:
        logging.info("Clearing adapter buffers...")
        # Stop monitoring if on
        try:
            ser.write(b'STMA 0\r')
            time.sleep(0.1)
        except Exception:
            pass

        old = ser.timeout
        ser.timeout = timeout
        while True:
            n = ser.in_waiting
            if n <= 0:
                break
            ser.read(n)
        ser.timeout = old

        # harmless info probe to flush
        ser.write(b'ATI\r')
        _ = _read_until_prompt(ser, b'>', timeout)
        logging.info("Buffers cleared.")
        return True
    except Exception as e:
        logging.warning(f"Buffer clear failed: {e}")
        return False


def initialize_elm_device(ser: serial.Serial, args) -> bool:
    try:
        clear_elm_buffers(ser, timeout=0.2)
        # Base init
        init_cmds = [
            b'ATZ\r',
            b'ATE0\r',
            b'ATL0\r',
            b'ATS0\r',
            b'ATH1\r',
            b'ATCAF0\r',
            b'ATCFC0\r',
            f'STP {args.protocol}\r'.encode('ascii'),
            b'STPTO 20\r',
        ]

        # Filters
        if args.filter:
            init_cmds += [b'STFPC\r', f'STFPA {args.filter}\r'.encode('ascii')]

        # STN monitor configuration if supported
        init_cmds += [f'STCMM {args.monitor_mode}\r'.encode('ascii')]

        # Custom
        if args.custom_init:
            init_cmds += [f"{c}\r".encode('ascii') for c in args.custom_init]

        logging.info("Initializing adapter...")
        for cmd in init_cmds:
            ser.write(cmd)
            time.sleep(args.init_delay)
            rsp = _read_until_prompt(ser, b'>', args.response_timeout).decode('ascii', errors='ignore')
            logging.debug(f"{cmd.strip().decode(errors='ignore')} -> {rsp.strip()}")
            if '?' in rsp:
                # graceful fallbacks
                if cmd.startswith(b'STCMM'):
                    logging.warning("STCMM unsupported; will try ATMA later to monitor.")
                elif cmd.startswith(b'STFP'):
                    logging.warning("STN filter command unsupported; continuing without filters.")
                # otherwise keep going

        logging.info("Adapter init complete.")
        return True
    except Exception as e:
        logging.error(f"Adapter init failed: {e}")
        return False


def start_monitoring(ser: serial.Serial, args) -> bool:
    """
    Enter monitor mode: prefer STMA (STN), fallback to ATMA (ELM).
    Consume trailing prompt.
    """
    try:
        ser.write(b'STMA\r')
        logging.info("Entered monitor mode (STMA).")
        _ = _read_until_prompt(ser, b'>', 1.0)
        return True
    except Exception:
        pass
    try:
        ser.write(b'ATMA\r')
        logging.info("Entered monitor mode (ATMA).")
        _ = _read_until_prompt(ser, b'>', 1.0)
        return True
    except Exception as e:
        logging.error(f"Failed to start monitoring: {e}")
        return False


def stop_monitoring(ser: serial.Serial):
    try:
        ser.write(b'STMA 0\r')
        time.sleep(0.1)
    except Exception:
        try:
            ser.write(b'ATPC\r')
            time.sleep(0.1)
        except Exception:
            pass


def run_bridge(args):
    """
    Main bridge execution function that orchestrates the entire CAN bridge operation.

    This function sets up logging, initializes the serial connection, configures the
    CAN adapter, and manages the TCP server or file logging operation. It handles
    both TCP server mode (for SavvyCAN) and file-only logging mode.

    Args:
        args: Parsed command-line arguments from argparse

    Key Operations:
    1. Configure logging based on debug/quiet flags
    2. Validate serial port availability
    3. Initialize serial connection with flow control probing
    4. Configure CAN adapter with protocol-specific initialization
    5. Start monitoring mode (STMA preferred, ATMA fallback)
    6. Run TCP server or file logging loop
    7. Handle graceful shutdown and cleanup

    Note:
        This function runs indefinitely until interrupted or client disconnects.
        Automatic restart capability available via main_entry() wrapper.
    """
    log_level = logging.DEBUG if args.debug else logging.INFO
    log_fmt = '%(asctime)s %(levelname)s %(message)s'
    if args.log_file:
        logging.basicConfig(level=log_level, filename=args.log_file, format=log_fmt)
    else:
        logging.basicConfig(level=log_level, format=log_fmt)

    if args.quiet:
        logging.getLogger().setLevel(logging.WARNING)

    setup_signal_handlers()
    logging.info("Press Ctrl+C to stop.")

    if serial is None:
        logging.error("pyserial is required. Install with: pip install pyserial")
        return

    parity_map = {'N': serial.PARITY_NONE, 'E': serial.PARITY_EVEN, 'O': serial.PARITY_ODD, 'M': serial.PARITY_MARK, 'S': serial.PARITY_SPACE}
    bytesize_map = {5: serial.FIVEBITS, 6: serial.SIXBITS, 7: serial.SEVENBITS, 8: serial.EIGHTBITS}
    stopbits_map = {1: serial.STOPBITS_ONE, 2: serial.STOPBITS_TWO}

    if not is_port_available(args.port, args.baud):
        logging.error(f"Serial port {args.port} not available. Close other apps, unplug/replug, or use --list-ports.")
        return

    logging.info(f"Opening {args.port} @ {args.baud} ...")
    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            timeout=args.timeout,
            bytesize=bytesize_map[args.bytesize],
            parity=parity_map[args.parity],
            stopbits=stopbits_map[args.stopbits],
            xonxoff=args.xonxoff,
            rtscts=args.rtscts,
            dsrdtr=args.dsrdtr,
        )
    except serial.SerialException as e:
        logging.error(f"Serial open failed: {e}")
        return

    # Flow control probing (optional)
    if ser and not args.disable_flow_control:
        chosen = None
        if args.force_flow_control:
            chosen = args.force_flow_control if test_flow_control(ser, args.force_flow_control) else None
        elif args.test_flow_control:
            for m in ('none', 'hardware_cts', 'hardware_dtr', 'software'):
                if test_flow_control(ser, m):
                    chosen = m
                    break
        if chosen:
            logging.info(f"Flow control active: {chosen}")
        else:
            logging.info("Flow control left at default (ATCFC0).")

    if ser and not args.skip_init:
        if not initialize_elm_device(ser, args):
            logging.error("Init failed. Exiting.")
            try:
                ser.close()
            except Exception:
                pass
            return

    # File-only logging (no TCP)
    if args.file_only:
        if not args.output_file:
            logging.error("--file-only requires --output-file")
            return

        logging.info(f"File-only logging to {args.output_file} in {args.format} format.")
        if not start_monitoring(ser, args):
            logging.error("Could not enter monitor mode for file-only logging.")
            try:
                ser.close()
            except Exception:
                pass
            return

        stop_event = threading.Event()
        q = queue.Queue()

        t_reader = threading.Thread(
            target=serial_reader,
            args=(ser, q, stop_event),
            kwargs=dict(
                quiet=args.quiet,
                batch_size=args.batch_size,
                batch_timeout=args.batch_timeout,
                extended_only=args.extended_only,
                standard_only=args.standard_only,
                output_format=args.format,
                protocol=args.protocol
            )
        )
        t_reader.start()

        # Open file and write header (if needed), then pump queue
        try:
            with open(args.output_file, 'ab') as fh:
                # headers for text formats
                if args.format == 'gvret':
                    fh.write(b"Time Stamp,ID,Extended,Dir,Bus,LEN,D1,D2,D3,D4,D5,D6,D7,D8\r\n")
                elif args.format == 'crtd':
                    hdr = "// CAN Log File\n// Format: CRTD\n// Generated: " + time.strftime("%Y-%m-%d %H:%M:%S") + "\n"
                    fh.write(hdr.encode('ascii'))
                while not stop_event.is_set() and not shutdown_event.is_set():
                    try:
                        line = q.get(timeout=1)
                        fh.write(line)
                    except queue.Empty:
                        continue
        except KeyboardInterrupt:
            pass
        finally:
            logging.info("Stopping file logging...")
            stop_event.set()
            stop_monitoring(ser)
            clear_elm_buffers(ser, timeout=0.2)
            try:
                ser.close()
            except Exception:
                pass
            t_reader.join(timeout=2.0)
            logging.info("File logging stopped.")
        return

    # TCP server mode
    logging.info("Starting TCP server...")
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1 if args.reuse_addr else 0)
    server.bind((args.host, args.tcp_port))
    server.listen(args.backlog)
    logging.info(f"Listening on {args.host}:{args.tcp_port}")

    try:
        while not shutdown_event.is_set():
            try:
                server.settimeout(1.0)
                client, addr = server.accept()
            except socket.timeout:
                continue
            except OSError as e:
                if shutdown_event.is_set():
                    break
                logging.error(f"Accept error: {e}")
                break

            logging.info(f"Client {addr} connected.")
            clear_elm_buffers(ser, timeout=0.3)
            if not start_monitoring(ser, args):
                client.close()
                continue

            client.settimeout(args.tcp_timeout)
            client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

            # Peek for GVRET binary handshake (0xE7 0xE7)
            try:
                initial = client.recv(1024)
            except socket.timeout:
                initial = b""

            binary_mode = (b'\xE7\xE7' in initial)
            stop_event = threading.Event()
            serial_to_tcp_q = queue.Queue()

            if not binary_mode and args.format in ('gvret', 'crtd'):
                logging.info(f"Text streaming mode: {args.format}")
                t_reader = threading.Thread(
                    target=serial_reader,
                    args=(ser, serial_to_tcp_q, stop_event),
                    kwargs=dict(
                        quiet=args.quiet,
                        batch_size=args.batch_size,
                        batch_timeout=args.batch_timeout,
                        extended_only=args.extended_only,
                        standard_only=args.standard_only,
                        output_format=args.format,
                        protocol=args.protocol
                    )
                )
                t_writer = threading.Thread(
                    target=tcp_writer,
                    args=(client, serial_to_tcp_q, stop_event),
                    kwargs=dict(stream_format=args.format, output_file_path=args.output_file)
                )
                t_reader.start()
                t_writer.start()

                while not stop_event.is_set() and not shutdown_event.is_set():
                    time.sleep(0.5)

                logging.info("Tearing down client (text mode)...")
                stop_event.set()
                stop_monitoring(ser)
                clear_elm_buffers(ser, timeout=0.2)
                try:
                    client.close()
                except Exception:
                    pass
                t_reader.join(timeout=2.0)
                t_writer.join(timeout=2.0)

            else:
                # Binary GVRET
                if not binary_mode:
                    logging.warning("No E7E7 handshake; closing.")
                    client.close()
                    continue

                logging.info("Binary GVRET mode.")
                send_lock = threading.Lock()
                # Send device info + bus params + bus count
                try:
                    with send_lock:
                        client.sendall(bytes([GVRET_COMMAND_ID, 0x07, 0xD2, 0x04, 1, 1, 0]))
                        bitrate = int(runtime_config.get('device', {}).get('bus_bitrate', 500000))
                        bb = bitrate.to_bytes(4, 'little')
                        client.sendall(bytes([GVRET_COMMAND_ID, 0x06, 0x01, bb[0], bb[1], bb[2], bb[3],
                                              0x00, 0x00, 0x00, 0x00, 0x00]))
                        client.sendall(bytes([GVRET_COMMAND_ID, 0x0C, 1]))
                except socket.error as e:
                    logging.error(f"Handshake send failed: {e}")
                    client.close()
                    continue

                tcp_to_serial_q = queue.Queue()
                t_reader = threading.Thread(
                    target=serial_reader,
                    args=(ser, serial_to_tcp_q, stop_event),
                    kwargs=dict(
                        quiet=args.quiet,
                        batch_size=args.batch_size,
                        batch_timeout=args.batch_timeout,
                        extended_only=args.extended_only,
                        standard_only=args.standard_only,
                        output_format='binary',
                        protocol=args.protocol
                    )
                )
                t_writer = threading.Thread(
                    target=tcp_writer,
                    args=(client, serial_to_tcp_q, stop_event),
                    kwargs=dict(stream_format='binary', output_file_path=args.output_file, send_lock=send_lock)
                )
                t_cmdin = threading.Thread(target=tcp_reader, args=(client, tcp_to_serial_q, stop_event, send_lock))
                t_cmdout = threading.Thread(
                    target=serial_writer,
                    args=(ser, tcp_to_serial_q, stop_event),
                    kwargs=dict(serial_delay=args.serial_delay)
                )
                for t in (t_reader, t_writer, t_cmdin, t_cmdout):
                    t.start()

                # Process any residual after stripping E7E7
                rem = initial.replace(b'\xE7\xE7', b'')
                if rem:
                    process_gvret_buffer(bytearray(rem), client, tcp_to_serial_q, send_lock)

                while not stop_event.is_set() and not shutdown_event.is_set():
                    time.sleep(0.5)

                logging.info("Tearing down client (binary mode)...")
                stop_event.set()
                stop_monitoring(ser)
                clear_elm_buffers(ser, timeout=0.2)
                try:
                    client.close()
                except Exception:
                    pass
                for t in (t_reader, t_writer, t_cmdin, t_cmdout):
                    t.join(timeout=2.0)

            # Reset adapter between sessions
            if not shutdown_event.is_set():
                logging.info("Resetting serial adapter for next session...")
                try:
                    ser.close()
                except Exception:
                    pass
                try:
                    ser = serial.Serial(
                        port=args.port,
                        baudrate=args.baud,
                        timeout=args.timeout,
                        bytesize=bytesize_map[args.bytesize],
                        parity=parity_map[args.parity],
                        stopbits=stopbits_map[args.stopbits],
                        xonxoff=args.xonxoff,
                        rtscts=args.rtscts,
                        dsrdtr=args.dsrdtr,
                    )
                    if not args.skip_init and not initialize_elm_device(ser, args):
                        logging.error("Re-init failed; stopping server loop.")
                        break
                except Exception as e:
                    logging.error(f"Serial reopen failed: {e}")
                    break

            if args.exit_on_disconnect or shutdown_event.is_set():
                break

    except KeyboardInterrupt:
        logging.info("Ctrl+C, exiting.")
    finally:
        logging.info("Server shutdown...")
        shutdown_event.set()
        try:
            stop_monitoring(ser)
            clear_elm_buffers(ser, timeout=0.2)
        except Exception:
            pass
        try:
            server.close()
        except Exception:
            pass
        try:
            ser.close()
        except Exception:
            pass
        logging.info("Cleanup complete.")


def build_arg_parser(config: dict = None) -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="ELM327/STN to GVRET TCP Bridge for SavvyCAN",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s -p COM3
  %(prog)s -p COM3 -b 115200 --quiet
  %(prog)s -p COM3 --filter 7E0,7FF --test-flow-control
  %(prog)s -p COM3 --host 0.0.0.0 --tcp-port 29000 --format binary
  %(prog)s --config STN1170_HSCAN_500000 -p COM3
"""
    )

    p.add_argument('--config', help="Config file path or profile name (e.g., STN1170_HSCAN_500000)")
    p.add_argument('-p', '--port', help="Serial port (e.g., COM3, /dev/ttyUSB0)")

    sg = p.add_argument_group('Serial')
    sg.add_argument('-b', '--baud', type=int, default=115200,
                    choices=[9600, 19200, 38400, 57600, 115200, 230400, 460800, 500000, 1000000],
                    help="Serial baud (default 115200)")
    sg.add_argument('--timeout', type=float, default=1.0, help="Serial read timeout (s)")
    sg.add_argument('--bytesize', type=int, default=8, choices=[5, 6, 7, 8], help="Data bits")
    sg.add_argument('--parity', choices=['N', 'E', 'O', 'M', 'S'], default='N', help="Parity")
    sg.add_argument('--stopbits', type=int, choices=[1, 2], default=1, help="Stop bits")
    sg.add_argument('--xonxoff', action='store_true', help="XON/XOFF")
    sg.add_argument('--rtscts', action='store_true', help="RTS/CTS")
    sg.add_argument('--dsrdtr', action='store_true', help="DSR/DTR")

    cg = p.add_argument_group('CAN')
    cg.add_argument('--protocol', type=int, default=33, choices=[6, 7, 33, 34],
                    help="6=HS-CAN 11-bit, 7=MS-CAN 11-bit, 33=29-bit, 34=both (default 33)")
    cg.add_argument('--monitor-mode', type=int, choices=[0, 1, 2], default=1,
                    help="STN monitor mode: 0=off, 1=normal, 2=extended")
    cg.add_argument('--filter', help="STN filter range (e.g., 7E0,7FF or 7E0-7FF)")
    cg.add_argument('--extended-only', action='store_true', help="Only capture extended frames")
    cg.add_argument('--standard-only', action='store_true', help="Only capture standard frames")

    ig = p.add_argument_group('Adapter Init')
    ig.add_argument('--skip-init', action='store_true', help="Skip adapter initialization")
    ig.add_argument('--init-delay', type=float, default=0.1, help="Delay after each init command (s)")
    ig.add_argument('--response-timeout', type=float, default=1.0, help="Per-command response timeout (s)")
    ig.add_argument('--custom-init', action='append', help="Extra init command (repeatable)")

    tg = p.add_argument_group('TCP Server')
    tg.add_argument('--host', default='127.0.0.1', help="Bind host (default 127.0.0.1)")
    tg.add_argument('--tcp-port', type=int, default=23, help="Listen port (default 23)")
    tg.add_argument('--backlog', type=int, default=1, help="Backlog (default 1)")
    tg.add_argument('--reuse-addr', action='store_true', default=True, help="SO_REUSEADDR")

    pg = p.add_argument_group('Performance')
    pg.add_argument('--batch-size', type=int, default=5, help="Batch size (default 5)")
    pg.add_argument('--batch-timeout', type=float, default=0.005, help="Batch flush timeout (s)")
    pg.add_argument('--serial-delay', type=float, default=0.005, help="Delay between serial commands (s)")
    pg.add_argument('--tcp-timeout', type=float, default=1.0, help="TCP socket timeout (s)")

    fg = p.add_argument_group('Flow Control')
    fg.add_argument('--test-flow-control', action='store_true', help="Probe flow control modes")
    fg.add_argument('--force-flow-control', choices=['none', 'hardware_cts', 'hardware_dtr', 'software'],
                    help="Force a flow control mode")
    fg.add_argument('--disable-flow-control', action='store_true', help="Disable flow control")

    lg = p.add_argument_group('Logging / Behavior')
    lg.add_argument('--quiet', action='store_true', help="Reduce console chatter")
    lg.add_argument('--debug', action='store_true', default=False, help="Enable debug logging")
    lg.add_argument('--log-file', help="Log to file")
    lg.add_argument('--timestamp-format', choices=['none', 'iso', 'unix', 'elapsed'], default='elapsed',
                    help="Log timestamp style (for logger messages)")
    lg.add_argument('--exit-on-disconnect', action='store_true', help="Exit when client disconnects")
    lg.add_argument('--restart-on-error', action='store_true', help="Auto-restart on crash")
    lg.add_argument('--max-restarts', type=int, default=5, help="Max restarts (default 5)")
    lg.add_argument('--graceful-shutdown', action='store_true', default=True, help="Handle signals gracefully")
    lg.add_argument('--env-check', action='store_true', help="Print environment info and exit")

    og = p.add_argument_group('Output')
    og.add_argument('--format', choices=['gvret', 'crtd', 'binary'], default='binary',
                    help="Stream format over TCP (default binary)")
    og.add_argument('--output-file', help="Optional mirror file (append mode)")
    og.add_argument('--file-only', action='store_true', help="Log to file only (still requires --port)")

    p.add_argument('--list-ports', action='store_true', help="List available serial ports and exit")
    p.add_argument('--version', action='version', version='savvycan_bridge 1.1.0')

    if config:
        p.set_defaults(**flatten_config(config))

    return p


def parse_cli_args(argv=None):
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument('--config', help="Config file path or profile name (e.g., STN1170_HSCAN_500000)")
    pre_args, remaining = pre.parse_known_args(argv)

    config_path = resolve_config_path(pre_args.config)
    config = {}
    runtime_config.clear()
    if os.path.exists(config_path):
        config = load_config(config_path)
        runtime_config.update(config)
        logging.info(f"Loaded config: {config_path}")
    elif pre_args.config:
        logging.warning(f"Config not found: {config_path}")

    parser = build_arg_parser(config)
    args = parser.parse_args(remaining)
    args.config = pre_args.config
    args.config_path = config_path

    if not args.port and not (args.env_check or args.list_ports):
        parser.error("the following arguments are required: -p/--port")

    if args.file_only and not args.output_file:
        parser.error("--file-only requires --output-file")

    return args


def main_entry(args):
    restart_count = 0
    max_restarts = args.max_restarts if args.restart_on_error else 0
    while restart_count <= max_restarts and not shutdown_event.is_set():
        try:
            run_bridge(args)
            break
        except Exception as e:
            restart_count += 1
            if restart_count <= max_restarts:
                logging.error(f"Bridge crashed: {e}. Restarting ({restart_count}/{max_restarts})...")
                time.sleep(2)
            else:
                logging.error(f"Bridge crashed: {e}. Max restarts exceeded.")
                raise


if __name__ == "__main__":
    args = parse_cli_args()

    if args.env_check:
        print("--- Python Environment Check ---")
        print(f"Python Executable: {sys.executable}")
        print(f"Python Version: {sys.version}")
        if serial:
            print(f"pyserial Version: {getattr(serial, 'VERSION', 'unknown')}")
        else:
            print("pyserial is not installed.")
        print("--------------------------------")
        sys.exit(0)

    if args.list_ports:
        try:
            import serial.tools.list_ports
            ports = serial.tools.list_ports.comports()
            if ports:
                print("Available serial ports:")
                for port in ports:
                    print(f"  {port.device} - {port.description}")
            else:
                print("No serial ports found.")
        except Exception:
            print("Cannot list ports (pyserial not installed?).")
        sys.exit(0)

    main_entry(args)
