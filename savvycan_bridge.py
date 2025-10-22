
0
import argparse
import socket
import threading
import time
import queue
import signal
import sys
import logging
import os

# Guarded pyserial import to allow --list-ports without pyserial
try:
    import serial
except ImportError:
    serial = None

# Time base for accurate timestamps
TIME_BASE = time.perf_counter()

# GVRET protocol constants
GVRET_VERSION = 1
GVRET_COMMAND_ID = 0xF1

# Global shutdown event
shutdown_event = threading.Event()

def signal_handler(signum, frame):
    """Handle shutdown signals gracefully."""
    print(f"\nReceived signal {signum}. Initiating graceful shutdown...")
    shutdown_event.set()

def setup_signal_handlers():
    """Setup signal handlers for graceful shutdown."""
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    if hasattr(signal, 'SIGHUP'):
        signal.signal(signal.SIGHUP, signal_handler)

def build_gvret_can_frame(frame_id, bus, data, is_extended=False):
    """Builds a GVRET CAN frame using high-resolution timestamps."""
    timestamp = int((time.perf_counter() - TIME_BASE) * 1000000) & 0xFFFFFFFF
    
    gvret_id = frame_id
    if is_extended:
        gvret_id |= (1 << 31)

    frame = bytearray([GVRET_COMMAND_ID, 0x00])
    frame += timestamp.to_bytes(4, 'little')
    frame += gvret_id.to_bytes(4, 'little')
    
    dlc = len(data)
    frame.append((bus << 4) | dlc)
    
    frame.extend(data)
    
    return bytes(frame)

def format_gvret_line(ts_ms: int, can_id: int, is_ext: bool, bus: int, data: bytes) -> str:
    """Formats a CAN frame as GVRET CSV text per can_log_formats.md."""
    hex_id = f"{can_id:X}"
    ext = 1 if is_ext else 0
    dlc = len(data)
    data_str = ",".join(f"{b:02X}" for b in data[:dlc])
    base = f"{ts_ms},{hex_id},{ext},{bus},{dlc}"
    return (base + ("," + data_str if dlc else "")) + "\r\n"

def format_crtd_line(ts_sec: float, can_id: int, is_ext: bool, bus: int, data: bytes) -> str:
    """Formats a CAN frame as CRTD text per can_log_formats.md (no bus prefix)."""
    frame_type = "R29" if is_ext else "R11"
    hex_id = f"{can_id:X}"
    data_str = " ".join(f"{b:02X}" for b in data)
    base = f"{ts_sec:.6f} {frame_type} {hex_id}"
    return (base + (" " + data_str if data else "")) + "\r\n"

def parse_elm_frame(line: str):
    """Parses a line from ELM327/STN monitor mode into ID, data, and extended flag."""
    line = line.strip()
    # Ignore common status/error lines
    bad_prefixes = ('>', 'OK', '?')
    bad_lines = ('NO DATA', 'STOPPED', 'SEARCHING', 'BUS BUSY', 'CAN ERROR', 'ERROR')
    if not line or line.startswith(bad_prefixes) or any(line.startswith(x) for x in bad_lines):
        return None, None, None, None
    
    # Strip leading RX/TX tokens if present
    parts = line.split()
    if parts and parts[0] in ('RX', 'TX'):
        line = ' '.join(parts[1:])
    
    header = ""
    data_str = ""

    # Handle both spaced and non-spaced formats
    if ' ' in line:
        # Spaced format: "7E8 8 02 41 0C 00 00 00 00 00" or "7E8 02 41 0C..."
        parts = line.split()
        header = parts[0]
        # Check if second part is DLC or start of data
        if len(parts) > 1 and len(parts[1]) <= 2 and parts[1].isdigit():
            data_parts = parts[2:]
        else:
            data_parts = parts[1:]
        data_str = "".join(data_parts)
    else:
        # Non-spaced format: "201800007D002710FFFF" or "123ABCDEF"
        # For 11-bit CAN, IDs are typically 3 hex chars, but some adapters
        # may output them as 8 chars with leading zeros. Since we're configured
        # for 11-bit CAN, always treat as 11-bit regardless of format.
        if len(line) >= 8:
            # 8-char format: take first 3 chars for 11-bit ID, rest is data
            header = line[:3]
            data_str = line[3:]
        elif len(line) >= 3:
            header = line[:3]
            data_str = line[3:]
        else:
            return None, None, None, None

    # Since we're configured for 11-bit CAN, always treat as standard (not extended)
    is_extended = False
    
    try:
        frame_id = int(header, 16)
        
        # Ensure even number of hex chars for data
        if len(data_str) % 2 != 0:
            data_str = data_str[:-1]
        
        data = bytes.fromhex(data_str) if data_str else b''
        
        return frame_id, data, is_extended, 0 # Assume bus 0

    except (ValueError, IndexError):
        return None, None, None, None

def test_flow_control(ser: serial.Serial, flow_control_type: str):
    """Test different flow control modes and return the best one."""
    logging.info(f"Testing {flow_control_type} flow control...")
    
    test_commands = {
        'none': [b'ATCFC0\r'],
        'hardware_cts': [b'STFCSD 115200\r'],
        'hardware_dtr': [b'STFCSR 115200\r'],
        'software': [b'ATCFC1\r']
    }
    
    if flow_control_type not in test_commands:
        return False
    
    for cmd in test_commands[flow_control_type]:
        ser.write(cmd)
        time.sleep(0.1)
        response = ser.read_until(b'>').decode('ascii', errors='ignore')
        if '?' in response:
            logging.debug(f"  {cmd.decode().strip()} failed: {response.strip()}")
            return False
        else:
            logging.debug(f"  {cmd.decode().strip()} OK: {response.strip()}")
    
    return True

def serial_reader(ser: serial.Serial, tcp_queue: queue.Queue, stop_event: threading.Event, quiet=False, batch_size=5, batch_timeout=0.005, extended_only=False, standard_only=False, output_format: str = 'binary'):
    """Optimized serial reader with batching."""
    if not quiet:
        logging.info("Serial reader thread started.")
    
    buffer = ""
    last_line_ts = time.time()
    frame_batch = []
    
    while not stop_event.is_set() and not shutdown_event.is_set():
        try:
            # Read multiple bytes at once for efficiency
            available = ser.in_waiting or 1
            chunk = ser.read(available).decode('ascii', errors='ignore')
            
            for char in chunk:
                if char == '\r' or char == '\n':
                    if buffer.strip() and not buffer.startswith('>') and not buffer.startswith('OK'):
                        if not quiet:
                            logging.debug(f"ELM line: {buffer.strip()}")
                        frame_id, data, is_extended, bus = parse_elm_frame(buffer)
                        if frame_id is not None:
                            if not quiet:
                                logging.debug(f"Parsed frame: ID={frame_id:X}, ext={is_extended}, bus={bus}, data={data.hex() if data else 'empty'}")
                            # Apply extended/standard-only filtering
                            if extended_only and not is_extended:
                                continue
                            if standard_only and is_extended:
                                continue
                            
                            # Clamp to 8 bytes per classic CAN
                            if data and len(data) > 8:
                                data = data[:8]

                            if output_format == 'binary':
                                gvret_frame = build_gvret_can_frame(frame_id, bus, data, is_extended)
                                frame_batch.append(gvret_frame)
                            else:
                                now = time.perf_counter() - TIME_BASE
                                if output_format == 'gvret':
                                    ts_ms = int(now * 1000)
                                    line = format_gvret_line(ts_ms, frame_id, is_extended, bus or 0, data)
                                elif output_format == 'crtd':
                                    ts_sec = now
                                    line = format_crtd_line(ts_sec, frame_id, is_extended, bus or 0, data)
                                else:
                                    # Fallback to binary if unknown format
                                    gvret_frame = build_gvret_can_frame(frame_id, bus, data, is_extended)
                                    frame_batch.append(gvret_frame)
                                    line = None
                                if line is not None:
                                    frame_batch.append(line.encode('ascii'))
                            
                            if len(frame_batch) >= batch_size:
                                for frame in frame_batch:
                                    tcp_queue.put(frame)
                                frame_batch.clear()
                    buffer = ""
                    last_line_ts = time.time()
                else:
                    buffer += char
            
            # Flush remaining batch periodically
            if frame_batch and (time.time() - last_line_ts > batch_timeout):
                for frame in frame_batch:
                    tcp_queue.put(frame)
                frame_batch.clear()
                    
        except (serial.SerialException, UnicodeDecodeError):
            logging.error("Serial port error or disconnection.")
            stop_event.set()
            break
    
    if not quiet:
        logging.info("Serial reader thread stopped.")


def tcp_writer(client_socket: socket.socket, tcp_queue: queue.Queue, stop_event: threading.Event, output_file_path: str = None):
    """Sends frames from the queue to the TCP client and optionally mirrors to a file."""
    logging.info("TCP writer thread started.")
    file_handle = None
    if output_file_path:
        try:
            # Check if file exists to determine if we need to write header
            file_exists = os.path.exists(output_file_path)
            file_handle = open(output_file_path, 'ab')
            logging.info(f"Mirroring output to file: {output_file_path}")
            
            # Write header if file is new (empty or just created)
            if not file_exists or os.path.getsize(output_file_path) == 0:
                # Determine format from file extension or assume GVRET
                if output_file_path.lower().endswith('.crtd'):
                    header = "// CAN Log File\n// Format: CRTD\n// Generated: " + time.strftime("%Y-%m-%d %H:%M:%S") + "\n"
                else:
                    # Default to GVRET header
                    header = "Time Stamp,ID,Extended,Dir,Bus,LEN,D1,D2,D3,D4,D5,D6,D7,D8\n"
                file_handle.write(header.encode('ascii'))
                file_handle.flush()
        except Exception as e:
            logging.error(f"Failed to open output file '{output_file_path}': {e}")
            file_handle = None
    while not stop_event.is_set() and not shutdown_event.is_set():
        try:
            frame = tcp_queue.get(timeout=1)
            client_socket.sendall(frame)
            if file_handle is not None:
                try:
                    file_handle.write(frame)
                except Exception:
                    pass
        except queue.Empty:
            continue
        except socket.timeout:
            # Timeout is expected, continue the loop
            continue
        except socket.error as e:
            logging.error(f"TCP socket error on write: {e}")
            stop_event.set()
            break
    if file_handle is not None:
        try:
            file_handle.flush()
            file_handle.close()
        except Exception:
            pass
    logging.info("TCP writer thread stopped.")


def serial_writer(ser: serial.Serial, serial_queue: queue.Queue, stop_event: threading.Event):
    """Writes AT commands from a queue to the serial port."""
    logging.info("Serial writer thread started.")
    while not stop_event.is_set() and not shutdown_event.is_set():
        try:
            cmd_list = serial_queue.get(timeout=1)
            for cmd in cmd_list:
                ser.write(cmd)
                time.sleep(0.005)  # Reduced delay for clones
                # Drain the response to prevent buffer overflow
                ser.read_until(b'>')
        except queue.Empty:
            continue
        except serial.SerialException as e:
            logging.error(f"Serial port error on write: {e}")
            stop_event.set()
            break
    logging.info("Serial writer thread stopped.")


def gvret_command_handler(client_socket: socket.socket, data: bytes, serial_queue: queue.Queue):
    """Processes a single GVRET command from SavvyCAN."""
    if not data or data[0] != GVRET_COMMAND_ID:
        return

    command = data[1]
    logging.debug(f"Received GVRET command: 0x{command:02X}")
    response = None

    if command == 0x00:  # Build CAN Frame (TX)
        # Format: F1 00 [ID4] [BUS] [LEN] [DATA...LEN] [0]
        frame_id = int.from_bytes(data[2:6], 'little')
        bus = data[6] & 0x0F
        dlc = data[7] & 0x0F
        payload = data[8:8+dlc]
        
        is_extended = bool(frame_id & (1 << 31))
        if is_extended:
            frame_id &= ~(1 << 31)

        # Zero-pad ATSH header explicitly
        hdr = f"{frame_id:08X}" if is_extended else f"{frame_id:03X}"
        at_commands = [
            f'ATSH{hdr}\r'.encode('ascii'),
            f'{payload.hex()}\r'.encode('ascii')
        ]
        serial_queue.put(at_commands)

    elif command == 0x01: # Time Sync
        response = bytearray([GVRET_COMMAND_ID, 0x01])
        # GVRET expects microseconds since boot (we'll use system time lower 32 bits)
        response += (int(time.time() * 1000000) & 0xFFFFFFFF).to_bytes(4, 'little')
    
    elif command == 0x06: # Get CANBus Params
        # CAN0: enabled, 500000 bps (0x0007A120 LE = 0x20,0xA1,0x07,0x00)
        # CAN1: disabled, 0 bps
        response = bytes([
            GVRET_COMMAND_ID, 0x06,
            0x01,              # CAN0 flags: enabled
            0x20, 0xA1, 0x07, 0x00, # CAN0 baud LE
            0x00,              # CAN1 flags: disabled
            0x00, 0x00, 0x00, 0x00  # CAN1 baud LE
        ])

    elif command == 0x07: # Get Device Info
        build_num = 1234
        response = bytes([
            GVRET_COMMAND_ID, 0x07,
            build_num & 0xFF, (build_num >> 8) & 0xFF,
            1, 1, 0
        ])

    elif command == 0x09: # Comm validation
        response = bytes([GVRET_COMMAND_ID, 0x09])

    elif command == 0x0C: # Get Num Buses
        response = bytes([GVRET_COMMAND_ID, 0x0C, 1]) # We have 1 bus

    elif command == 0x0D: # Get Extended Buses (SWCAN/LIN) - return zeros
        # Payload: 15 bytes
        # [swcanFlags] [swcanBaud 4] [lin1Flags] [lin1Baud 4] [lin2Flags] [lin2Baud 4]
        response = bytes([GVRET_COMMAND_ID, 0x0D] + [0x00]*15)
        
    if response:
        logging.debug(f"Sending GVRET response: {response.hex()}")
        try:
            client_socket.sendall(response)
        except socket.error as e:
            logging.error(f"Failed to send GVRET response: {e}")


def process_gvret_buffer(buffer: bytearray, client_socket: socket.socket, serial_queue: queue.Queue):
    """
    Parses all complete GVRET commands from a buffer, handles them, 
    and returns the remaining (incomplete) buffer.
    """
    while len(buffer) > 1:
        if buffer[0] != GVRET_COMMAND_ID:
            buffer = buffer[1:] # Sync to next command marker
            continue

        cmd = buffer[1]
        frame_len = 0
        
        # Determine frame length based on command
        if cmd in [0x01, 0x06, 0x07, 0x09, 0x0C, 0x0D]:
            frame_len = 2 # Fixed length handshake commands
        elif cmd == 0x00: # Build CAN Frame (variable length)
            if len(buffer) < 8: # Not enough data for header (up to DLC)
                break # Incomplete frame, wait for more data
            dlc = buffer[7] & 0x0F
            min_len = 8 + dlc
            if len(buffer) < min_len:
                break # Incomplete frame, wait for more data
            frame_len = min_len
            # Tolerate optional 0x00 terminator
            if len(buffer) >= min_len + 1 and buffer[min_len] == 0x00:
                frame_len = min_len + 1
        else:
            # Unrecognized command, skip it to avoid getting stuck
            buffer = buffer[1:]
            continue

        if len(buffer) >= frame_len:
            command_data = buffer[:frame_len]
            gvret_command_handler(client_socket, bytes(command_data), serial_queue)
            buffer = buffer[frame_len:]
        else:
            break # Incomplete frame, wait for more data
            
    return buffer


def tcp_reader(client_socket: socket.socket, serial_queue: queue.Queue, stop_event: threading.Event):
    """Reads GVRET frames from TCP and puts AT commands for the serial writer."""
    logging.info("TCP reader thread started.")
    buffer = bytearray()
    while not stop_event.is_set() and not shutdown_event.is_set():
        try:
            data = client_socket.recv(1024)
            if not data:
                logging.info("TCP client disconnected.")
                stop_event.set()
                break
            
            buffer.extend(data)
            buffer = process_gvret_buffer(buffer, client_socket, serial_queue)
            
        except socket.timeout:
            # Timeout is expected, continue the loop
            continue
        except socket.error as e:
            logging.error(f"TCP socket error on read: {e}")
            stop_event.set()
            break
    logging.info("TCP reader thread stopped.")


def is_port_available(port: str) -> bool:
    """Check if a serial port is available for use."""
    try:
        # Try to open the port briefly to see if it's available
        test_ser = serial.Serial(port, 115200, timeout=0.1)
        test_ser.close()
        return True
    except (serial.SerialException, OSError):
        return False

def initialize_elm_device(ser: serial.Serial, args) -> bool:
    """Initialize ELM/STN device with AT commands. Returns True if successful."""
    try:
        # Initialize ELM/STN device with AT commands
        init_commands = [
            b'ATZ\r',            # Reset device
            b'ATE 0\r',
            b'ATL 0\r',
            b'ATS 0\r',
            b'ATH 1\r',
            b'ATCAF 0\r',
            b'ATCFC0\r',
            b'ATCSM 1\r',      # Silent mode (do not ACK) - safest on shared bus
            f'STP {args.protocol}\r'.encode('ascii'),  # Use CLI protocol
            b'STPTO 20\r',       # Protocol timeout
            b'STCMM 1\r',      # Monitor mode: normal
         ]
        
        # Add flow control if it worked
        if hasattr(args, 'flow_control_working') and args.flow_control_working and (args.test_flow_control or args.force_flow_control):
            init_commands.append(b'STFCSD 500000\r')

        # Add filtering if specified
        if args.filter:
            init_commands.extend([
                b'STFPC\r',
                f'STFPA {args.filter}\r'.encode('ascii'),
            ])

        # Add monitoring commands
        init_commands.extend([
            f'STCMM {args.monitor_mode}\r'.encode('ascii'),  # Use CLI monitor mode
            b'STMA\r'           # Start monitoring all traffic
        ])
        
        # Add custom init commands if provided
        if args.custom_init:
            init_commands.extend([f"{cmd}\r".encode('ascii') for cmd in args.custom_init])
        
        logging.info("Initializing ELM/STN device...")
        for cmd in init_commands:
            ser.write(cmd)
            time.sleep(args.init_delay)
            if cmd == b'STMA\r':
                logging.info(f"Sent: {cmd.strip().decode()}, entering monitor mode...")
                continue
            response = ser.read_until(b'>').decode('ascii', errors='ignore')
            logging.debug(f"Sent: {cmd.strip().decode()}, Got: {response.strip()}")
            if "?" in response:
                if cmd == b'STCMM 1\r':
                    logging.warning("STCMM 1 failed, trying STCMM 2...")
                    ser.write(b'STCMM 2\r')
                    time.sleep(args.init_delay)
                    response = ser.read_until(b'>').decode('ascii', errors='ignore')
                    logging.debug(f"Sent: STCMM 2, Got: {response.strip()}")
                elif cmd == b'STMA\r':
                    logging.warning("STMA failed, falling back to ATMA.")
                    ser.write(b'ATMA\r')
                    time.sleep(args.init_delay)
                    logging.info("Sent: ATMA, entering monitor mode...")
        
        logging.info("ELM/STN device initialized successfully.")
        return True
    except Exception as e:
        logging.error(f"Failed to initialize ELM/STN device: {e}")
        return False

def run_bridge(args):
    """Main bridge logic without restart loop."""
    # Setup logging
    log_level = logging.DEBUG if args.debug else logging.INFO
    log_format = '%(asctime)s %(levelname)s %(message)s'
    
    if args.log_file:
        logging.basicConfig(level=log_level, filename=args.log_file, format=log_format)
    else:
        logging.basicConfig(level=log_level, format=log_format)
    
    # Suppress debug logs if quiet mode
    if args.quiet:
        logging.getLogger().setLevel(logging.WARNING)
    
    # Setup signal handlers for graceful shutdown
    setup_signal_handlers()
    logging.info("Graceful shutdown enabled. Press Ctrl+C or send SIGTERM to shutdown cleanly.")
    
    # Check if pyserial is available
    if serial is None:
        logging.error("pyserial is required. Install with: pip install pyserial")
        return
    
    # Map CLI values to pyserial enums
    parity_map = {'N': serial.PARITY_NONE, 'E': serial.PARITY_EVEN, 'O': serial.PARITY_ODD, 'M': serial.PARITY_MARK, 'S': serial.PARITY_SPACE}
    bytesize_map = {5: serial.FIVEBITS, 6: serial.SIXBITS, 7: serial.SEVENBITS, 8: serial.EIGHTBITS}
    stopbits_map = {1: serial.STOPBITS_ONE, 2: serial.STOPBITS_TWO}
    
    # Check if port is available before attempting to open
    if not is_port_available(args.port):
        logging.error(f"Serial port {args.port} is not available. It may be in use by another application.")
        logging.error("Please close any other applications using this port and try again.")
        logging.error("Common solutions:")
        logging.error("  1. Close SavvyCAN or other CAN tools")
        logging.error("  2. Kill any running Python processes: taskkill /f /im python.exe")
        logging.error("  3. Unplug and replug the USB adapter")
        logging.error("  4. Restart the computer if needed")
        return
    
    logging.info(f"Attempting to open serial port {args.port} at {args.baud} baud.")
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
        logging.error(f"Error opening serial port: {e}")
        return

    # Handle flow control based on CLI flags
    flow_control_working = False
    if args.disable_flow_control:
        # Keep ATCFC0 in init commands (already there)
        pass
    elif args.force_flow_control:
        if test_flow_control(ser, args.force_flow_control):
            logging.info(f"✓ {args.force_flow_control} flow control works!")
            flow_control_working = True
        else:
            logging.warning(f"✗ {args.force_flow_control} flow control failed")
    elif args.test_flow_control:
        for flow_type in ['none', 'hardware_cts', 'hardware_dtr', 'software']:
            if test_flow_control(ser, flow_type):
                logging.info(f"✓ {flow_type} flow control works!")
                flow_control_working = True
                break
            else:
                logging.warning(f"✗ {flow_type} flow control failed")
    
    # Initialize ELM device
    if not args.skip_init:
        if not initialize_elm_device(ser, args):
            logging.error("Failed to initialize ELM device. Exiting.")
            ser.close()
            return

    # File-only mode: log directly to file without TCP server
    if args.file_only:
        logging.info(f"File-only mode: logging to {args.output_file}")
        
        # Open output file
        try:
            output_file = open(args.output_file, 'w', newline='')
            logging.info(f"Opened output file: {args.output_file}")
            
            # Write format-specific header
            if args.format == 'gvret':
                header = "Time Stamp,ID,Extended,Dir,Bus,LEN,D1,D2,D3,D4,D5,D6,D7,D8\n"
                output_file.write(header)
                output_file.flush()
            elif args.format == 'crtd':
                # CRTD doesn't require a header, but we can add an optional comment
                header = "// CAN Log File\n// Format: CRTD\n// Generated: " + time.strftime("%Y-%m-%d %H:%M:%S") + "\n"
                output_file.write(header)
                output_file.flush()
        except Exception as e:
            logging.error(f"Failed to open output file '{args.output_file}': {e}")
            return
        
        stop_event = threading.Event()
        serial_to_file_queue = queue.Queue()
        
        # Start serial reader thread
        serial_thread = threading.Thread(
            target=serial_reader, 
            args=(ser, serial_to_file_queue, stop_event), 
            kwargs={
                'quiet': args.quiet, 
                'batch_size': args.batch_size, 
                'batch_timeout': args.batch_timeout, 
                'extended_only': args.extended_only, 
                'standard_only': args.standard_only, 
                'output_format': args.format
            }
        )
        serial_thread.start()
        
        # File writer loop
        try:
            while not stop_event.is_set() and not shutdown_event.is_set():
                try:
                    line = serial_to_file_queue.get(timeout=1)
                    line_str = line.decode('ascii')
                    logging.debug(f"Writing to file: {line_str.strip()}")
                    output_file.write(line_str)
                    output_file.flush()
                except queue.Empty:
                    continue
        except KeyboardInterrupt:
            logging.info("Ctrl+C detected. Shutting down.")
        finally:
            logging.info("Stopping file logging...")
            stop_event.set()
            # Stop monitoring
            try:
                ser.write(b'STMA 0\r')
                time.sleep(0.05)
            except Exception:
                try:
                    ser.write(b'ATPC\r')
                    time.sleep(0.05)
                except Exception:
                    pass
            ser.write(b'\r')
            output_file.close()
            serial_thread.join(timeout=2.0)
            ser.close()
            logging.info("File logging stopped.")
        return

    logging.info("Device initialized. Starting TCP server...")

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1 if args.reuse_addr else 0)
    server.bind((args.host, args.tcp_port))
    server.listen(args.backlog)
    logging.info(f"TCP server listening on {args.host}:{args.tcp_port}")

    try:
        while not shutdown_event.is_set():
            try:
                # Use a timeout for accept to allow checking shutdown_event
                server.settimeout(1.0)
                client_socket, addr = server.accept()
                logging.info(f"Accepted connection from {addr}")
            except socket.timeout:
                # Timeout is expected, continue to check shutdown_event
                continue
            except OSError as e:
                if shutdown_event.is_set():
                    break
                logging.error(f"Server accept error: {e}")
                break

            # Set socket timeouts and TCP_NODELAY
            client_socket.settimeout(args.tcp_timeout)
            client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

            stop_event = threading.Event()
            serial_to_tcp_queue = queue.Queue()

            # Text output modes: GVRET/CRTD
            if args.format in ('gvret', 'crtd'):
                logging.info(f"Starting text streaming in '{args.format}' format.")

                threads = [
                    threading.Thread(target=serial_reader, args=(ser, serial_to_tcp_queue, stop_event), kwargs={'quiet': args.quiet, 'batch_size': args.batch_size, 'batch_timeout': args.batch_timeout, 'extended_only': args.extended_only, 'standard_only': args.standard_only, 'output_format': args.format}),
                    threading.Thread(target=tcp_writer, args=(client_socket, serial_to_tcp_queue, stop_event), kwargs={'output_file_path': args.output_file})
                ]

                for t in threads:
                    t.start()

                # Wait for any thread to signal a stop or global shutdown
                while not stop_event.is_set() and not shutdown_event.is_set():
                    time.sleep(0.5)

                logging.info("Closing connection and stopping threads...")
                stop_event.set()  # Signal all threads to stop
                # Explicitly stop monitoring before teardown
                try:
                    ser.write(b'STMA 0\r')  # Stop monitoring
                    time.sleep(0.05)
                except Exception:
                    try:
                        ser.write(b'ATPC\r')  # Fallback stop command
                        time.sleep(0.05)
                    except Exception:
                        pass
                ser.write(b'\r') # Final CR
                client_socket.close()
                for t in threads:
                    t.join(timeout=2.0)  # Wait up to 2 seconds for threads to finish
                
                # Close serial port on disconnect to properly reset adapter
                if not shutdown_event.is_set():
                    logging.info("Closing serial port to reset adapter state...")
                    ser.close()
                    # Reopen serial port for next connection
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
                        logging.info(f"Reopened serial port: {args.port}")
                        
                        # Reinitialize ELM device
                        if not initialize_elm_device(ser, args):
                            logging.error("Failed to reinitialize ELM device after reconnect.")
                            break
                    except Exception as e:
                        logging.error(f"Failed to reopen serial port: {e}")
                        break

                if args.exit_on_disconnect or shutdown_event.is_set():
                    break

                # Continue to accept next connection
                continue

            # Binary GVRET mode (legacy)
            try:
                initial_data = client_socket.recv(1024)
            except socket.timeout:
                logging.warning("Client handshake timed out.")
                client_socket.close()
                continue

            if b'\xE7\xE7' not in initial_data:
                logging.warning("Client did not request binary mode. Closing.")
                client_socket.close()
                continue

            logging.info("GVRET binary mode entered.")

            # Send initial GVRET handshake responses
            try:
                # Send device info response
                device_info = bytes([GVRET_COMMAND_ID, 0x07, 0xD2, 0x04, 1, 1, 0])  # Build 1234
                client_socket.sendall(device_info)
                logging.debug("Sent device info response")

                # Send CAN bus params response
                can_params = bytes([GVRET_COMMAND_ID, 0x06, 0x01, 0x20, 0xA1, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00])
                client_socket.sendall(can_params)
                logging.debug("Sent CAN bus params response")

                # Send number of buses
                num_buses = bytes([GVRET_COMMAND_ID, 0x0C, 1])
                client_socket.sendall(num_buses)
                logging.debug("Sent number of buses response")

            except socket.error as e:
                logging.error(f"Failed to send initial GVRET responses: {e}")
                client_socket.close()
                continue

            tcp_to_serial_queue = queue.Queue()

            threads = [
                threading.Thread(target=serial_reader, args=(ser, serial_to_tcp_queue, stop_event), kwargs={'quiet': args.quiet, 'batch_size': args.batch_size, 'batch_timeout': args.batch_timeout, 'extended_only': args.extended_only, 'standard_only': args.standard_only, 'output_format': 'binary'}),
                threading.Thread(target=tcp_writer, args=(client_socket, serial_to_tcp_queue, stop_event), kwargs={'output_file_path': args.output_file}),
                threading.Thread(target=tcp_reader, args=(client_socket, tcp_to_serial_queue, stop_event)),
                threading.Thread(target=serial_writer, args=(ser, tcp_to_serial_queue, stop_event))
            ]

            for t in threads:
                t.start()

            # The tcp_reader will now handle the GVRET handshake commands.
            # Also process any commands that arrived with the initial E7E7 burst.
            remnant = initial_data.replace(b'\xE7\xE7', b'')
            if remnant:
                logging.debug(f"Processing initial GVRET data: {remnant.hex()}")
                # Feed remnant into the unified parsing logic
                process_gvret_buffer(bytearray(remnant), client_socket, tcp_to_serial_queue)

            # Wait for any thread to signal a stop or global shutdown
            while not stop_event.is_set() and not shutdown_event.is_set():
                time.sleep(0.5)

            logging.info("Closing connection and stopping threads...")
            stop_event.set()  # Signal all threads to stop
            # Explicitly stop monitoring before teardown
            try:
                ser.write(b'STMA 0\r')  # Stop monitoring
                time.sleep(0.05)
            except Exception:
                try:
                    ser.write(b'ATPC\r')  # Fallback stop command
                    time.sleep(0.05)
                except Exception:
                    pass
            ser.write(b'\r') # Final CR
            client_socket.close()
            for t in threads:
                t.join(timeout=2.0)  # Wait up to 2 seconds for threads to finish
            
            # Close serial port on disconnect to properly reset adapter
            if not shutdown_event.is_set():
                logging.info("Closing serial port to reset adapter state...")
                ser.close()
                # Reopen serial port for next connection
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
                    logging.info(f"Reopened serial port: {args.port}")
                    
                    # Reinitialize ELM device
                    if not initialize_elm_device(ser, args):
                        logging.error("Failed to reinitialize ELM device after reconnect.")
                        break
                except Exception as e:
                    logging.error(f"Failed to reopen serial port: {e}")
                    break

            if args.exit_on_disconnect or shutdown_event.is_set():
                break

    except KeyboardInterrupt:
        logging.info("Ctrl+C detected. Shutting down.")
    finally:
        logging.info("Cleaning up resources...")
        shutdown_event.set()  # Ensure shutdown is signaled
        
        # Stop monitoring before closing serial port
        try:
            ser.write(b'STMA 0\r')  # Stop monitoring
            time.sleep(0.05)
        except Exception:
            try:
                ser.write(b'ATPC\r')  # Fallback stop command
                time.sleep(0.05)
            except Exception:
                pass
        
        server.close()
        ser.close()
        logging.info("Server and serial port closed.")


def main(args):
    """Main entry point with optional restart loop."""
    restart_count = 0
    max_restarts = args.max_restarts if args.restart_on_error else 0
    
    while restart_count <= max_restarts:
        try:
            run_bridge(args)
            break  # Normal exit
        except Exception as e:
            restart_count += 1
            if restart_count <= max_restarts:
                logging.error(f"Bridge crashed: {e}. Restarting ({restart_count}/{max_restarts})...")
                time.sleep(2)  # Brief delay before restart
            else:
                logging.error(f"Bridge crashed: {e}. Max restarts ({max_restarts}) exceeded.")
                raise


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="ELM327/STN to GVRET TCP Bridge for SavvyCAN",
        epilog="""
Examples:
  %(prog)s -p COM3
  %(prog)s -p COM3 -b 115200 --quiet
  %(prog)s -p COM3 --filter 7E0,7FF --test-flow-control
  %(prog)s -p COM3 --host 0.0.0.0 --tcp-port 8080
  %(prog)s -p COM3 --monitor-mode 2 --timeout 5
        """,
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    # Required arguments
    parser.add_argument('-p', '--port', 
                       help="Serial port of the ELM327 device (e.g., COM3, /dev/ttyUSB0)")
    
    # Serial communication options
    serial_group = parser.add_argument_group('Serial Communication')
    serial_group.add_argument('-b', '--baud', type=int, default=115200, 
                             choices=[9600, 19200, 38400, 57600, 115200, 230400, 460800, 500000, 1000000],
                             help="Baud rate for the serial port (default: 115200)")
    serial_group.add_argument('--timeout', type=float, default=1.0,
                             help="Serial port timeout in seconds (default: 1.0)")
    serial_group.add_argument('--bytesize', type=int, default=8, choices=[5, 6, 7, 8],
                             help="Serial data bits (default: 8)")
    serial_group.add_argument('--parity', choices=['N', 'E', 'O', 'M', 'S'], default='N',
                             help="Serial parity (default: N)")
    serial_group.add_argument('--stopbits', type=int, choices=[1, 2], default=1,
                             help="Serial stop bits (default: 1)")
    serial_group.add_argument('--xonxoff', action='store_true',
                             help="Enable software flow control (XON/XOFF)")
    serial_group.add_argument('--rtscts', action='store_true',
                             help="Enable hardware flow control (RTS/CTS)")
    serial_group.add_argument('--dsrdtr', action='store_true',
                             help="Enable hardware flow control (DSR/DTR)")
    
    # CAN protocol options
    can_group = parser.add_argument_group('CAN Protocol')
    can_group.add_argument('--protocol', type=int, default=33, choices=[6, 33, 34],
                           help="CAN protocol: 6=ISO 15765-4 (11-bit), 33=ISO 15765-4 (29-bit), 34=ISO 15765-4 (both) (default: 33)")
    can_group.add_argument('--monitor-mode', type=int, choices=[0, 1, 2], default=1,
                           help="Monitor mode: 0=off, 1=normal, 2=extended (default: 1)")
    can_group.add_argument('--filter', 
                           help="CAN ID filter range (e.g., 7E0,7FF or 7E0-7FF)")
    can_group.add_argument('--extended-only', action='store_true',
                           help="Only capture extended (29-bit) CAN frames")
    can_group.add_argument('--standard-only', action='store_true',
                           help="Only capture standard (11-bit) CAN frames")
    
    # TCP server options
    tcp_group = parser.add_argument_group('TCP Server')
    tcp_group.add_argument('--host', default='127.0.0.1',
                           help="Host address for the TCP server (default: 127.0.0.1)")
    tcp_group.add_argument('--tcp-port', type=int, default=23,
                           help="Port for the TCP server (default: 23)")
    tcp_group.add_argument('--backlog', type=int, default=1,
                           help="Maximum number of queued connections (default: 1)")
    tcp_group.add_argument('--reuse-addr', action='store_true', default=True,
                           help="Allow address reuse (default: True)")
    
    # Performance options
    perf_group = parser.add_argument_group('Performance')
    perf_group.add_argument('--batch-size', type=int, default=5,
                            help="Frame batch size for processing (default: 5)")
    perf_group.add_argument('--batch-timeout', type=float, default=0.005,
                            help="Batch timeout in seconds (default: 0.005)")
    perf_group.add_argument('--serial-delay', type=float, default=0.005,
                            help="Delay between serial commands in seconds (default: 0.005)")
    perf_group.add_argument('--tcp-timeout', type=float, default=1.0,
                            help="TCP socket timeout in seconds (default: 1.0)")
    
    # Flow control options
    flow_group = parser.add_argument_group('Flow Control')
    flow_group.add_argument('--test-flow-control', action='store_true',
                            help="Test different flow control modes and use the best one")
    flow_group.add_argument('--force-flow-control', choices=['none', 'hardware_cts', 'hardware_dtr', 'software'],
                            help="Force specific flow control mode")
    flow_group.add_argument('--disable-flow-control', action='store_true',
                            help="Disable all flow control")
    
    # Logging and output options
    log_group = parser.add_argument_group('Logging and Output')
    log_group.add_argument('--quiet', action='store_true',
                           help="Disable verbose logging of CAN frames")
    log_group.add_argument('--debug', action='store_true',
                           help="Enable debug logging")
    log_group.add_argument('--log-file',
                           help="Log output to file instead of console")
    log_group.add_argument('--timestamp-format', choices=['none', 'iso', 'unix', 'elapsed'], default='elapsed',
                           help="Timestamp format for log messages (default: elapsed)")
    
    # Behavior options
    behavior_group = parser.add_argument_group('Behavior')
    behavior_group.add_argument('--exit-on-disconnect', action='store_true',
                                help="Exit the script when the TCP client disconnects")
    behavior_group.add_argument('--restart-on-error', action='store_true',
                                help="Restart the bridge on serial port errors")
    behavior_group.add_argument('--max-restarts', type=int, default=5,
                                help="Maximum number of restarts on error (default: 5)")
    behavior_group.add_argument('--graceful-shutdown', action='store_true', default=True,
                                help="Enable graceful shutdown on signals (default: True)")
    
    # Advanced options
    advanced_group = parser.add_argument_group('Advanced')
    advanced_group.add_argument('--init-delay', type=float, default=0.1,
                                help="Delay between initialization commands (default: 0.1)")
    advanced_group.add_argument('--response-timeout', type=float, default=5.0,
                                help="Timeout for ELM327 responses (default: 5.0)")
    advanced_group.add_argument('--custom-init', nargs='+',
                                help="Custom initialization commands (space-separated)")
    advanced_group.add_argument('--skip-init', action='store_true',
                                help="Skip ELM327 initialization (for debugging)")
    
    # Version and info
    parser.add_argument('--version', action='version', version='%(prog)s 1.0.0')
    parser.add_argument('--list-ports', action='store_true',
                        help="List available serial ports and exit")
    # Output format options
    output_group = parser.add_argument_group('Output Format')
    output_group.add_argument('--format', choices=['gvret', 'crtd', 'binary'], default='binary',
                              help="Output format to stream over TCP: gvret (CSV), crtd (space-separated), or binary GVRET (default: binary)")
    output_group.add_argument('--output-file',
                              help="Optional file to mirror the streamed output (append mode)")
    output_group.add_argument('--file-only', action='store_true',
                              help="Log directly to file without TCP server (requires --output-file)")
    
    args = parser.parse_args()
    
    # Handle list-ports option
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
        except ImportError:
            print("Cannot list ports: pyserial not installed properly.")
        sys.exit(0)
    
    # Check if port is provided (required unless listing ports or file-only mode)
    if not args.port and not args.file_only:
        parser.error("the following arguments are required: -p/--port (unless using --file-only)")
    
    # Check if output-file is provided when using file-only mode
    if args.file_only and not args.output_file:
        parser.error("--output-file is required when using --file-only")
    
    main(args)
