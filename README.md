# Shady Elm

A bridge that connects ELM327 / STN1170 CAN adapters to [SavvyCAN](https://www.savvycan.com/) using the GVRET protocol over TCP. Also supports file-only logging in GVRET CSV or CRTD format.

## Install

Requires Python 3.8+ and `pyserial`:

```bash
pip install pyserial
```

## Quick start

```bash
# HS-CAN (500 kbps, 11-bit) on COM3, listening on 127.0.0.1:23
python savvycan_bridge.py --config STN1170_HSCAN_500000 -p COM3

# Ford MS-CAN (125 kbps)
python savvycan_bridge.py --config STN1170_MSCAN_125000 -p COM3
```

In SavvyCAN: connect as **GVRET** to `127.0.0.1:23`.

For file-only logging (no SavvyCAN):

```bash
python savvycan_bridge.py -p COM3 --file-only --output-file capture.bin --format binary
```

See `python savvycan_bridge.py --help` for the full option list.

## Configuration profiles

JSON files in [configs/](configs/) provide reusable adapter settings. Pass `--config <name>` (without `.json`) or `--config path/to/file.json`. Legacy aliases `STN1170_HSCAN` and `STN1170_MSCAN_Ford` are still accepted.

| Profile                | Bus    | Bitrate  | Frame IDs |
| ---------------------- | ------ | -------- | --------- |
| `STN1170_HSCAN_500000` | HS-CAN | 500 kbps | 11-bit    |
| `STN1170_MSCAN_125000` | MS-CAN | 125 kbps | 11-bit    |

To add a profile, drop a JSON file in [configs/](configs/) with `serial`, `can`, and `device` sections — see the existing files for shape.

## Output formats

- `binary` — GVRET binary (default; required for live SavvyCAN streaming)
- `gvret` — GVRET CSV (text, human-readable, compatible with SavvyCAN file import)
- `crtd` — CRTD text log

## Supported adapters

- **ELM327** — standard OBD-II CAN adapters
- **STN1170** — adds flow control, ID filtering, and monitor modes

The bridge starts monitoring with `STMA` (STN) and falls back to `ATMA` (ELM).

## Troubleshooting

- **Port unavailable** — close other apps holding the port; run with `--list-ports` to confirm the device is present.
- **Init failed** — try `--debug` to see the adapter's responses, or `--skip-init` if the adapter is already configured externally.
- **No frames** — verify CAN-bus wiring and try `--debug` to inspect raw serial.
- **Garbled data** — recheck `--baud` and flow control (`--test-flow-control` probes available modes).

Use `--env-check` to print Python and `pyserial` versions.

## Testing

```bash
python -m pytest tests/
```

## Layout

- [savvycan_bridge.py](savvycan_bridge.py) — bridge entrypoint
- [configs/](configs/) — adapter profiles
- [tests/](tests/) — pytest suite
- [docs/](docs/) — protocol notes
