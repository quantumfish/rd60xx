# rd60xx

Helper library for Riden RD60xx programmable power supplies
(RD6006, RD6006P, RD6012, RD6018, RD6024, RD6030, etc.) over Modbus RTU (USB).

## Features

- Auto-discovery by Modbus address across all serial ports.
- Automatic model detection:
  - ID `60061` → `RD6006`
  - ID `60065` → `RD6006P`
- Model-specific scaling for voltage and current.
- Simple high-level API:
  - `set_voltage(V)`, `set_current(A)`
  - `output_on()`, `output_off()`
  - `read_output()` (measured values)
  - `read_setpoints()` (configured values)
  - `read_firmware_version()` / `read_firmware_version_str()`
- Context manager support with automatic `output_off()` on exit.
- Debug prints toggle: `RD60xx.DEBUG = True`.

## Installation

From local source:

```bash
pip install build
python -m build
pip install dist/rd60xx-0.1.0-py3-none-any.whl
