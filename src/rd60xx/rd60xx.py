"""
Simple helper library for Riden RD60xx programmable power supplies
(RD6006, RD6006P, RD6012, RD6018, RD6024, RD6030, etc.) over Modbus RTU (USB).

Features:
    * RD60xx.open(addr)     - find and open a single device by Modbus address
    * RD60xx.discover(...)  - discover all RD60xx devices on all serial ports

Instance methods:
    * set_voltage(V)        - set output voltage setpoint (in Volts)
    * set_current(A)        - set current limit setpoint (in Amps)
    * output_on()/off()     - enable/disable the output
    * read_output()         - read measured voltage and current
    * read_setpoints()      - read configured voltage and current setpoints
    * read_firmware_version() / read_firmware_version_str()

Debug:
    * RD60xx.DEBUG = True   - enable debug prints
    * RD60xx.DEBUG = False  - disable debug prints (default)

Dependencies:
    pip install minimalmodbus pyserial
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional, Dict, List, Tuple

import minimalmodbus
import serial
import serial.tools.list_ports


# ================================================================
#   Constants and model table
# ================================================================

DEFAULT_BAUDRATE = 115200
DEFAULT_TIMEOUT = 0.25        # seconds
DEFAULT_MAX_RETRIES = 3       # number of retries for read/write operations

# Common RD60xx registers
REG_ID     = 0     # device ID / model code
REG_V_SET  = 8     # configured voltage
REG_I_SET  = 9     # configured current limit
REG_V_OUT  = 10    # measured output voltage
REG_I_OUT  = 11    # measured output current
REG_OUTPUT = 18    # output on/off (1 / 0)
REG_FW_VER = 3     # firmware version / build code (device-specific format)


@dataclass
class ModelInfo:
    """Model-specific scale factors and limits."""
    code: str
    v_lsb: float     # Volts per LSB for voltage registers
    i_lsb: float     # Amps per LSB for current registers
    v_max: float     # maximum voltage in Volts
    i_max: float     # maximum current in Amps


# Model table: extend as needed for your hardware park
MODEL_TABLE: Dict[str, ModelInfo] = {
    # RD6006 (non-P): 1 mV steps, 1 mA current steps
    "RD6006":  ModelInfo("RD6006",  v_lsb=1e-3, i_lsb=1e-3, v_max=60.0, i_max=6.0),

    # RD6006P: 1 mV steps, 0.1 mA current steps (confirmed on your hardware)
    "RD6006P": ModelInfo("RD6006P", v_lsb=1e-3, i_lsb=1e-4, v_max=60.0, i_max=6.0),

    # Other typical models (10 mA current steps)
    "RD6012":  ModelInfo("RD6012",  v_lsb=1e-3, i_lsb=1e-2, v_max=60.0, i_max=12.0),
    "RD6018":  ModelInfo("RD6018",  v_lsb=1e-3, i_lsb=1e-2, v_max=60.0, i_max=18.0),
    "RD6024":  ModelInfo("RD6024",  v_lsb=1e-3, i_lsb=1e-2, v_max=60.0, i_max=24.0),
    "RD6030":  ModelInfo("RD6030",  v_lsb=1e-3, i_lsb=1e-2, v_max=60.0, i_max=30.0),
}


# ================================================================
#   Exceptions
# ================================================================

class RD60xxError(Exception):
    """Base exception type for RD60xx-related errors."""
    pass


class DeviceNotFoundError(RD60xxError):
    """Raised when no device with the given Modbus address is found."""
    pass


class AddressConflictError(RD60xxError):
    """Raised when multiple devices share the same Modbus address."""
    pass


# ================================================================
#   Debug logging helper
# ================================================================

def _log(*args, **kwargs) -> None:
    """
    Internal logging helper.

    Uses the class-level flag RD60xx.DEBUG.
    No output is printed when RD60xx.DEBUG == False.
    """
    if getattr(RD60xx, "DEBUG", False):
        print(*args, **kwargs)


# ================================================================
#   Helper functions
# ================================================================

def _decode_model_from_id(id_value: int) -> Optional[str]:
    """
    Decode the model name from the device ID register (REG_ID).

    According to your mapping:
        60061 -> RD6006   (non-P version)
        60065 -> RD6006P  (P-version)

    For other models we use a "base" scheme:
        60121 -> base 6012 -> RD6012
        60181 -> base 6018 -> RD6018
        60241 -> base 6024 -> RD6024
        60301 -> base 6030 -> RD6030
    """
    # Explicit handling of RD6006 vs RD6006P
    if id_value == 60061:
        return "RD6006"
    if id_value == 60065:
        return "RD6006P"

    # Generic handling for larger models by dividing by 10
    base = id_value // 10
    if base == 6012:
        return "RD6012"
    if base == 6018:
        return "RD6018"
    if base == 6024:
        return "RD6024"
    if base == 6030:
        return "RD6030"

    # Unknown device ID
    return None


def _iter_ports() -> List[str]:
    """
    Enumerate all available serial ports using pyserial.
    Returns a list of port names, e.g. ['COM5', '/dev/ttyUSB0', ...].
    """
    return [p.device for p in serial.tools.list_ports.comports()]


def _probe_on_port(port: str, addr: int) -> Optional[Tuple[int, ModelInfo]]:
    """
    Try to talk to a device with given Modbus address on a specific serial port.

    Returns:
        (id_value, ModelInfo) if a RD60xx device is detected on this port+addr,
        None if there is no RD60xx device or no response.

    Behaviour:
        * If the port cannot be opened (OS error, busy, etc.) -> returns None.
        * If Modbus read fails -> returns None.
        * If device ID is decoded to a model which is not in MODEL_TABLE ->
          raises RD60xxError (this is a configuration issue in the library).
    """
    try:
        instr = minimalmodbus.Instrument(port, addr)
    except Exception:
        # Serial port cannot be opened or is invalid -> skip silently.
        return None

    instr.serial.baudrate = DEFAULT_BAUDRATE
    instr.serial.bytesize = 8
    instr.serial.parity   = serial.PARITY_NONE
    instr.serial.stopbits = 1
    instr.serial.timeout  = DEFAULT_TIMEOUT
    instr.mode = minimalmodbus.MODE_RTU

    try:
        id_val = instr.read_register(REG_ID, 0, functioncode=3)
    except Exception:
        # No response or invalid Modbus -> not our device.
        return None

    model_code = _decode_model_from_id(id_val)
    if model_code is None:
        return None

    info = MODEL_TABLE.get(model_code)
    if info is None:
        raise RD60xxError(
            f"Model '{model_code}' (ID={id_val}) is not defined in MODEL_TABLE"
        )

    return id_val, info


# ================================================================
#   RD60xx class
# ================================================================

class RD60xx:
    """
    Abstraction for a single RD60xx power supply.

    Typical usage:

        from rd60xx import RD60xx

        RD60xx.DEBUG = True  # optional

        with RD60xx.open(1) as psu:
            psu.set_voltage(15.0)
            psu.set_current(2.0)
            psu.output_on()

            u, i = psu.read_output()
            print("U =", u, "V, I =", i, "A")

        # Output will be turned off automatically on leaving 'with'-block.
    """

    # Class-level flag controlling debug output
    DEBUG: bool = False

    def __init__(self, port: str, addr: int, model_info: ModelInfo) -> None:
        """
        Initialize a RD60xx instance bound to a specific serial port and Modbus address.

        Args:
            port: serial port name, e.g. 'COM5' or '/dev/ttyUSB0'.
            addr: Modbus slave address configured on the device.
            model_info: ModelInfo with proper scale factors and limits.
        """
        self.port = port
        self.addr = addr
        self.model_info = model_info

        instr = minimalmodbus.Instrument(port, addr)
        instr.serial.baudrate = DEFAULT_BAUDRATE
        instr.serial.bytesize = 8
        instr.serial.parity   = serial.PARITY_NONE
        instr.serial.stopbits = 1
        instr.serial.timeout  = DEFAULT_TIMEOUT
        instr.mode = minimalmodbus.MODE_RTU

        self._instr = instr

    # ------------------------------------------------------------
    #   Context manager support
    # ------------------------------------------------------------

    def __enter__(self) -> "RD60xx":
        """
        Enter the context manager.

        Returns:
            Self, so you can use "with RD60xx.open(addr) as psu:".
        """
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """
        Exit the context manager.

        This method is called at the end of the 'with' block,
        even if an exception was raised inside the block.

        We use it to safely turn the output off.
        """
        try:
            self.output_off()
        except Exception:
            # Suppress errors during cleanup.
            pass

    # ------------------------------------------------------------
    #   1. Open a single device by Modbus address
    # ------------------------------------------------------------

    @classmethod
    def open(cls, addr: int) -> "RD60xx":
        """
        Find and open exactly one RD60xx device with the given Modbus address.

        Scans all available serial ports and tries to talk to the device.
        Ports that are busy or not responding are silently skipped.

        Raises:
            DeviceNotFoundError: if no device with this address is found.
            AddressConflictError: if more than one device shares this address.

        Returns:
            RD60xx instance bound to the detected serial port and model.
        """
        ports = _iter_ports()
        _log(f"[rd60xx] open: scanning ports={ports} for addr={addr}")

        found: List[Tuple[str, int, ModelInfo]] = []

        for port in ports:
            _log(f"[rd60xx]   probing port {port}...")
            try:
                probed = _probe_on_port(port, addr)
            except RD60xxError as e:
                _log(f"[rd60xx]   error on {port}: {e}")
                continue

            if probed is None:
                _log(f"[rd60xx]   no RD60xx with addr={addr} on {port}")
                continue

            id_raw, info = probed
            _log(f"[rd60xx]   found {info.code} on {port}, ID={id_raw}")
            found.append((port, id_raw, info))

        if not found:
            raise DeviceNotFoundError(f"No RD60xx with address {addr} was found.")

        if len(found) > 1:
            desc = ", ".join(f"{p}:{mi.code}" for p, _, mi in found)
            raise AddressConflictError(
                f"Multiple devices with address {addr} detected: {desc}"
            )

        port, id_raw, info = found[0]
        _log(f"[rd60xx] opening {info.code} on {port}, ID={id_raw}")
        return cls(port, addr, info)

    # ------------------------------------------------------------
    #   2. Discover all devices (descriptions only)
    # ------------------------------------------------------------

    @classmethod
    def discover(cls, addr_min: int = 1, addr_max: int = 16) -> List[dict]:
        """
        Discover all RD60xx devices on all serial ports within the given address range.

        Args:
            addr_min: minimum Modbus address to probe (inclusive).
            addr_max: maximum Modbus address to probe (inclusive).

        Returns:
            List of dictionaries of the form:
                {
                    "port":  "COM5",
                    "addr":  1,
                    "model": "RD6006P",
                    "id":    60065
                }

        Notes:
            * This method does NOT raise on address conflicts (multiple devices
              with the same addr). It only logs a warning if DEBUG is True.
        """
        ports = _iter_ports()
        _log(f"[rd60xx] discover: ports={ports}, addrs={addr_min}..{addr_max}")

        results: List[dict] = []
        addr_map: Dict[int, List[Tuple[str, str]]] = {}

        for port in ports:
            _log(f"[rd60xx]   scanning port {port}...")
            for addr in range(addr_min, addr_max + 1):
                try:
                    probed = _probe_on_port(port, addr)
                except RD60xxError as e:
                    _log(f"[rd60xx]     addr={addr}: error {e}")
                    continue

                if probed is None:
                    _log(f"[rd60xx]     addr={addr}: empty")
                    continue

                id_raw, info = probed
                _log(
                    f"[rd60xx]     addr={addr}: found {info.code} on {port}, ID={id_raw}"
                )

                results.append({
                    "port":  port,
                    "addr":  addr,
                    "model": info.code,
                    "id":    id_raw,
                })

                addr_map.setdefault(addr, []).append((port, info.code))

        # Log address conflicts if any (do not raise here)
        for a, lst in addr_map.items():
            if len(lst) > 1:
                _log(f"[rd60xx] WARNING: address conflict addr={a}: {lst}")

        return results

    # ------------------------------------------------------------
    #   3. Low-level register helpers with retries
    # ------------------------------------------------------------

    def _write_reg(self, reg: int, value: int, name: str = "") -> None:
        """
        Write a single holding register with retries.

        Args:
            reg: register address.
            value: integer value to write.
            name: optional human-readable name for error messages.

        Raises:
            RD60xxError if all retries fail.
        """
        last_exc: Optional[Exception] = None
        for _ in range(DEFAULT_MAX_RETRIES):
            try:
                self._instr.write_register(reg, value, functioncode=6)
                return
            except Exception as e:
                last_exc = e
                time.sleep(0.05)
        raise RD60xxError(f"Failed to write {name or reg}: {last_exc}")

    def _read_reg(self, reg: int, name: str = "") -> int:
        """
        Read a single holding register with retries.

        Args:
            reg: register address.
            name: optional human-readable name for error messages.

        Returns:
            Integer value read from the register.

        Raises:
            RD60xxError if all retries fail.
        """
        last_exc: Optional[Exception] = None
        for _ in range(DEFAULT_MAX_RETRIES):
            try:
                return self._instr.read_register(reg, 0, functioncode=3)
            except Exception as e:
                last_exc = e
                time.sleep(0.05)
        raise RD60xxError(f"Failed to read {name or reg}: {last_exc}")

    # ------------------------------------------------------------
    #   4. Voltage and current configuration
    # ------------------------------------------------------------

    def set_voltage(self, voltage_v: float) -> None:
        """
        Set the output voltage setpoint in Volts.

        Checks that the requested value is within the model's valid range.

        Raises:
            ValueError if the requested voltage is out of range.
        """
        if not (0.0 <= voltage_v <= self.model_info.v_max):
            raise ValueError(
                f"Voltage {voltage_v} V is out of range "
                f"(0..{self.model_info.v_max} V)"
            )
        raw = int(round(voltage_v / self.model_info.v_lsb))
        self._write_reg(REG_V_SET, raw, "V_SET")

    def set_current(self, current_a: float) -> None:
        """
        Set the current limit setpoint in Amps.

        Checks that the requested value is within the model's valid range.

        Raises:
            ValueError if the requested current is out of range.
        """
        if not (0.0 <= current_a <= self.model_info.i_max):
            raise ValueError(
                f"Current {current_a} A is out of range "
                f"(0..{self.model_info.i_max} A)"
            )
        raw = int(round(current_a / self.model_info.i_lsb))
        self._write_reg(REG_I_SET, raw, "I_SET")

    # ------------------------------------------------------------
    #   5. Output control
    # ------------------------------------------------------------

    def output_on(self) -> None:
        """Enable the power supply output."""
        self._write_reg(REG_OUTPUT, 1, "OUTPUT")

    def output_off(self) -> None:
        """Disable the power supply output."""
        self._write_reg(REG_OUTPUT, 0, "OUTPUT")

    # ------------------------------------------------------------
    #   6. Reading measured and setpoint values
    # ------------------------------------------------------------

    def read_output(self) -> Tuple[float, float]:
        """
        Read measured output voltage and current.

        Returns:
            (U_out, I_out) in Volts and Amps.
        """
        v_raw = self._read_reg(REG_V_OUT, "V_OUT")
        i_raw = self._read_reg(REG_I_OUT, "I_OUT")
        return (
            v_raw * self.model_info.v_lsb,
            i_raw * self.model_info.i_lsb,
        )

    def read_setpoints(self) -> Tuple[float, float]:
        """
        Read configured voltage and current setpoints.

        Returns:
            (U_set, I_set) in Volts and Amps.
        """
        v_raw = self._read_reg(REG_V_SET, "V_SET")
        i_raw = self._read_reg(REG_I_SET, "I_SET")
        return (
            v_raw * self.model_info.v_lsb,
            i_raw * self.model_info.i_lsb,
        )

    # ------------------------------------------------------------
    #   7. Firmware version
    # ------------------------------------------------------------

    def read_firmware_version(self) -> int:
        """
        Read raw firmware version / build code from the device.

        The exact meaning of this value depends on the manufacturer.
        For example, you may get 145 for "v1.45", etc.

        Returns:
            Raw integer value from REG_FW_VER.
        """
        fw_raw = self._read_reg(REG_FW_VER, "FW_VERSION")
        return fw_raw

    def read_firmware_version_str(self) -> str:
        """
        Read firmware version and convert it to a human-readable string.

        This is a heuristic based on typical Riden encoding:
            fw_raw = 145  -> "v1.45"
            fw_raw = 130  -> "v1.30"

        If your devices use another format, adjust this method according
        to real values read from your hardware.
        """
        fw_raw = self.read_firmware_version()
        major = fw_raw // 100
        minor = fw_raw % 100
        return f"v{major}.{minor:02d}"


