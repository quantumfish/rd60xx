"""
RD60xx Python Driver - Modbus/RTU version
A helper library for Riden RD60xx programmable power supplies over Modbus RTU (USB).

Supported models: RD6006, RD6006P, RD6012, RD6018, RD6024, RD6030

Features:
    * RD60xx.open(addr)     - find and open a single device by Modbus address
    * RD60xx.discover(...)  - discover all RD60xx devices on all serial ports
    * Context manager support with automatic output shutdown
    * Retry mechanism for robust communication
    * Automatic model detection and scaling
"""

__version__ = "0.1.0"
__author__ = "quantumfish"
__description__ = "Python driver for Riden RD60xx power supplies over Modbus RTU"

from .rd60xx import (
    RD60xx,
    RD60xxError,
    DeviceNotFoundError,
    AddressConflictError,
)

__all__ = [
    "RD60xx",
    "RD60xxError",
    "DeviceNotFoundError",
    "AddressConflictError",
]


# Package-level configuration
DEBUG = False

def enable_debug():
    """Enable debug output for all RD60xx instances."""
    global DEBUG
    DEBUG = True
    RD60xx.DEBUG = True

def disable_debug():
    """Disable debug output for all RD60xx instances."""
    global DEBUG
    DEBUG = False
    RD60xx.DEBUG = False

def get_version() -> str:
    """Return the package version."""
    return __version__

