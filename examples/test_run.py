from rd60xx import RD60xx
import time

RD60xx.DEBUG = True

print("Discovering devices...")
devices = RD60xx.discover()
print("Found devices:", devices)

print("\nOpening device with Modbus address 1...\n")

with RD60xx.open(1) as psu:
    fw_raw = psu.read_firmware_version()
    fw_str = psu.read_firmware_version_str()
    print(f"Firmware raw = {fw_raw}, parsed = {fw_str}")

    psu.set_current(2.0)

    print("\nSetting 0.8 V...")
    psu.set_voltage(0.8)
    psu.output_on()
    time.sleep(1.0)
    u, i = psu.read_output()
    print(f"Measured: U = {u:.3f} V, I = {i:.4f} A")

    print("\nSetting 5.8 V...")
    psu.set_voltage(5.8)
    time.sleep(1.0)
    u, i = psu.read_output()
    print(f"Measured: U = {u:.3f} V, I = {i:.4f} A")

print("\nDone. Output should be OFF now.")
