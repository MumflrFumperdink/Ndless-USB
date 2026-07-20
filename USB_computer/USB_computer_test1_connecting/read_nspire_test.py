#!/usr/bin/env python3
"""
read_nspire_test.py

Reads from the bulk IN endpoint of the test USB device implemented by
usbdev_test.c on the TI-Nspire.

Requires: pip install pyusb
Also requires a libusb backend installed on your system (libusb-1.0).

VID/PID match the device descriptor in usbdev_test.c:
  idVendor  = 0x1209 (pid.codes shared test VID)
  idProduct = 0x0001 (unregistered -- testing only)
"""

import sys
import usb.core
import usb.util

VENDOR_ID = 0x1209
PRODUCT_ID = 0x0001
EP_IN = 0x81  # matches bEndpointAddress in config_descriptor
EP_OUT = 0x01  # matches bEndpointAddress in config_descriptor


def main():
    devices = usb.core.find(find_all=True)

    # Loop through the generator and print device details
    for dev in devices:
        print(f"Vendor ID: {hex(dev.idVendor)} | Product ID: {hex(dev.idProduct)}")

    
    dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
    if dev is None:
        print("Device not found. Is the Nspire program running and plugged in?")
        sys.exit(1)

    # Detach kernel driver if something claimed it (unlikely for a
    # vendor-class device, but harmless to check)
    try:
        if dev.is_kernel_driver_active(0):
            dev.detach_kernel_driver(0)
    except (NotImplementedError, usb.core.USBError):
        pass  # not all platforms support this; fine to ignore

    dev.set_configuration()

    print("Reading from EP1 IN... (Ctrl+C to stop)")
    try:
        while True:
            try:
                data = dev.read(EP_IN, 64, timeout=2000)
                text = bytes(data).decode(errors="replace")
                print(f"Got {len(data)} bytes: {text!r}")

                break;
            except usb.core.USBTimeoutError:
                print("(timeout, no data yet)")
    except KeyboardInterrupt:
        print("\nStopped.")


    data_to_send = b'test'    # Must be bytes, a string array, or an array of integers
    bytes_written = dev.write(EP_OUT, data_to_send, timeout=5000)


if __name__ == "__main__":
    main()
