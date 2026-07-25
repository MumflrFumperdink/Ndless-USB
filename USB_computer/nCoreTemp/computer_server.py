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

import os
import platform
import subprocess
import psutil
import json
import datetime
import select
import collections
import time

DEBUG_USB = False

# --- Persistent macmon process (macOS only) ---
_macmon_proc = None
_last_macmon_temp = None

def start_macmon_if_needed():
    global _macmon_proc
    if platform.system() != "Darwin" or _macmon_proc is not None:
        return
    try:
        _macmon_proc = subprocess.Popen(
            ["macmon", "pipe"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
    except Exception as e:
        log(f"Could not start macmon: {e}")
        _macmon_proc = None

def stop_macmon():
    global _macmon_proc
    if _macmon_proc is not None:
        _macmon_proc.terminate()
        try:
            _macmon_proc.wait(timeout=1)
        except Exception:
            _macmon_proc.kill()
        _macmon_proc = None

if DEBUG_USB:
    LOG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "nspire_usb_debug.log")
    _log_file = open(LOG_PATH, "a", buffering=1)

def log(msg):
    line = str(msg)
    print(line)
    if DEBUG_USB:
        _log_file.write(f"[{datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]}] {line}\n")

_cached_physical_cores = None
_cached_logical_cores = None

SMOOTHING_WINDOW_SECONDS = 0.5
_cpu_times_history = collections.deque()  # [(timestamp, percpu_times), ...]

def get_smoothed_cpu_percent():
    now = time.monotonic()
    times_now = psutil.cpu_times(percpu=True)
    _cpu_times_history.append((now, times_now))

    while len(_cpu_times_history) > 1 and now - _cpu_times_history[0][0] > SMOOTHING_WINDOW_SECONDS:
        _cpu_times_history.popleft()

    if len(_cpu_times_history) < 2:
        return [0.0] * len(times_now)

    old_time, old_times = _cpu_times_history[0]
    dt = now - old_time
    if dt <= 0:
        return [0.0] * len(times_now)

    percents = []
    for old_t, new_t in zip(old_times, times_now):
        total_delta = sum(new_t) - sum(old_t)
        idle_delta = new_t.idle - old_t.idle
        pct = ((total_delta - idle_delta) / total_delta * 100.0) if total_delta > 0 else 0.0
        percents.append(round(pct, 1))
    return percents

def get_cpu_info():
    global _cached_physical_cores, _cached_logical_cores
    if _cached_physical_cores is None:
        _cached_physical_cores = psutil.cpu_count(logical=False) or "N/A"
        _cached_logical_cores = psutil.cpu_count(logical=True) or "N/A"

    per_core_usage = get_smoothed_cpu_percent()
    total_usage = sum(per_core_usage) / len(per_core_usage) if per_core_usage else 0.0
    cpu_temp = get_temperature_by_os()
    return {
        "os": platform.system(),
        "cores_physical": _cached_physical_cores,
        "cores_logical": _cached_logical_cores,
        "total_usage_pct": total_usage,
        "per_core_usage_pct": per_core_usage,
        "temperature_celsius": cpu_temp
    }

def get_temperature_by_os():
    current_os = platform.system()
    if current_os == "Linux":
        try:
            temps = psutil.sensors_temperatures()
            if temps:
                for key in ['coretemp', 'cpu_thermal', 'acpitz']:
                    if key in temps and len(temps[key]) > 0:
                        return temps[key][0].current
            if os.path.exists("/sys/class/thermal/thermal_zone0/temp"):
                with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                    return float(f.read().strip()) / 1000.0
        except Exception:
            pass
        return "Unsupported Configuration"
    elif current_os == "Windows":
        try:
            import WinTmp
            return WinTmp.CPU_Temp()
        except ImportError:
            return "Missing Dependency: Run 'pip install WinTmp' as Admin"
        except Exception:
            return "Requires Elevated Administrator Command Prompt"
    elif current_os == "Darwin":
        global _last_macmon_temp
        if _macmon_proc is None or _macmon_proc.poll() is not None:
            return "macmon not running (start_macmon_if_needed() wasn't called, or it exited)"
        try:
            latest_line = None
            while select.select([_macmon_proc.stdout], [], [], 0)[0]:
                line = _macmon_proc.stdout.readline()
                if not line:
                    break
                line = line.strip()
                if line.startswith("{"):
                    latest_line = line

            if latest_line is None:
                return _last_macmon_temp if _last_macmon_temp is not None else "No macmon sample available yet"

            data = json.loads(latest_line)
            cpu_temp = data.get("temp", {}).get("cpu_temp_avg")
            if cpu_temp is not None:
                _last_macmon_temp = f"{round(cpu_temp, 1)}°C"
                return _last_macmon_temp
            return "No cpu_temp_avg field in macmon output"
        except json.JSONDecodeError:
            return "Failed to parse macmon JSON stream"
        except Exception as e:
            return f"macmon read error: {str(e)}"
    return "Unknown Platform OS"

def normalize_temperature(raw_temp_value):
    if isinstance(raw_temp_value, (int, float)):
        return float(raw_temp_value)
    if isinstance(raw_temp_value, str):
        cleaned = raw_temp_value.replace("°C", "").strip()
        try:
            return float(cleaned)
        except ValueError:
            return -1.0
    return -1.0

import sys
import usb.core
import usb.util
import struct
import array
import time
import errno

VENDOR_ID = 0x1209
PRODUCT_ID = 0x0001
EP_IN = 0x81
EP_OUT = 0x01
MAX_BULK_PACKET = 256
UPDATE_INTERVAL_SECONDS = 1.0 / 60.0

dev = None
def main():
    log(f"=== Run started {datetime.datetime.now().isoformat()} ===")
    psutil.cpu_percent(percpu=True)
    start_macmon_if_needed()

    dev = None
    try:
        while dev is None:
            dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
    except KeyboardInterrupt:
        return False

    log(f"Device found: bus={dev.bus} address={dev.address} bcdDevice=0x{dev.bcdDevice:04x}")

    try:
        if dev.is_kernel_driver_active(0):
            log("Kernel driver active on interface 0, detaching...")
            dev.detach_kernel_driver(0)
    except (NotImplementedError, usb.core.USBError):
        pass

    dev.set_configuration()
    try:
        active_cfg = dev.get_active_configuration()
        
        if DEBUG_USB:
            log(f"set_configuration() sent, current config now reads {active_cfg.bConfigurationValue}")
            for intf in active_cfg:
                log(f"  interface {intf.bInterfaceNumber} alt {intf.bAlternateSetting}: "
                    f"class=0x{intf.bInterfaceClass:02x} numEndpoints={intf.bNumEndpoints}")
                for ep in intf:
                    log(f"    endpoint 0x{ep.bEndpointAddress:02x}: "
                        f"attr=0x{ep.bmAttributes:02x} maxPacket={ep.wMaxPacketSize}")
    except usb.core.USBError as e:
        log(f"set_configuration() sent, but GET_CONFIGURATION check failed: {e}")

    try:
        usb.util.claim_interface(dev, 0)
        if DEBUG_USB:
            log("Interface 0 claimed explicitly.")
    except usb.core.USBError as e:
        log(f"claim_interface(0) failed: {e}")

    time.sleep(0.2)
    for ep in (EP_IN, EP_OUT):
        try:
            dev.clear_halt(ep)
            if DEBUG_USB:
                log(f"clear_halt(0x{ep:02x}) OK")
        except usb.core.USBError as e:
            log(f"clear_halt(0x{ep:02x}) failed (may be harmless): {e}")

    data = array.array('B', [1])
    try:
        while data.tobytes().decode('utf-8') != "HELLO FROM THE BARE METAL CALCULATOR!":
            try:
                data = dev.read(EP_IN, 64, timeout=2000)
                text = bytes(data).decode(errors="replace")
                log(f"EP_IN read OK, {len(data)} bytes: {text!r}")
                break
            except usb.core.USBTimeoutError:
                log("(timeout, no data yet)")
            except usb.core.USBError as e:
                log(f"EP_IN read error: {e} (errno={e.errno}), clearing halt and retrying")
                try:
                    dev.clear_halt(EP_IN)
                except usb.core.USBError:
                    pass
                time.sleep(0.1)
                continue
    except KeyboardInterrupt:
        return False

    last_verbose_log = 0.0
    loop_start = time.monotonic()
    frame_count = 0
    try:
        while True:
            iter_start = time.monotonic()
            stats = get_cpu_info()
            t_after_cpu_info = time.monotonic()

            raw_cores_physics = int(stats['cores_physical'])
            raw_cores_logical = int(stats['cores_logical'])
            raw_temp = normalize_temperature(stats['temperature_celsius'])
            raw_usage = float(stats['total_usage_pct'])

            payload = struct.pack(f"<iiff{len(stats['per_core_usage_pct'])}f", raw_cores_physics, raw_cores_logical, raw_usage, raw_temp, *stats['per_core_usage_pct'])

            if len(payload) > MAX_BULK_PACKET:
                max_cores_that_fit = (MAX_BULK_PACKET - 16) // 4
                payload = struct.pack(
                    f"<iiff{max_cores_that_fit}f",
                    raw_cores_physics, raw_cores_logical, raw_usage, raw_temp,
                    *stats['per_core_usage_pct'][:max_cores_that_fit]
                )

            try:
                bytes_written = dev.write(EP_OUT, payload, timeout=5000)
                write_ok = True
            except usb.core.USBError as e:
                if (e.backend_error_code == errno.ENODEV or e.errno == errno.ENODEV):
                    print("Device was physically disconnected!")
                    return True
                write_ok = False
                log(f"Write failed: {e}")
            t_after_write = time.monotonic()

            frame_count += 1
            now = time.monotonic()

            if DEBUG_USB:
                if now - last_verbose_log >= 1.0:
                    elapsed = now - loop_start
                    achieved_hz = frame_count / elapsed if elapsed > 0 else 0.0
                    cpu_info_ms = (t_after_cpu_info - iter_start) * 1000
                    write_ms = (t_after_write - t_after_cpu_info) * 1000
                    log(f"=== System Monitor [{stats['os']}] === "
                        f"(achieved {achieved_hz:.1f} fps over last {elapsed:.1f}s, {frame_count} frames | "
                        f"last frame: get_cpu_info={cpu_info_ms:.1f}ms dev.write={write_ms:.1f}ms)")
                    log(f"Processor Structure: {stats['cores_physical']} Physical / {stats['cores_logical']} Threads")
                    log(f"Total Load Capacity: {stats['total_usage_pct']}%")
                    log(f"Core Load Breakdown: {stats['per_core_usage_pct']}")
                    log(f"Current Thermal Core: {stats['temperature_celsius']}")
                    log(f"Packed payload: {len(payload)} bytes, wrote={write_ok}")
                    last_verbose_log = now
                    loop_start = now
                    frame_count = 0
                else:
                    log(f"frame: usage={stats['total_usage_pct']:.1f}% write_ok={write_ok}")

            # Account for time already spent this iteration instead of
            # blindly sleeping the full interval on top of it.
            iter_elapsed = time.monotonic() - iter_start
            remaining = UPDATE_INTERVAL_SECONDS - iter_elapsed
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        pass

    log(f"=== Run finished, log saved to {LOG_PATH} ===")
    return False


if __name__ == "__main__":
    if DEBUG_USB:
        print(f"Logging to: {LOG_PATH}")

    
    keepGoing = True
    try:
        while keepGoing:
            keepGoing = main()
    finally:
        stop_macmon()