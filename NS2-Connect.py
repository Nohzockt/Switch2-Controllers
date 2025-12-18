#!/usr/bin/env python3
"""
NS2-Connect (Python) v1.8
With GUI
Connect Nintendo Switch Pro Controllers, Joy-Cons, and GameCube Controllers via Bluetooth
Joy-Cons are only supported as single controllers.
"""

import asyncio
import threading
import platform
import signal
import sys
import platform
import argparse
import threading
import time
import queue
import argparse
from enum import IntEnum

import tkinter as tk
from tkinter import ttk, scrolledtext

from bleak import BleakScanner, BleakClient
import vgamepad as vg

# --- Output-Queue for GUI ---
gui_output_queue = queue.Queue()
def gui_print(msg):
    gui_output_queue.put(str(msg))

# --- Status-Queue for Live-Status display ---
status_update_queue = queue.Queue()
def update_status(msg):
    status_update_queue.put(str(msg))

# --- VX360 Gamepad instance ---
gamepad = vg.VX360Gamepad()

# Nintendo Switch Controller IDs
VENDOR_ID = 0x057E
PRODUCT_ID_PRO = 0x2009
PRODUCT_ID_L = 0x2006
PRODUCT_ID_R = 0x2007
PRODUCT_ID_GC = 0x2073
rumble_counter = 0

# --- Trigger deadzone configuration ---
TRIGGER_DEADZONE_L = 35  # Analog trigger deadzone for Left trigger (L) (0-255). Adjust as needed.
TRIGGER_DEADZONE_R = 35  # Analog trigger deadzone for Right trigger (R) (0-255). Adjust as needed.
TRIGGER_THRESHOLD_L = 209   # Minimum value to be considered "pressed" for Left trigger (L)
TRIGGER_THRESHOLD_R = 209   # Minimum value to be considered "pressed" for Right trigger (R)

# UUIDs
HID_SERVICE_UUID = "00001812-0000-1000-8000-00805f9b34fb"
NINTENDO_SERVICE_UUID = "ab7de9be-89fe-49ad-828f-118f09df7fd0"
NINTENDO_INPUT_UUID = "ab7de9be-89fe-49ad-828f-118f09df7fd2"

BT_HID_LED_DEV_ID_MAP = [0x01, 0x02, 0x04, 0x08, 0x03, 0x06, 0x0C, 0x0F]

class ControllerState(IntEnum):
    READ_INFO = 0
    READ_LTK = 1
    SET_BDADDR = 2
    READ_NEW_LTK = 3
    SET_LED = 4
    EN_REPORT = 5
    DONE = 6

class SW2(IntEnum):
    Y = 0
    X = 1
    B = 2
    A = 3
    R_SR = 4
    R_SL = 5
    R = 6
    ZR = 7
    MINUS = 8
    PLUS = 9
    RJ = 10
    LJ = 11
    HOME = 12
    CAPTURE = 13
    C = 14
    UNKNOWN = 15
    DOWN = 16
    UP = 17
    RIGHT = 18
    LEFT = 19
    L_SR = 20
    L_SL = 21
    L = 22
    ZL = 23
    GR = 24
    GL = 25

# GameCube Controller Button Mapping
GC_BUTTON_MAP = {
    SW2.A: "A",
    SW2.B: "B",
    SW2.X: "X",
    SW2.Y: "Y",
    SW2.PLUS: "Start",
    SW2.C: "C",
    SW2.HOME: "Home",
    SW2.CAPTURE: "Capture",
    SW2.ZL: "ZL",
    SW2.ZR: "Z",
    SW2.UP: "DPad-Up",
    SW2.DOWN: "DPad-Down",
    SW2.LEFT: "DPad-Left",
    SW2.RIGHT: "DPad-Right",
}

XBOX_BUTTON_MAP = {
    "A": vg.XUSB_BUTTON.XUSB_GAMEPAD_A,
    "B": vg.XUSB_BUTTON.XUSB_GAMEPAD_B,
    "X": vg.XUSB_BUTTON.XUSB_GAMEPAD_X,
    "Y": vg.XUSB_BUTTON.XUSB_GAMEPAD_Y,
    "Start": vg.XUSB_BUTTON.XUSB_GAMEPAD_START,
    "Z": vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_SHOULDER,
    "ZL": vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_SHOULDER, # Für GameCube Mapping
    "DPad-Up": vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_UP,
    "DPad-Down": vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_DOWN,
    "DPad-Left": vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_LEFT,
    "DPad-Right": vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_RIGHT,
    # --- Neue Mappings für Switch Pro/Joy-Cons ---
    "Plus": vg.XUSB_BUTTON.XUSB_GAMEPAD_START,
    "Minus": vg.XUSB_BUTTON.XUSB_GAMEPAD_BACK,
    "Home": vg.XUSB_BUTTON.XUSB_GAMEPAD_GUIDE,
    "L": vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_SHOULDER,
    "R": vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_SHOULDER,
    "LStick": vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_THUMB,
    "RStick": vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_THUMB,
}

# Switch Pro/Joy-Con Button Mapping
SWITCH_BUTTON_MAP = {
    SW2.A: "A",
    SW2.B: "B",
    SW2.X: "X",
    SW2.Y: "Y",
    SW2.PLUS: "Plus",
    SW2.MINUS: "Minus",
    SW2.C: "C",
    SW2.HOME: "Home",
    SW2.CAPTURE: "Capture",
    SW2.ZL: "ZL",
    SW2.L: "L",
    SW2.ZR: "ZR",
    SW2.R: "R",
    SW2.R_SR: "R-SR",
    SW2.R_SL: "R-SL",
    SW2.L_SR: "L-SR",
    SW2.L_SL: "L-SL",
    SW2.UP: "DPad-Up",
    SW2.DOWN: "DPad-Down",
    SW2.LEFT: "DPad-Left",
    SW2.RIGHT: "DPad-Right",
    SW2.LJ: "LStick",
    SW2.RJ: "RStick",
}

keep_running = True
debug_mode = False
verbose_mode = False
controller_state = None
output_characteristic = None
input_characteristic = None
nintendo_device_info = {}
current_state = ControllerState.READ_INFO
last_raw_data = None

# Calibration Defaults (Fallback, if not found)
default_calibration = {
    "LX": {"min": 746, "center": 1998, "max": 3249, "deadzone": 90},
    "LY": {"min": 746, "center": 1998, "max": 3249, "deadzone": 90},
    "RX": {"min": 746, "center": 1998, "max": 3249, "deadzone": 300},
    "RY": {"min": 746, "center": 1998, "max": 3249, "deadzone": 300},
}
controller_calibration = default_calibration.copy()

# Global reference to the current BLE client for rumble callback
current_ble_client = None
rumble_event_loop = None
rumble_active = False

def extract_nintendo_info(manufacturer_data):
    if not manufacturer_data:
        return None
    for company_id, data in manufacturer_data.items():
        if not data or len(data) < 6:
            continue
        if len(data) >= 5 and data[2] == 0x03 and data[3] == 0x7E:
            vendor_id = 0x057E
            product_id = (data[5] << 8) | data[4] if len(data) > 5 else 0
            return (vendor_id, product_id)
    return None

def is_nintendo_device(device):
    if not device:
        return False
    name = device.name.lower() if device.name else ""
    if name and any(
            keyword in name for keyword in ["nintendo", "pro controller", "joy-con", "joy con", "joycon", "switch"]):
        nintendo_device_info[device.address] = {
            'vendor_id': VENDOR_ID,
            'product_id': PRODUCT_ID_PRO,
            'name': device.name
        }
        return True
    if hasattr(device, "metadata") and device.metadata.get("manufacturer_data"):
        nintendo_info = extract_nintendo_info(device.metadata["manufacturer_data"])
        if nintendo_info:
            pid = nintendo_info[1]
            if pid == 0x7305:
                pid = 0x2073
            elif pid == 0x0920:
                pid = 0x2009
            elif pid == 0x0620:
                pid = 0x2006
            elif pid == 0x0720:
                pid = 0x2007
            nintendo_device_info[device.address] = {
                'vendor_id': nintendo_info[0],
                'product_id': pid,
                'name': device.name
            }
            return True
    return False

def get_nintendo_device_name(device):
    if device.address not in nintendo_device_info:
        return device.name or "Nintendo device"
    info = nintendo_device_info[device.address]
    pid = info.get('product_id', 0)
    if pid == PRODUCT_ID_PRO:
        return "Nintendo Switch Pro Controller"
    elif pid == PRODUCT_ID_L:
        return "Nintendo Switch Joy-Con (L)"
    elif pid == PRODUCT_ID_R:
        return "Nintendo Switch Joy-Con (R)"
    elif pid == PRODUCT_ID_GC:
        return "Nintendo GameCube Controller"
    else:
        return f"Nintendo Controller (PID: 0x{pid:04X})"

def apply_trigger_deadzone_and_threshold(value, deadzone, threshold):
    if abs(value) < deadzone:
        return 0
    if value >= threshold:
        return 255
    return value

def extract_gc_triggers(data):
    left_trigger = 0
    right_trigger = 0
    if len(data) >= 62:
        left_trigger = data[60]
        right_trigger = data[61]
    elif len(data) >= 14:
        left_trigger = data[12]
        right_trigger = data[13]
    if left_trigger == 0 and len(data) >= 5:
        buttons = int.from_bytes(data[4:8], byteorder='little')
        if buttons & (1 << SW2.L):
            left_trigger = 255
    if right_trigger == 0 and len(data) >= 5:
        buttons = int.from_bytes(data[4:8], byteorder='little')
        if buttons & (1 << SW2.R):
            right_trigger = 255
    left_trigger = apply_trigger_deadzone_and_threshold(left_trigger, TRIGGER_DEADZONE_L, TRIGGER_THRESHOLD_L)
    right_trigger = apply_trigger_deadzone_and_threshold(right_trigger, TRIGGER_DEADZONE_R, TRIGGER_THRESHOLD_R)
    return left_trigger, right_trigger

def get_pressed_buttons_switch(button_value):
    return [name for bit, name in SWITCH_BUTTON_MAP.items() if button_value & (1 << bit)]

def get_pressed_buttons_gc(button_value):
    return [name for bit, name in GC_BUTTON_MAP.items() if button_value & (1 << bit)]

def print_raw_bytes(data):
    if not data or len(data) < 16:
        return
    raw_str = " ".join([f"{b:02X}" for b in data[:16]])
    return f"Raw: {raw_str}"

def update_xbox_gamepad(pressed_gc_buttons, L, R, LX, LY, RX, RY):
    gamepad.reset()
    for btn in pressed_gc_buttons:
        xb_btn = XBOX_BUTTON_MAP.get(btn)
        if xb_btn:
            gamepad.press_button(xb_btn)
    gamepad.left_trigger(value=L)
    gamepad.right_trigger(value=R)
    gamepad.left_joystick(x_value=LX, y_value=LY)  # values between -32768 and 32767
    gamepad.right_joystick(x_value=RX, y_value=RY)  # values between -32768 and 32767
    gamepad.update()

def parse_calibration_from_report(data):
    # Parse calibration data from an incoming SPI report
    # data: bytes-like, with calibration starting at offset 12 for typical SPI read response
    # Returns: dict for LX, LY, RX, RY with min, center, max, deadzone
    if len(data) < 28:
        return None
    try:
        # Calibration layout: 9 bytes per stick, left at offset 12, right at 21
        lx = int.from_bytes(data[12:14], "little")
        ly = int.from_bytes(data[14:16], "little")
        rx = int.from_bytes(data[21:23], "little")
        ry = int.from_bytes(data[23:25], "little")
        # BlueRetro-style: min/max/center
        # For demo, center = LX, min = LX-900, max = LX+900
        return {
            "LX": {"min": max(lx-900, 0), "center": lx, "max": lx+900, "deadzone": 300},
            "LY": {"min": max(ly-900, 0), "center": ly, "max": ly+900, "deadzone": 300},
            "RX": {"min": max(rx-900, 0), "center": rx, "max": rx+900, "deadzone": 300},
            "RY": {"min": max(ry-900, 0), "center": ry, "max": ry+900, "deadzone": 300},
        }
    except Exception as e:
        if debug_mode or verbose_mode:
            gui_print(f"[DEBUG] Could not parse calibration from report: {e}")
        return None

def is_calibration_spi_reply(data):
    # SPI calibration reply detection
    if len(data) < 16:
        return False
    spi_offset = int.from_bytes(data[8:12], "little")
    spi_len = int.from_bytes(data[6:8], "little")
    # BlueRetro reads calibration at 0x603D0, len 0x18 (24)
    return spi_offset == 0x603D0 and spi_len == 0x18

def normalize_axis(value, axis_name="LX"):
    calib = controller_calibration.get(axis_name, default_calibration[axis_name])
    min_val = calib["min"]
    center_val = calib["center"]
    max_val = calib["max"]
    deadzone = calib["deadzone"]

    if value < min_val:
        value = min_val
    if value > max_val:
        value = max_val

    if abs(value - center_val) < deadzone:
        return 0

    if value >= center_val:
        return int(((value - center_val) / (max_val - center_val)) * 32767)
    else:
        return int(((value - center_val) / (center_val - min_val)) * 32768)


async def notification_callback(sender, data):
    global controller_state, last_raw_data, controller_calibration
    if not data or len(data) < 10:
        return
    last_raw_data = data
    # Improved calibration SPI report detection
    if len(data) >= 32 and data[0] == 0x21 and data[1] == 0x01 and is_calibration_spi_reply(data):
        parsed = parse_calibration_from_report(data)
        if parsed:
            controller_calibration = parsed
            if verbose_mode or debug_mode:
                gui_print(f"[INFO] Calibration data loaded: {controller_calibration}")
    pid = controller_state.get('product_id', PRODUCT_ID_PRO) if controller_state else PRODUCT_ID_PRO
    if len(data) >= 8:
        button_data = int.from_bytes(data[4:8], byteorder='little')
    else:
        button_data = 0
    axes = [0, 0, 0, 0, 0, 0]
    if len(data) >= 16:
        axes_data = data[10:16]
        axes[0] = normalize_axis(axes_data[0] | ((axes_data[1] & 0xF) << 8), "LX")  # LX
        axes[1] = normalize_axis((axes_data[1] >> 4) | (axes_data[2] << 4), "LY")  # LY
        axes[2] = normalize_axis(axes_data[3] | ((axes_data[4] & 0xF) << 8), "RX")  # RX
        axes[3] = normalize_axis((axes_data[4] >> 4) | (axes_data[5] << 4), "RY")  # RY

    if pid == PRODUCT_ID_GC:
        left_trigger, right_trigger = extract_gc_triggers(data)
        axes[4] = left_trigger
        axes[5] = right_trigger
        pressed = get_pressed_buttons_gc(button_data)
        update_xbox_gamepad(pressed, axes[4], axes[5], axes[0], axes[1], axes[2], axes[3])

        btns_display = ", ".join(pressed) if pressed else "none"
        trigger_display = f" | L:{axes[4]:3d} R:{axes[5]:3d}"
        axes_display = f"LX:{axes[0]:3d} LY:{axes[1]:3d} RX:{axes[2]:3d} RY:{axes[3]:3d}"
        status_str = f"[GC] Buttons: {btns_display:<30} | Sticks: {axes_display} {trigger_display}"
        update_status(status_str)
    else:
        # --- Logic for Switch Pro / Joy-Cons ---
        pressed = get_pressed_buttons_switch(button_data)

        # Handle Digital Triggers (ZL/ZR) as Analog XBox Triggers
        l_trig = 0
        r_trig = 0

        # Wenn ZL gedrückt ist -> Full Left Trigger
        if "ZL" in pressed:
            l_trig = 255
            # Entfernen, damit es nicht als Button (Left Shoulder) doppelt gemappt wird,
            # falls "ZL" in der Map vorhanden ist.
            if "ZL" in pressed: pressed.remove("ZL")

        # Wenn ZR gedrückt ist -> Full Right Trigger
        if "ZR" in pressed:
            r_trig = 255
            if "ZR" in pressed: pressed.remove("ZR")

        # Update des virtuellen Xbox Controllers
        update_xbox_gamepad(pressed, l_trig, r_trig, axes[0], axes[1], axes[2], axes[3])

        btns_display = ", ".join(pressed) if pressed else "none"
        axes_display = f"LX:{axes[0]:3d} LY:{axes[1]:3d} RX:{axes[2]:3d} RY:{axes[3]:3d}"
        status_str = f"[SW] Buttons: {btns_display:<30} | Axes: {axes_display}"
        update_status(status_str)

async def send_command(client, command, retry=3):
    global output_characteristic
    if not output_characteristic:
        if debug_mode:
            gui_print("[DEBUG] No output characteristic found!")
        return False
    try:
        if verbose_mode:
            gui_print(f"[VERBOSE] Sending command: {command.hex(' ')}")
        await client.write_gatt_char(output_characteristic, command)
        return True
    except Exception as e:
        if retry > 0:
            if debug_mode:
                gui_print(f"[DEBUG] Error sending (attempt {4 - retry}/3): {e}")
            await asyncio.sleep(0.1)
            return await send_command(client, command, retry - 1)
        else:
            if debug_mode:
                gui_print(f"[DEBUG] Sending failed after 3 attempts: {e}")
            return False

async def set_player_leds(client, player_num=1):
    if player_num < 1 or player_num > 8:
        player_num = 1
    led_value = BT_HID_LED_DEV_ID_MAP[player_num - 1]
    led_cmd = bytearray(16)
    led_cmd[0] = 0x30  # SET_LED
    led_cmd[1] = 0x01  # REQ_TYPE_REQ
    led_cmd[2] = 0x00  # INT_BLE
    led_cmd[3] = 0x30  # SUBCMD_SET_LED
    led_cmd[4] = 0x00
    led_cmd[5] = 0x08
    led_cmd[6] = 0x00
    led_cmd[7] = 0x00
    led_cmd[8] = led_value
    await client.write_gatt_char(output_characteristic, led_cmd, response=True)
    await asyncio.sleep(0.1)

async def set_rumble(client, on=True):
    global rumble_counter

    # NS2 rumble format based on BlueRetro developer's specification
    rumble_cmd = bytearray([
        0x50,  # out[0] - NS2 rumble command identifier
        0x50 | (rumble_counter & 0x0F),  # out[1] - 4 MSB set to 5 (0x50), 4 LSB are counter
        0x01 if on else 0x00,  # out[2] - rumble state: 0x01 = on, 0x00 = off
        0x00, 0x00, 0x00, 0x00, 0x00,  # Padding to match expected packet size
    ])

    # Increment counter for next rumble command (wraps around 0-15)
    rumble_counter = (rumble_counter + 1) & 0x0F
    if verbose_mode:
        gui_print(
            f"[VERBOSE] Sending NS2 rumble command: {rumble_cmd.hex(' ')} (counter: {rumble_counter - 1 & 0x0F}, state: {'ON' if on else 'OFF'})")
    return await send_command(client, rumble_cmd)

async def dump_raw_data():
    global last_raw_data
    if last_raw_data:
        gui_print("\nRaw data of the last report:")
        gui_print("---------------------------------")
        for i in range(0, len(last_raw_data), 8):
            group = last_raw_data[i:i + 8]
            hex_values = " ".join([f"{b:02X}" for b in group])
            ascii_values = "".join([chr(b) if 32 <= b <= 126 else "." for b in group])
            gui_print(f"{i:04X}: {hex_values:<24} | {ascii_values}")
        gui_print("")
    else:
        gui_print("No raw data yet.")

def async_rumble_handler(large_motor, small_motor):
    """Handle rumble in async context"""
    global current_ble_client, rumble_event_loop
    if not current_ble_client or not rumble_event_loop:
        if debug_mode:
            gui_print("[DEBUG] No BLE client or event loop available for rumble")
        return
    if rumble_active:
        if debug_mode:
            gui_print("[DEBUG] Rumble active, skipping.")
        return
    if large_motor > 0 or small_motor > 0:
        # Schedule the coroutine in the event loop running the BLE client
        def run_rumble():
            try:
                future = asyncio.run_coroutine_threadsafe(perform_rumble_sequence(), rumble_event_loop)
                future.result(timeout=2.0)
            except Exception as e:
                if debug_mode:
                    gui_print(f"[DEBUG] Error scheduling rumble: {e}")
        # Run in a separate thread to avoid blocking
        threading.Thread(target=run_rumble, daemon=True).start()

async def perform_rumble_sequence():
    global current_ble_client, rumble_active
    if rumble_active:
        return
    rumble_active = True
    try:
        if current_ble_client:
            if debug_mode:
                gui_print("[DEBUG] Starting rumble sequence")
            await set_rumble(current_ble_client, True)
            await asyncio.sleep(0.2)  # Brief rumble duration
            await set_rumble(current_ble_client, False)
            if debug_mode:
                gui_print("[DEBUG] Rumble sequence completed")
    except Exception as e:
        if debug_mode:
            gui_print(f"[DEBUG] Error in rumble sequence: {e}")
    finally:
        rumble_active = False

def vgamepad_notification_callback(client, target, large_motor, small_motor, led_number, user_data):
    """
    Synchronous callback for vgamepad notifications.
    This schedules async rumble handling.
    """
    async_rumble_handler(large_motor, small_motor)

def setup_vgamepad_callback():
    """Setup the vgamepad notification callback"""
    try:
        gamepad.register_notification(callback_function=vgamepad_notification_callback)
        if debug_mode:
            gui_print("[DEBUG] vgamepad notification callback registered successfully")
        return True
    except Exception as e:
        if debug_mode:
            gui_print(f"[DEBUG] Failed to register vgamepad callback: {e}")
        return False

async def find_characteristics(client):
    global output_characteristic, input_characteristic
    try:
        services = await client.get_services()
        for service in services:
            if service.uuid.lower() == NINTENDO_SERVICE_UUID.lower():
                for char in service.characteristics:
                    props = char.properties
                    if "notify" in props and not input_characteristic:
                        input_characteristic = char.uuid
                    if ("write-without-response" in props or "write" in props) and not output_characteristic:
                        output_characteristic = char.uuid
        if not input_characteristic or not output_characteristic:
            for service in services:
                if service.uuid.lower() == HID_SERVICE_UUID.lower():
                    for char in service.characteristics:
                        props = char.properties
                        if "notify" in props and not input_characteristic:
                            input_characteristic = char.uuid
                        if ("write" in props or "write-without-response" in props) and not output_characteristic:
                            output_characteristic = char.uuid
        return input_characteristic is not None and output_characteristic is not None
    except Exception as e:
        if debug_mode:
            gui_print(f"[DEBUG] Error finding characteristics: {e}")
        return False

async def send_spi_read_calibration(client):
    # BlueRetro default: Read calibration at 0x603D0 for left stick
    # 0x603D0 (offset), 0x18 (24 bytes, covers both sticks)
    spi_read = bytearray([
        0x21, 0x01, 0x00, 0x10,
        0x00, 0x18, 0x00, 0x00,
        0xd0, 0x3d, 0x06, 0x00,
        0x00, 0x00, 0x00, 0x00
    ])
    await send_command(client, spi_read)

async def initialize_controller(client, device):
    global current_state, controller_state, input_characteristic, output_characteristic, current_ble_client, rumble_event_loop, controller_calibration
    controller_info = nintendo_device_info.get(device.address, {})
    pid = controller_info.get('product_id', PRODUCT_ID_PRO)
    controller_state = {'product_id': pid, 'connected': True}
    current_ble_client = client  # Store reference for rumble callback
    rumble_event_loop = asyncio.get_event_loop()  # Store current event loop

    if not await find_characteristics(client):
        gui_print("❌ Could not find suitable characteristics.")
        return False
    gui_print("⏳ Initializing controller...")
    current_state = ControllerState.DONE
    await client.start_notify(input_characteristic, notification_callback)

    # Calibration request (if not already included in first report)
    await send_spi_read_calibration(client)
    callback_success = setup_vgamepad_callback()
    gui_print(f"✅ Controller successfully initialized! ({get_nintendo_device_name(device)})")
    if callback_success:
        gui_print("🎮 Rumble callback registered - games should be able to rumble the controller!")
    else:
        gui_print("⚠️ Rumble callback registration failed - manual rumble only")
    gui_print("\n📊 Receiving controller data...")
    gui_print("📍 Move sticks and press buttons to see the data...")
    return True

# --- GUI Class ---
class NS2ControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("NS2-Connect")
        self.root.geometry("800x600")
        self.root.resizable(False, False)

        import sys
        import os

        # Prüfen, ob das Programm als PyInstaller-Bundle ausgeführt wird
        if hasattr(sys, '_MEIPASS'):
            # Datei liegt im temporären Verzeichnis des PyInstaller-Bundles
            icon_path = os.path.join(sys._MEIPASS, "ns2icon.ico")
        else:
            # Datei liegt normal im Skriptverzeichnis
            icon_path = "ns2icon.ico"

        self.root.iconbitmap(icon_path)

        # Live status display at the top
        status_frame = tk.Frame(root)
        status_frame.pack(pady=3)
        self.status_var = tk.StringVar(value="No data received yet.")
        self.status_label = ttk.Label(status_frame, textvariable=self.status_var, font=("Consolas", 11), anchor="w")
        self.status_label.pack(fill="x", padx=6)

        # Output log
        self.output = scrolledtext.ScrolledText(root, wrap=tk.WORD, height=24, width=100, state='disabled')
        self.output.pack(padx=8, pady=8)

        btn_frame = tk.Frame(root)
        btn_frame.pack(pady=5)

        self.scan_btn = ttk.Button(btn_frame, text="Scan & Connect Controller", command=self.scan_and_connect)
        self.scan_btn.grid(row=0, column=0, padx=5)
        self.rumble_btn = ttk.Button(btn_frame, text="Rumble Test", command=self.rumble_test, state='disabled')
        self.rumble_btn.grid(row=0, column=1, padx=5)
        self.led_var = tk.IntVar(value=1)
        self.led_spin = tk.Spinbox(btn_frame, from_=1, to=8, width=3, textvariable=self.led_var, state='readonly')
        self.led_spin.grid(row=0, column=2, padx=5)
        self.led_btn = ttk.Button(btn_frame, text="Set LED", command=self.set_led, state='disabled')
        self.led_btn.grid(row=0, column=3, padx=5)
        self.calib_btn = ttk.Button(btn_frame, text="Show Calibration", command=self.show_calib, state='disabled')
        self.calib_btn.grid(row=0, column=4, padx=5)
        self.raw_btn = ttk.Button(btn_frame, text="Show Raw Data", command=self.show_raw, state='disabled')
        self.raw_btn.grid(row=0, column=5, padx=5)
        self.debug_var = tk.BooleanVar()
        self.debug_chk = ttk.Checkbutton(btn_frame, text='Debug', variable=self.debug_var, command=self.toggle_debug)
        self.debug_chk.grid(row=0, column=6, padx=8)
        self.verbose_var = tk.BooleanVar()
        self.verbose_chk = ttk.Checkbutton(btn_frame, text='Verbose', variable=self.verbose_var, command=self.toggle_verbose)
        self.verbose_chk.grid(row=0, column=7, padx=8)

        self.asyncio_loop = asyncio.new_event_loop()
        self.asyncio_thread = threading.Thread(target=self.start_asyncio_loop, daemon=True)
        self.asyncio_thread.start()

        self.connected_device = None
        self.ble_client = None

        self.root.after(100, self.update_output)

    def start_asyncio_loop(self):
        asyncio.set_event_loop(self.asyncio_loop)
        self.asyncio_loop.run_forever()

    def run_async(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self.asyncio_loop)

    def update_output(self):
        while not gui_output_queue.empty():
            line = gui_output_queue.get()
            self.output.config(state='normal')
            self.output.insert(tk.END, line + "\n")
            self.output.see(tk.END)
            self.output.config(state='disabled')
        while not status_update_queue.empty():
            status = status_update_queue.get()
            self.status_var.set(status)
        self.root.after(100, self.update_output)

    def scan_and_connect(self):
        self.scan_btn.config(state='disabled')
        self.rumble_btn.config(state='disabled')
        self.led_btn.config(state='disabled')
        self.calib_btn.config(state='disabled')
        self.raw_btn.config(state='disabled')
        self.run_async(self.async_scan_and_connect())

    async def async_scan_and_connect(self):
        gui_print("\n🔍 Searching for Nintendo Switch controller (5 seconds)...")
        devices = await BleakScanner.discover(timeout=5.0)
        nintendo_devices = []
        for device in devices:
            if is_nintendo_device(device):
                nintendo_devices.append(device)
                gui_print(f"✅ Nintendo device found: {get_nintendo_device_name(device)} ({device.address})")
        if not nintendo_devices:
            gui_print("❌ No Nintendo Switch controller found.")
            self.root.after(0, lambda: self.scan_btn.config(state='normal'))
            return
        device = nintendo_devices[0]
        try:
            async with BleakClient(device) as client:
                gui_print(f"🔄 Connecting to {get_nintendo_device_name(device)} ({device.address})...")
                if await initialize_controller(client, device):
                    self.connected_device = device
                    self.ble_client = client
                    self.root.after(0, lambda: [
                        self.rumble_btn.config(state='normal'),
                        self.led_btn.config(state='normal'),
                        self.calib_btn.config(state='normal'),
                        self.raw_btn.config(state='normal')
                    ])
                    while client.is_connected:
                        await asyncio.sleep(0.2)
                    gui_print("\n🔌 Controller disconnected.")
                else:
                    gui_print("❌ Initialization failed.")
        except Exception as e:
            gui_print(f"❌ Connection error: {e}")
        finally:
            self.connected_device = None
            self.ble_client = None
            self.root.after(0, lambda: self.scan_btn.config(state='normal'))

    def rumble_test(self):
        if self.ble_client:
            self.run_async(set_rumble(self.ble_client, True))
            self.asyncio_loop.call_later(0.5, lambda: self.run_async(set_rumble(self.ble_client, False)))
            gui_print("🎮 Rumble test executed.")

    def set_led(self):
        if self.ble_client:
            player_num = self.led_var.get()
            self.run_async(set_player_leds(self.ble_client, player_num))
            gui_print(f"💡 Set player LED to {player_num}.")

    def show_calib(self):
        gui_print("Calibration data:")
        gui_print(str(controller_calibration))

    def show_raw(self):
        if last_raw_data:
            gui_print("\nRaw data of the last report:")
            gui_print("---------------------------------")
            for i in range(0, len(last_raw_data), 8):
                group = last_raw_data[i:i + 8]
                hex_values = " ".join([f"{b:02X}" for b in group])
                ascii_values = "".join([chr(b) if 32 <= b <= 126 else "." for b in group])
                gui_print(f"{i:04X}: {hex_values:<24} | {ascii_values}")
        else:
            gui_print("No raw data received yet.")

    def toggle_debug(self):
        global debug_mode
        debug_mode = self.debug_var.get()
        gui_print(f"Debug mode {'enabled' if debug_mode else 'disabled'}")

    def toggle_verbose(self):
        global verbose_mode
        verbose_mode = self.verbose_var.get()
        gui_print(f"Verbose mode {'enabled' if verbose_mode else 'disabled'}")

def tk_main():
    root = tk.Tk()
    app = NS2ControllerGUI(root)
    root.protocol("WM_DELETE_WINDOW", lambda: sys.exit(0))
    root.mainloop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='NS2 Bluetooth Enabler (GUI)')
    parser.add_argument('-d', '--debug', action='store_true', help='Enable debug output')
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable verbose output')
    args = parser.parse_args()
    debug_mode = args.debug
    verbose_mode = args.verbose

    try:
        tk_main()
    except KeyboardInterrupt:
        gui_print("\n👋 Program exited.")
    except Exception as e:
        gui_print(f"❌ Unexpected error: {e}")
