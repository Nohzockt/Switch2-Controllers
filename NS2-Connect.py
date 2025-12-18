#!/usr/bin/env python3
"""
NS2-Connect (Python) v1.8.2
With GUI, Remapping, Dynamic UI
+ Dynamic window autosizing
+ Adjust Trigger Deadzone/Threshold for GameCube controllers in GUI
+ Remap GL/GR buttons for Pro Controllers and Joy-Cons in GUI
Connect Nintendo Switch Pro Controllers, Joy-Cons, and GameCube Controllers via Bluetooth
Joy-Cons are only supported as single controllers.
"""

import asyncio
import threading
import sys
import os
import argparse
import time
import queue
import traceback
from enum import IntEnum

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox

try:
    from bleak import BleakScanner, BleakClient
    import vgamepad as vg
except ImportError as e:
    print(f"CRITICAL ERROR: Missing libraries. {e}")
    input("Press Enter to exit...")
    sys.exit(1)

# --- Global Vars ---
gamepad = None
gui_output_queue = queue.Queue()
status_update_queue = queue.Queue()
layout_update_queue = queue.Queue()

# --- Configuration Globals ---
TRIGGER_DEADZONE_L = 35
TRIGGER_DEADZONE_R = 35
TRIGGER_THRESHOLD_L = 209
TRIGGER_THRESHOLD_R = 209

gl_map_target = "None"
gr_map_target = "None"

MAPPING_OPTIONS = sorted([
    "A", "B", "X", "Y",
    "L", "R", "ZL", "ZR",
    "Plus", "Minus", "Home",
    "DPad-Up", "DPad-Down", "DPad-Left", "DPad-Right",
    "LStick", "RStick"
])

# Nintendo Switch Controller IDs
VENDOR_ID = 0x057E
PRODUCT_ID_PRO = 0x2009
PRODUCT_ID_L = 0x2006
PRODUCT_ID_R = 0x2007
PRODUCT_ID_GC = 0x2073
rumble_counter = 0

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


GC_BUTTON_MAP = {
    SW2.A: "A", SW2.B: "B", SW2.X: "X", SW2.Y: "Y",
    SW2.PLUS: "Start", SW2.C: "C", SW2.HOME: "Home", SW2.CAPTURE: "Capture",
    SW2.ZL: "ZL", SW2.ZR: "Z", SW2.UP: "DPad-Up", SW2.DOWN: "DPad-Down",
    SW2.LEFT: "DPad-Left", SW2.RIGHT: "DPad-Right",
}

XBOX_BUTTON_MAP = {
    "A": vg.XUSB_BUTTON.XUSB_GAMEPAD_A, "B": vg.XUSB_BUTTON.XUSB_GAMEPAD_B,
    "X": vg.XUSB_BUTTON.XUSB_GAMEPAD_X, "Y": vg.XUSB_BUTTON.XUSB_GAMEPAD_Y,
    "Start": vg.XUSB_BUTTON.XUSB_GAMEPAD_START, "Z": vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_SHOULDER,
    "ZL": vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_SHOULDER, "DPad-Up": vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_UP,
    "DPad-Down": vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_DOWN, "DPad-Left": vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_LEFT,
    "DPad-Right": vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_RIGHT,
    "Plus": vg.XUSB_BUTTON.XUSB_GAMEPAD_START, "Minus": vg.XUSB_BUTTON.XUSB_GAMEPAD_BACK,
    "Home": vg.XUSB_BUTTON.XUSB_GAMEPAD_GUIDE, "L": vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_SHOULDER,
    "R": vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_SHOULDER, "LStick": vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_THUMB,
    "RStick": vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_THUMB,
}

SWITCH_BUTTON_MAP = {
    SW2.A: "A", SW2.B: "B", SW2.X: "X", SW2.Y: "Y",
    SW2.PLUS: "Plus", SW2.MINUS: "Minus", SW2.C: "C", SW2.HOME: "Home",
    SW2.CAPTURE: "Capture", SW2.ZL: "ZL", SW2.L: "L", SW2.ZR: "ZR", SW2.R: "R",
    SW2.R_SR: "R-SR", SW2.R_SL: "R-SL", SW2.L_SR: "L-SR", SW2.L_SL: "L-SL",
    SW2.UP: "DPad-Up", SW2.DOWN: "DPad-Down", SW2.LEFT: "DPad-Left", SW2.RIGHT: "DPad-Right",
    SW2.LJ: "LStick", SW2.RJ: "RStick", SW2.GR: "GR", SW2.GL: "GL",
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

default_calibration = {
    "LX": {"min": 746, "center": 1998, "max": 3249, "deadzone": 90},
    "LY": {"min": 746, "center": 1998, "max": 3249, "deadzone": 90},
    "RX": {"min": 746, "center": 1998, "max": 3249, "deadzone": 300},
    "RY": {"min": 746, "center": 1998, "max": 3249, "deadzone": 300},
}
controller_calibration = default_calibration.copy()
current_ble_client = None
rumble_event_loop = None
rumble_active = False


def gui_print(msg):
    gui_output_queue.put(str(msg))


def update_status(msg):
    status_update_queue.put(str(msg))


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
    if name and any(k in name for k in ["nintendo", "pro controller", "joy-con", "joy con", "joycon", "switch"]):
        nintendo_device_info[device.address] = {'vendor_id': VENDOR_ID, 'product_id': PRODUCT_ID_PRO, 'name': device.name}
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
            nintendo_device_info[device.address] = {'vendor_id': nintendo_info[0], 'product_id': pid, 'name': device.name}
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


def update_xbox_gamepad(pressed_buttons, L, R, LX, LY, RX, RY):
    if gamepad is None:
        return
    gamepad.reset()
    for btn in pressed_buttons:
        xb_btn = XBOX_BUTTON_MAP.get(btn)
        if xb_btn:
            gamepad.press_button(xb_btn)
    gamepad.left_trigger(value=L)
    gamepad.right_trigger(value=R)
    gamepad.left_joystick(x_value=LX, y_value=LY)
    gamepad.right_joystick(x_value=RX, y_value=RY)
    gamepad.update()


def parse_calibration_from_report(data):
    if len(data) < 28:
        return None
    try:
        lx = int.from_bytes(data[12:14], "little")
        ly = int.from_bytes(data[14:16], "little")
        rx = int.from_bytes(data[21:23], "little")
        ry = int.from_bytes(data[23:25], "little")
        return {
            "LX": {"min": max(lx - 900, 0), "center": lx, "max": lx + 900, "deadzone": 300},
            "LY": {"min": max(ly - 900, 0), "center": ly, "max": ly + 900, "deadzone": 300},
            "RX": {"min": max(rx - 900, 0), "center": rx, "max": rx + 900, "deadzone": 300},
            "RY": {"min": max(ry - 900, 0), "center": ry, "max": ry + 900, "deadzone": 300},
        }
    except Exception:
        return None


def is_calibration_spi_reply(data):
    if len(data) < 16:
        return False
    spi_offset = int.from_bytes(data[8:12], "little")
    spi_len = int.from_bytes(data[6:8], "little")
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
    global controller_state, last_raw_data, controller_calibration, gl_map_target, gr_map_target
    if not data or len(data) < 10:
        return
    last_raw_data = data

    if len(data) >= 32 and data[0] == 0x21 and data[1] == 0x01 and is_calibration_spi_reply(data):
        parsed = parse_calibration_from_report(data)
        if parsed:
            controller_calibration = parsed

    pid = controller_state.get('product_id', PRODUCT_ID_PRO) if controller_state else PRODUCT_ID_PRO
    button_data = int.from_bytes(data[4:8], byteorder='little') if len(data) >= 8 else 0

    axes = [0, 0, 0, 0, 0, 0]
    if len(data) >= 16:
        axes_data = data[10:16]
        axes[0] = normalize_axis(axes_data[0] | ((axes_data[1] & 0xF) << 8), "LX")
        axes[1] = normalize_axis((axes_data[1] >> 4) | (axes_data[2] << 4), "LY")
        axes[2] = normalize_axis(axes_data[3] | ((axes_data[4] & 0xF) << 8), "RX")
        axes[3] = normalize_axis((axes_data[4] >> 4) | (axes_data[5] << 4), "RY")

    if pid == PRODUCT_ID_GC:
        left_trigger, right_trigger = extract_gc_triggers(data)
        axes[4] = left_trigger
        axes[5] = right_trigger
        pressed = get_pressed_buttons_gc(button_data)
        update_xbox_gamepad(pressed, axes[4], axes[5], axes[0], axes[1], axes[2], axes[3])
        btns_display = ", ".join(pressed) if pressed else "none"
        trigger_display = f" | L:{axes[4]:3d} R:{axes[5]:3d}"
        axes_display = f"LX:{axes[0]:3d} LY:{axes[1]:3d} RX:{axes[2]:3d} RY:{axes[3]:3d}"
        update_status(f"[GC] Btn: {btns_display:<20} | {axes_display} {trigger_display}")
    else:
        pressed = get_pressed_buttons_switch(button_data)

        # Remap GL/GR (only if present)
        if "GL" in pressed:
            pressed.remove("GL")
            if gl_map_target and gl_map_target != "None":
                pressed.append(gl_map_target)
        if "GR" in pressed:
            pressed.remove("GR")
            if gr_map_target and gr_map_target != "None":
                pressed.append(gr_map_target)

        l_trig = 0
        r_trig = 0
        if "ZL" in pressed:
            l_trig = 255
            if "ZL" in pressed:
                pressed.remove("ZL")
        if "ZR" in pressed:
            r_trig = 255
            if "ZR" in pressed:
                pressed.remove("ZR")

        update_xbox_gamepad(pressed, l_trig, r_trig, axes[0], axes[1], axes[2], axes[3])
        btns_display = ", ".join(pressed) if pressed else "none"
        axes_display = f"LX:{axes[0]:3d} LY:{axes[1]:3d} RX:{axes[2]:3d} RY:{axes[3]:3d}"
        update_status(f"[SW] Btn: {btns_display:<20} | {axes_display}")


async def send_command(client, command, retry=3):
    global output_characteristic
    if not output_characteristic:
        return False
    try:
        if verbose_mode:
            gui_print(f"[VERBOSE] Send: {command.hex(' ')}")
        await client.write_gatt_char(output_characteristic, command)
        return True
    except Exception as e:
        if retry > 0:
            if debug_mode:
                gui_print(f"[DEBUG] send_command error: {e}")
            await asyncio.sleep(0.1)
            return await send_command(client, command, retry - 1)
        return False


async def set_player_leds(client, player_num=1):
    if player_num < 1 or player_num > 8:
        player_num = 1
    led_value = BT_HID_LED_DEV_ID_MAP[player_num - 1]
    led_cmd = bytearray(16)
    led_cmd[0] = 0x30
    led_cmd[1] = 0x01
    led_cmd[2] = 0x00
    led_cmd[3] = 0x30
    led_cmd[5] = 0x08
    led_cmd[8] = led_value
    await client.write_gatt_char(output_characteristic, led_cmd, response=True)
    await asyncio.sleep(0.1)


async def set_rumble(client, on=True):
    global rumble_counter
    rumble_cmd = bytearray([
        0x50,
        0x50 | (rumble_counter & 0x0F),
        0x01 if on else 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00
    ])
    rumble_counter = (rumble_counter + 1) & 0x0F
    return await send_command(client, rumble_cmd)


def async_rumble_handler(large_motor, small_motor):
    global current_ble_client, rumble_event_loop, rumble_active
    if not current_ble_client or not rumble_event_loop or rumble_active:
        return
    if large_motor > 0 or small_motor > 0:
        def run_rumble():
            try:
                future = asyncio.run_coroutine_threadsafe(perform_rumble_sequence(), rumble_event_loop)
                future.result(timeout=2.0)
            except Exception:
                pass
        threading.Thread(target=run_rumble, daemon=True).start()


async def perform_rumble_sequence():
    global current_ble_client, rumble_active
    if rumble_active:
        return
    rumble_active = True
    try:
        if current_ble_client:
            await set_rumble(current_ble_client, True)
            await asyncio.sleep(0.2)
            await set_rumble(current_ble_client, False)
    finally:
        rumble_active = False


def vgamepad_notification_callback(client, target, large_motor, small_motor, led_number, user_data):
    async_rumble_handler(large_motor, small_motor)


async def find_characteristics(client):
    global output_characteristic, input_characteristic
    try:
        services = await client.get_services()

        # Reset
        output_characteristic = None
        input_characteristic = None

        for service in services:
            if service.uuid.lower() == NINTENDO_SERVICE_UUID.lower():
                for char in service.characteristics:
                    if "notify" in char.properties and not input_characteristic:
                        input_characteristic = char.uuid
                    if ("write" in char.properties or "write-without-response" in char.properties) and not output_characteristic:
                        output_characteristic = char.uuid

        if not input_characteristic or not output_characteristic:
            for service in services:
                if service.uuid.lower() == HID_SERVICE_UUID.lower():
                    for char in service.characteristics:
                        if "notify" in char.properties and not input_characteristic:
                            input_characteristic = char.uuid
                        if ("write" in char.properties or "write-without-response" in char.properties) and not output_characteristic:
                            output_characteristic = char.uuid

        return input_characteristic is not None and output_characteristic is not None
    except Exception as e:
        if debug_mode:
            gui_print(f"[DEBUG] find_characteristics error: {e}")
        return False


async def send_spi_read_calibration(client):
    spi_read = bytearray([
        0x21, 0x01, 0x00, 0x10,
        0x00, 0x18, 0x00, 0x00,
        0xd0, 0x3d, 0x06, 0x00,
        0x00, 0x00, 0x00, 0x00
    ])
    await send_command(client, spi_read)


async def initialize_controller(client, device):
    global current_state, controller_state, current_ble_client, rumble_event_loop

    controller_info = nintendo_device_info.get(device.address, {})
    pid = controller_info.get('product_id', PRODUCT_ID_PRO)

    controller_state = {'product_id': pid, 'connected': True}
    current_ble_client = client
    rumble_event_loop = asyncio.get_event_loop()

    # Layout Update based on PID
    if pid == PRODUCT_ID_GC:
        layout_update_queue.put("GC")
    else:
        layout_update_queue.put("PRO")

    if not await find_characteristics(client):
        gui_print("❌ Could not find suitable characteristics.")
        return False

    gui_print("⏳ Initializing controller...")
    current_state = ControllerState.DONE

    await client.start_notify(input_characteristic, notification_callback)
    await send_spi_read_calibration(client)

    try:
        if gamepad is not None:
            gamepad.register_notification(callback_function=vgamepad_notification_callback)
            gui_print("🎮 Rumble registered.")
    except Exception:
        gui_print("⚠️ Manual rumble only.")

    gui_print(f"✅ Controller successfully initialized! ({get_nintendo_device_name(device)})")
    return True


# --- GUI Class ---
class NS2ControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("NS2 Connect")

        # Dynamic sizing: start with reasonable min size, but allow autosize
        self.root.minsize(850, 520)
        self.root.resizable(False, False)

        # Icon
        try:
            if hasattr(sys, '_MEIPASS'):
                icon_path = os.path.join(sys._MEIPASS, "ns2icon.ico")
            else:
                icon_path = "ns2icon.ico"
            self.root.iconbitmap(icon_path)
        except Exception:
            pass

        # Live status
        status_frame = tk.Frame(root)
        status_frame.pack(pady=3, fill="x")
        self.status_var = tk.StringVar(value="No data received yet.")
        self.status_label = ttk.Label(status_frame, textvariable=self.status_var, font=("Consolas", 11), anchor="w")
        self.status_label.pack(fill="x", padx=6)

        # Output log
        self.output = scrolledtext.ScrolledText(root, wrap=tk.WORD, height=20, width=105, state='disabled')
        self.output.pack(padx=8, pady=8, fill="both", expand=True)

        # Buttons row 1
        btn_frame = tk.Frame(root)
        btn_frame.pack(pady=5)
        self.scan_btn = ttk.Button(btn_frame, text="Scan & Connect", command=self.scan_and_connect)
        self.scan_btn.grid(row=0, column=0, padx=5)
        self.rumble_btn = ttk.Button(btn_frame, text="Rumble Test", command=self.rumble_test, state='disabled')
        self.rumble_btn.grid(row=0, column=1, padx=5)
        self.led_var = tk.IntVar(value=1)
        tk.Spinbox(btn_frame, from_=1, to=8, width=3, textvariable=self.led_var, state='readonly').grid(row=0, column=2, padx=5)
        self.led_btn = ttk.Button(btn_frame, text="Set LED", command=self.set_led, state='disabled')
        self.led_btn.grid(row=0, column=3, padx=5)

        # Buttons row 2
        btn_frame2 = tk.Frame(root)
        btn_frame2.pack(pady=3)

        self.calib_btn = ttk.Button(btn_frame2, text="Show Calibration", command=self.show_calibration)
        self.calib_btn.pack(side=tk.LEFT, padx=5)

        self.raw_btn = ttk.Button(btn_frame2, text="Show Raw Data", command=self.show_raw_data)
        self.raw_btn.pack(side=tk.LEFT, padx=5)

        self.debug_var = tk.BooleanVar(value=False)
        self.debug_chk = ttk.Checkbutton(btn_frame2, text="Debug", variable=self.debug_var, command=self.toggle_debug)
        self.debug_chk.pack(side=tk.LEFT, padx=10)

        self.verbose_var = tk.BooleanVar(value=False)
        self.verbose_chk = ttk.Checkbutton(btn_frame2, text="Verbose", variable=self.verbose_var, command=self.toggle_verbose)
        self.verbose_chk.pack(side=tk.LEFT, padx=10)

        # --- DYNAMIC FRAMES ---
        self.dynamic_frame_container = tk.Frame(root)
        self.dynamic_frame_container.pack(pady=5, fill="x")

        # 1) PRO Controller Options (GL/GR)
        self.pro_options_frame = tk.LabelFrame(self.dynamic_frame_container, text="Pro Controller / Joy-Con Options")

        pro_center = tk.Frame(self.pro_options_frame)
        pro_center.pack(fill="x")

        # 3 Columns: left Spacer, middle Inhalt, right Spacer
        pro_center.grid_columnconfigure(0, weight=1)
        pro_center.grid_columnconfigure(1, weight=0)
        pro_center.grid_columnconfigure(2, weight=1)

        pro_inner = tk.Frame(pro_center)
        pro_inner.grid(row=0, column=1, pady=5)  # mittig

        tk.Label(pro_inner, text="GL Map:").grid(row=0, column=0, padx=(0, 5))
        self.gl_combo = ttk.Combobox(pro_inner, values=["None"] + MAPPING_OPTIONS, width=10, state="readonly")
        self.gl_combo.set("None")
        self.gl_combo.grid(row=0, column=1, padx=(0, 15))
        self.gl_combo.bind("<<ComboboxSelected>>", self.update_mappings)

        tk.Label(pro_inner, text="GR Map:").grid(row=0, column=2, padx=(0, 5))
        self.gr_combo = ttk.Combobox(pro_inner, values=["None"] + MAPPING_OPTIONS, width=10, state="readonly")
        self.gr_combo.set("None")
        self.gr_combo.grid(row=0, column=3)
        self.gr_combo.bind("<<ComboboxSelected>>", self.update_mappings)

        # 2) GameCube Trigger Configuration
        self.gc_options_frame = tk.LabelFrame(self.dynamic_frame_container, text="GameCube Trigger Configuration")

        gc_center = tk.Frame(self.gc_options_frame)
        gc_center.pack(fill="x")

        gc_center.grid_columnconfigure(0, weight=1)
        gc_center.grid_columnconfigure(1, weight=0)
        gc_center.grid_columnconfigure(2, weight=1)

        gc_inner = tk.Frame(gc_center)
        gc_inner.grid(row=0, column=1, pady=5)  # mittig

        # Left (L-Trigger)
        l_frame = tk.Frame(gc_inner)
        l_frame.grid(row=0, column=0, padx=15)

        tk.Label(l_frame, text="L-Deadzone:").grid(row=0, column=0, sticky="e", padx=5)
        self.l_dz_scale = tk.Scale(l_frame, from_=0, to=255, orient=tk.HORIZONTAL, command=self.update_triggers,
                                   length=180)
        self.l_dz_scale.set(TRIGGER_DEADZONE_L)
        self.l_dz_scale.grid(row=0, column=1)

        tk.Label(l_frame, text="L-Threshold:").grid(row=1, column=0, sticky="e", padx=5)
        self.l_th_scale = tk.Scale(l_frame, from_=0, to=255, orient=tk.HORIZONTAL, command=self.update_triggers,
                                   length=180)
        self.l_th_scale.set(TRIGGER_THRESHOLD_L)
        self.l_th_scale.grid(row=1, column=1)

        # Right (R-Trigger)
        r_frame = tk.Frame(gc_inner)
        r_frame.grid(row=0, column=1, padx=15)

        tk.Label(r_frame, text="R-Deadzone:").grid(row=0, column=0, sticky="e", padx=5)
        self.r_dz_scale = tk.Scale(r_frame, from_=0, to=255, orient=tk.HORIZONTAL, command=self.update_triggers,
                                   length=180)
        self.r_dz_scale.set(TRIGGER_DEADZONE_R)
        self.r_dz_scale.grid(row=0, column=1)

        tk.Label(r_frame, text="R-Threshold:").grid(row=1, column=0, sticky="e", padx=5)
        self.r_th_scale = tk.Scale(r_frame, from_=0, to=255, orient=tk.HORIZONTAL, command=self.update_triggers,
                                   length=180)
        self.r_th_scale.set(TRIGGER_THRESHOLD_R)
        self.r_th_scale.grid(row=1, column=1)

        # Async Loop
        self.asyncio_loop = asyncio.new_event_loop()
        threading.Thread(target=self.start_asyncio_loop, daemon=True).start()
        self.ble_client = None

        # initial: hide dynamic frames + autosize
        self.pro_options_frame.pack_forget()
        self.gc_options_frame.pack_forget()
        self._autosize_window()

        self.root.after(100, self.update_loops)

    def start_asyncio_loop(self):
        asyncio.set_event_loop(self.asyncio_loop)
        self.asyncio_loop.run_forever()

    def run_async(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self.asyncio_loop)

    def _autosize_window(self):
        self.root.update_idletasks()
        req_w = self.root.winfo_reqwidth()
        req_h = self.root.winfo_reqheight()

        min_w, min_h = 850, 520
        w = max(req_w, min_w)
        h = max(req_h, min_h)

        self.root.geometry(f"{w}x{h}")

    def update_mappings(self, event=None):
        global gl_map_target, gr_map_target
        gl_map_target = self.gl_combo.get()
        gr_map_target = self.gr_combo.get()
        if debug_mode:
            gui_print(f"[DEBUG] Mapping updated: GL={gl_map_target} GR={gr_map_target}")

    def update_triggers(self, event=None):
        global TRIGGER_DEADZONE_L, TRIGGER_DEADZONE_R, TRIGGER_THRESHOLD_L, TRIGGER_THRESHOLD_R
        TRIGGER_DEADZONE_L = self.l_dz_scale.get()
        TRIGGER_DEADZONE_R = self.r_dz_scale.get()
        TRIGGER_THRESHOLD_L = self.l_th_scale.get()
        TRIGGER_THRESHOLD_R = self.r_th_scale.get()

        if debug_mode:
            gui_print(f"[DEBUG] Trigger config: DZ_L={TRIGGER_DEADZONE_L} TH_L={TRIGGER_THRESHOLD_L} "
                      f"DZ_R={TRIGGER_DEADZONE_R} TH_R={TRIGGER_THRESHOLD_R}")

    def show_calibration(self):
        gui_print("Calibration data:")
        gui_print(str(controller_calibration))

    def show_raw_data(self):
        if last_raw_data:
            gui_print("Raw data (last report):")
            gui_print(last_raw_data.hex(" "))
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

    def update_loops(self):
        # 1. Text Output
        while not gui_output_queue.empty():
            line = gui_output_queue.get()
            self.output.config(state='normal')
            self.output.insert(tk.END, line + "\n")
            self.output.see(tk.END)
            self.output.config(state='disabled')

        # 2. Status Bar
        while not status_update_queue.empty():
            self.status_var.set(status_update_queue.get())

        # 3. Dynamic UI Layout
        while not layout_update_queue.empty():
            layout_type = layout_update_queue.get()
            self.pro_options_frame.pack_forget()
            self.gc_options_frame.pack_forget()

            if layout_type == "GC":
                self.gc_options_frame.pack(padx=10, pady=5, fill="x")
                gui_print("-> Layout changed: GameCube Mode")
            elif layout_type == "PRO":
                self.pro_options_frame.pack(padx=10, pady=5, fill="x")
                gui_print("-> Layout changed: Pro Controller Mode")
            elif layout_type == "NONE":
                gui_print("-> Layout cleared")

            self._autosize_window()

        self.root.after(100, self.update_loops)

    def scan_and_connect(self):
        # reset dynamic UI while scanning
        layout_update_queue.put("NONE")

        self.scan_btn.config(state='disabled')
        self.rumble_btn.config(state='disabled')
        self.led_btn.config(state='disabled')
        self.run_async(self.async_scan_and_connect())

    def rumble_test(self):
        if self.ble_client:
            self.run_async(set_rumble(self.ble_client, True))
            self.asyncio_loop.call_later(0.5, lambda: self.run_async(set_rumble(self.ble_client, False)))

    def set_led(self):
        if self.ble_client:
            player_num = self.led_var.get()
            self.run_async(set_player_leds(self.ble_client, player_num))

    async def async_scan_and_connect(self):
        gui_print("\n🔍 Searching for Nintendo Switch controller (5 seconds)...")
        devices = await BleakScanner.discover(timeout=5.0)

        target = None
        for d in devices:
            if is_nintendo_device(d):
                target = d
                gui_print(f"✅ Found: {get_nintendo_device_name(d)}")
                break

        if not target:
            gui_print("❌ No controller found.")
            self.root.after(0, lambda: self.scan_btn.config(state='normal'))
            return

        try:
            async with BleakClient(target) as client:
                if await initialize_controller(client, target):
                    self.ble_client = client
                    self.root.after(0, lambda: [
                        self.rumble_btn.config(state='normal'),
                        self.led_btn.config(state='normal')
                    ])
                    while client.is_connected:
                        await asyncio.sleep(0.5)
                    gui_print("\n🔌 Disconnected.")
                else:
                    gui_print("❌ Initialization failed.")
        except Exception as e:
            gui_print(f"❌ Error: {e}")
            if debug_mode:
                gui_print(traceback.format_exc())
        finally:
            self.ble_client = None
            self.root.after(0, lambda: self.scan_btn.config(state='normal'))
            layout_update_queue.put("NONE")


def tk_main():
    root = tk.Tk()
    app = NS2ControllerGUI(root)
    root.mainloop()


if __name__ == "__main__":
    try:
        try:
            gamepad = vg.VX360Gamepad()
        except Exception as e:
            print(f"ViGEmBus Init Error: {e}\nEnsure ViGEmBus driver is installed.")
            gamepad = None

        tk_main()
    except Exception:
        traceback.print_exc()
        input("Critical Crash. Press Enter to exit...")
