#!/usr/bin/env python3
"""
NS2 Bluetooth Monitor (Python) + vgamepad Virtual Xbox 360 Gamepad (Windows)
----------------------------------------------------------------------------
Monitors Nintendo Switch Controllers via Bluetooth and exposes them as a virtual Xbox 360 gamepad using vgamepad (ViGEm).
Tested with GameCube Controller on Windows.
Requires: bleak, vgamepad, ViGEmBus driver installed

Run with: python ns2-ble-vgamepad.py
"""

import asyncio
import signal
import sys
import platform
import argparse
from enum import IntEnum
from bleak import BleakScanner, BleakClient

try:
    import vgamepad as vg
except ImportError:
    print("vgamepad is required. Install with 'pip install vgamepad'")
    sys.exit(1)

# Nintendo Switch Controller IDs
VENDOR_ID = 0x057E
PRODUCT_ID_PRO = 0x2009
PRODUCT_ID_L = 0x2006
PRODUCT_ID_R = 0x2007
PRODUCT_ID_GC = 0x2073

# UUIDs
HID_SERVICE_UUID = "00001812-0000-1000-8000-00805f9b34fb"
NINTENDO_SERVICE_UUID = "ab7de9be-89fe-49ad-828f-118f09df7fd0"
NINTENDO_INPUT_UUID = "ab7de9be-89fe-49ad-828f-118f09df7fd2"

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

# Switch to Xbox (vgamepad/ViGEm) button mapping
SWITCH_TO_XBOX_BUTTON = {
    SW2.A: vg.XUSB_BUTTON.XUSB_GAMEPAD_A,
    SW2.B: vg.XUSB_BUTTON.XUSB_GAMEPAD_B,
    SW2.X: vg.XUSB_BUTTON.XUSB_GAMEPAD_X,
    SW2.Y: vg.XUSB_BUTTON.XUSB_GAMEPAD_Y,
    SW2.PLUS: vg.XUSB_BUTTON.XUSB_GAMEPAD_START,
    SW2.MINUS: vg.XUSB_BUTTON.XUSB_GAMEPAD_BACK,
    SW2.HOME: vg.XUSB_BUTTON.XUSB_GAMEPAD_GUIDE,
    SW2.ZL: vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_SHOULDER,
    SW2.ZR: vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_SHOULDER,
    SW2.L: vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_SHOULDER,
    SW2.R: vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_SHOULDER,
    SW2.LJ: vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_THUMB,
    SW2.RJ: vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_THUMB,
    SW2.UP: vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_UP,
    SW2.DOWN: vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_DOWN,
    SW2.LEFT: vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_LEFT,
    SW2.RIGHT: vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_RIGHT,
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
vgamepad_device = None
last_button_states = set()
last_axes = [0, 0, 0, 0, 0, 0]

def handle_signal(signum, frame):
    global keep_running
    print("\nProgram is terminating...")
    keep_running = False

def log_debug(message):
    if debug_mode:
        print(f"[DEBUG] {message}")

def log_verbose(message):
    if verbose_mode:
        print(f"[VERBOSE] {message}")

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
    return left_trigger, right_trigger

def get_pressed_buttons_switch(button_value):
    return [bit for bit in SWITCH_TO_XBOX_BUTTON if button_value & (1 << bit)]

def print_raw_bytes(data):
    if not data or len(data) < 16:
        return
    raw_str = " ".join([f"{b:02X}" for b in data[:16]])
    return f"Raw: {raw_str}"

def create_vgamepad_device():
    gamepad = vg.VX360Gamepad()
    print("✅ Created virtual Xbox 360 gamepad (vgamepad).")
    return gamepad

def parse_stick(data, offset):
    # Nintendo Switch Controller: 12-bit pro Achse, 3 Bytes pro Stick
    # LX = data[offset] | ((data[offset + 1] & 0x0F) << 8)
    # LY = (data[offset + 1] >> 4) | (data[offset + 2] << 4)
    return (
        data[offset] | ((data[offset + 1] & 0x0F) << 8),
        (data[offset + 1] >> 4) | (data[offset + 2] << 4)
    )

def nintendo_to_xbox_axis(val):
    # Mappe 0-4095 (Switch) auf -32768..32767 (Xbox)
    return int((val - 2048) / 2047 * 32767)

def apply_deadzone(axis, deadzone=1200):
    return 0 if abs(axis) < deadzone else axis

async def notification_callback(sender, data):
    global controller_state, last_raw_data, vgamepad_device
    if not data or len(data) < 16 or vgamepad_device is None:
        return
    last_raw_data = data
    pid = controller_state.get('product_id', PRODUCT_ID_PRO) if controller_state else PRODUCT_ID_PRO

    button_data = int.from_bytes(data[4:8], byteorder='little')
    pressed = get_pressed_buttons_switch(button_data)

    lx_raw, ly_raw = parse_stick(data, 10)
    rx_raw, ry_raw = parse_stick(data, 13)

    lx = nintendo_to_xbox_axis(lx_raw)
    ly = nintendo_to_xbox_axis(ly_raw)
    rx = nintendo_to_xbox_axis(rx_raw)
    ry = nintendo_to_xbox_axis(ry_raw)

    deadzone = 1200
    lx = apply_deadzone(lx, deadzone)
    ly = apply_deadzone(ly, deadzone)
    rx = apply_deadzone(rx, deadzone)
    ry = apply_deadzone(ry, deadzone)

    lt = 0
    rt = 0
    if pid == PRODUCT_ID_GC:
        left_trigger, right_trigger = extract_gc_triggers(data)
        lt = left_trigger
        rt = right_trigger

    vgamepad_device.left_joystick(x_value=lx, y_value=ly)
    vgamepad_device.right_joystick(x_value=rx, y_value=ry)
    vgamepad_device.left_trigger(value=lt)
    vgamepad_device.right_trigger(value=rt)

    for btn in SWITCH_TO_XBOX_BUTTON.values():
        vgamepad_device.release_button(button=btn)
    for btn in pressed:
        xb_btn = SWITCH_TO_XBOX_BUTTON.get(btn)
        if xb_btn:
            vgamepad_device.press_button(button=xb_btn)

    vgamepad_device.update()

    if debug_mode:
        btns_display = ", ".join(str(b) for b in pressed) if pressed else "none"
        axes_display = f"LX:{lx:6d} LY:{ly:6d} RX:{rx:6d} RY:{ry:6d} LT:{lt:3d} RT:{rt:3d}"
        print(f"\r[vgamepad] Buttons: {btns_display:<35} | Axes: {axes_display}", end="")

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
        log_debug(f"Error finding characteristics: {e}")
        return False

async def initialize_controller(client, device):
    global current_state, controller_state, input_characteristic, output_characteristic
    controller_info = nintendo_device_info.get(device.address, {})
    pid = controller_info.get('product_id', PRODUCT_ID_PRO)
    controller_state = {'product_id': pid, 'connected': True}
    if not await find_characteristics(client):
        print("❌ Could not find suitable characteristics.")
        return False
    print("⏳ Initializing controller...")
    current_state = ControllerState.DONE
    await client.start_notify(input_characteristic, notification_callback)
    print(f"✅ Controller successfully initialized! ({get_nintendo_device_name(device)})")
    print("\n📊 Receiving controller data and forwarding to virtual gamepad...")
    print("   - Press Ctrl+C to quit")
    return True

async def connect_to_device(device):
    global controller_state
    device_name = get_nintendo_device_name(device)
    print(f"\n🔄 Connecting to {device_name} ({device.address})...")
    try:
        async with BleakClient(device) as client:
            print(f"✅ Connected to {device_name}!")
            if await initialize_controller(client, device):
                while keep_running and client.is_connected:
                    await asyncio.sleep(0.1)
                print("\n🔌 Controller disconnected.")
            else:
                print(f"❌ Controller initialization failed.")
    except Exception as e:
        print(f"❌ Connection error: {e}")
        controller_state = None

async def scan_for_nintendo_devices():
    print("\n🔍 Searching for Nintendo Switch controllers (5 seconds)...")
    try:
        devices = await BleakScanner.discover(timeout=5.0)
        nintendo_devices = []
        for device in devices:
            if is_nintendo_device(device):
                nintendo_devices.append(device)
                print(f"✅ Nintendo device found: {get_nintendo_device_name(device)} ({device.address})")
        if not nintendo_devices:
            print("❌ No Nintendo Switch controllers found.")
            print("\n📌 Make sure that:")
            print("   1. The controller is in pairing mode (LEDs blinking)")
            print("   2. Bluetooth is enabled on your device")
            print("   3. The controller is not connected to another device")
        return nintendo_devices
    except Exception as e:
        print(f"❌ Error scanning: {e}")
        return []

async def main():
    global vgamepad_device
    print("\n🎮 NS2 BLE Monitor + Virtual Gamepad (vgamepad/Xbox360)")
    print("=====================================================")
    print(f"🖥️  Platform: {platform.system()} {platform.release()}")
    print(f"🐍 Python: {platform.python_version()}")
    if platform.system() != "Windows":
        print("❌ This tool requires Windows with ViGEmBus. Exiting.")
        sys.exit(1)
    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    print("\nThis tool creates a virtual Xbox 360 gamepad based on a Nintendo Switch controller via Bluetooth.\n")
    print("📋 Pairing instructions:")
    print("1. Put your controller in pairing mode:")
    print("   - Pro Controller: Hold the small pairing button on the top")
    print("   - Joy-Con: Hold pairing button on the side")
    print("   - GameCube Controller: Hold pairing button on the top")
    print("2. Make sure the controller is not already connected to another device.\n")
    print("⚠️  NOTE: You must have ViGEmBus and vgamepad installed.\n")

    vgamepad_device = create_vgamepad_device()
    while keep_running:
        try:
            nintendo_devices = await scan_for_nintendo_devices()
            if nintendo_devices:
                await connect_to_device(nintendo_devices[0])
            if keep_running:
                print("\n⏳ Waiting 5 seconds before next scan...")
                for i in range(5, 0, -1):
                    if keep_running:
                        print(f"   Next scan in {i} seconds...", end="\r")
                        await asyncio.sleep(1)
                print(" " * 40, end="\r")
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"❌ Error: {e}")
            await asyncio.sleep(2)
    print("\n👋 Program ended.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='NS2 Bluetooth Monitor + Virtual Gamepad (vgamepad)')
    parser.add_argument('-d', '--debug', action='store_true', help='Enable debug output')
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable verbose output')
    args = parser.parse_args()
    debug_mode = args.debug
    verbose_mode = args.verbose
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n👋 Program terminated by user.")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
    finally:
        print("✅ Daemon terminated.")
