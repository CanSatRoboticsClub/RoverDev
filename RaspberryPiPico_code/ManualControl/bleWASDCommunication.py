from bleak import BleakClient, BleakScanner
import asyncio
import os
import sys

try:
    if sys.platform == 'win32':
        import msvcrt
    else:
        import tty
        import termios
        import select
except ImportError:
    print("Warning: Keyboard input may not work on this platform")


try:
    from pynput import keyboard
    USE_PYNPUT = True
except ImportError:
    USE_PYNPUT = False
    print("Note: For best results, install pynput: pip install pynput")

POSSIBLE_NAMES = ["PicoBLE"]
UU_SERVICE = "14128a76-04d1-6c4f-7e53-f2e80000b119"
UU_NOTIFY  = "14128a76-04d1-6c4f-7e53-f2e80100b119"
UU_WRITE   = "14128a76-04d1-6cq4f-7e53-f2e80200b119"

NOTIFY_FORMS = {UU_NOTIFY.lower()}
WRITE_FORMS  = {UU_WRITE.lower()}


key_states = {'w': False, 'a': False, 's': False, 'd': False}
state_changed = asyncio.Event()

async def find_pico_device():
    print("Scanning for Pico devices...")
    def filt(d, ad): return (d.name or "") in POSSIBLE_NAMES
    dev = await BleakScanner.find_device_by_filter(filt, timeout=10.0)
    if dev:
        print(f"Found: {dev.name or 'Unknown'} ({dev.address})")
        return dev
    devices = await BleakScanner.discover(timeout=5.0)
    print(f"Found {len(devices)} devices (no match):")
    for d in devices:
        print(f"  {d.name or 'Unknown'} ({d.address})")
    return None

async def wait_for_services(client, timeout=6.0):
    end = asyncio.get_event_loop().time() + timeout
    while asyncio.get_event_loop().time() < end:
        svcs = client.services
        if svcs and len(list(svcs)) > 0:
            return svcs
        await asyncio.sleep(0.2)
    return client.services

def format_command(w, a, s, d):
    """Format WASD command: WASD:wxaxsxdx where x is 0 or 1"""
    return f"WASD:{'1' if w else '0'}{'1' if a else '0'}{'1' if s else '0'}{'1' if d else '0'}"

def print_status():
    """Print current key states"""
    status = " | ".join([
        f"W: {'█' if key_states['w'] else '□'}",
        f"A: {'█' if key_states['a'] else '□'}",
        f"S: {'█' if key_states['s'] else '□'}",
        f"D: {'█' if key_states['d'] else '□'}"
    ])
    print(f"\r{status}", end='', flush=True)

class KeyboardListener:
    """Keyboard listener using pynput for press/release detection"""
    
    def __init__(self):
        self.listener = None
        self.running = False
    
    def on_press(self, key):
        try:
            if hasattr(key, 'char') and key.char:
                k = key.char.lower()
                if k in ['w', 'a', 's', 'd']:
                    if not key_states[k]:
                        key_states[k] = True
                        state_changed.set()
                elif k == 'q':
                    self.running = False
                    return False
        except AttributeError:
            pass
    
    def on_release(self, key):
        try:
            if hasattr(key, 'char') and key.char:
                k = key.char.lower()
                if k in ['w', 'a', 's', 'd']:
                    if key_states[k]: 
                        key_states[k] = False
                        state_changed.set()
        except AttributeError:
            pass
    
    def start(self):
        self.running = True
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()
    
    def stop(self):
        if self.listener:
            self.listener.stop()

async def main():
    dev = await find_pico_device()
    if not dev:
        print("Device not found")
        return

    print(f"\nConnecting to {dev.name or 'Unknown'}...")
    try:
        async with BleakClient(dev, timeout=15.0) as client:
            if not client.is_connected:
                print("Failed to connect.")
                return
            print("Connected.")


            services = await wait_for_services(client, timeout=6.0)
            print("=== Services/Characteristics discovered ===")
            if services:
                for svc in services:
                    print(f"Service: {svc.uuid}")
                    for ch in svc.characteristics:
                        print(f"  Char: {ch.uuid}  props={ch.properties}")
            else:
                print("(no services yet)")


            notify_char = write_char = None
            if services:
                for svc in services:
                    for ch in svc.characteristics:
                        u = str(ch.uuid).lower()
                        if u in NOTIFY_FORMS:
                            notify_char = ch
                            print(f"Found notify char: {ch.uuid}")
                        elif u in WRITE_FORMS:
                            write_char = ch
                            print(f"Found write char: {ch.uuid}")

            if not notify_char or not write_char:
                print("Notify/Write characteristics not found.")
                return

            ready = asyncio.Event()
            
            def on_notify(_, data: bytearray):
                try:
                    msg = data.decode(errors="ignore").strip()
                except Exception:
                    msg = repr(data)
                if msg == "<READY>":
                    ready.set()


            print("Starting notifications...")
            await client.start_notify(notify_char, on_notify)
            print("Notifications enabled.")


            try:
                await asyncio.wait_for(ready.wait(), timeout=5.0)
                print("Device is READY.")
            except asyncio.TimeoutError:
                print("No <READY> yet; continuing.")

            print("\n" + "="*60)
            print("WASD LED Controller - Real-time Press/Release Detection")
            print("="*60)
            print("Hold down W, A, S, D keys to turn LEDs ON")
            print("Release keys to turn LEDs OFF")
            print("Press Q to quit")
            print("="*60)
            
            if not USE_PYNPUT:
                print("\nWARNING: pynput not installed. Install it for proper key detection:")
                print("  pip install pynput")
                print("\nFalling back to basic mode (may not work well)...\n")
            
            print()
            print_status()

            if USE_PYNPUT:

                kb_listener = KeyboardListener()
                kb_listener.start()
                
                last_cmd = None
                
                try:
                    while kb_listener.running:

                        try:
                            await asyncio.wait_for(state_changed.wait(), timeout=0.1)
                            state_changed.clear()
                        except asyncio.TimeoutError:
                            pass
                        

                        cmd = format_command(
                            key_states['w'],
                            key_states['a'],
                            key_states['s'],
                            key_states['d']
                        )
                        
                        if cmd != last_cmd:
                            await client.write_gatt_char(write_char, cmd.encode(), response=True)
                            last_cmd = cmd
                            print_status()
                        

                        await asyncio.sleep(0.01)
                finally:
                    kb_listener.stop()
            else:

                print("Basic mode active. This may not detect key releases properly.")
                last_cmd = None
                
                while True:
                    if sys.platform == 'win32' and msvcrt.kbhit():
                        key = msvcrt.getch().decode('utf-8', errors='ignore').lower()
                        
                        if key == 'q':
                            break
                        elif key in ['w', 'a', 's', 'd']:

                            key_states[key] = not key_states[key]
                            
                            cmd = format_command(
                                key_states['w'],
                                key_states['a'],
                                key_states['s'],
                                key_states['d']
                            )
                            
                            if cmd != last_cmd:
                                await client.write_gatt_char(write_char, cmd.encode(), response=True)
                                last_cmd = cmd
                                print_status()
                    
                    await asyncio.sleep(0.01)


            await client.write_gatt_char(write_char, b"WASD:0000", response=True)
            print("\n\nAll LEDs turned off.")

    except Exception as e:
        print(f"\nConnection error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    os.system("cls" if os.name == "nt" else "clear")
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\nGoodbye!")
