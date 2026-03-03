from __future__ import annotations

import asyncio
import os
import sys
import time
import threading
from dataclasses import dataclass
from typing import Optional, Dict, Callable

from bleak import BleakClient, BleakScanner

POSSIBLE_NAMES = ["PicoBLE"]

UU_SERVICE = "14128a76-04d1-6c4f-7e53-f2e80000b119"
UU_NOTIFY  = "14128a76-04d1-6c4f-7e53-f2e80100b119"
UU_WRITE   = "14128a76-04d1-6c4f-7e53-f2e80200b119"

# Track key states (True = pressed, False = released)
key_states: Dict[str, bool] = {'w': False, 'a': False, 's': False, 'd': False}

# Async events
state_changed = asyncio.Event()


def format_command(w: bool, a: bool, s: bool, d: bool) -> str:
    # Must match Pico firmware: "WASD:wxaxsxdx" (bits)
    return f"WASD:{'1' if w else '0'}{'1' if a else '0'}{'1' if s else '0'}{'1' if d else '0'}"


def print_status() -> None:
    status = " | ".join([
        f"W: {'█' if key_states['w'] else '□'}",
        f"A: {'█' if key_states['a'] else '□'}",
        f"S: {'█' if key_states['s'] else '□'}",
        f"D: {'█' if key_states['d'] else '□'}",
    ])
    print(f"\r{status}", end="", flush=True)


async def find_pico_device(timeout: float = 10.0):
    print("Scanning for Pico devices...")

    # First try filter-based scan
    def filt(d, _ad):
        return (d.name or "") in POSSIBLE_NAMES

    dev = await BleakScanner.find_device_by_filter(filt, timeout=timeout)
    if dev:
        print(f"Found: {dev.name or 'Unknown'} ({dev.address})")
        return dev

    # Fallback: list devices
    devices = await BleakScanner.discover(timeout=min(6.0, timeout))
    matches = [d for d in devices if (d.name or "") in POSSIBLE_NAMES]
    if matches:
        dev = matches[0]
        print(f"Found: {dev.name or 'Unknown'} ({dev.address})")
        return dev

    print(f"Found {len(devices)} devices (no match):")
    for d in devices:
        print(f"  {d.name or 'Unknown'} ({d.address})")
    return None


class TerminalMode:
    """Disable echo + canonical mode for cleaner terminal while running."""
    def __init__(self):
        self._old = None
        self._fd = None

    def __enter__(self):
        if os.name != "posix":
            return self
        try:
            import termios
            self._fd = sys.stdin.fileno()
            self._old = termios.tcgetattr(self._fd)
            new = termios.tcgetattr(self._fd)
            new[3] &= ~(termios.ECHO | termios.ICANON)  # lflags
            termios.tcsetattr(self._fd, termios.TCSADRAIN, new)
            termios.tcflush(self._fd, termios.TCIFLUSH)
        except Exception:
            # If anything fails, just don't touch the terminal.
            self._old = None
            self._fd = None
        return self

    def __exit__(self, exc_type, exc, tb):
        if os.name != "posix":
            return
        if self._old is None or self._fd is None:
            return
        try:
            import termios
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)
        except Exception:
            pass


@dataclass
class InputBackend:
    name: str

    def start(self, loop: asyncio.AbstractEventLoop,
              on_key: Callable[[str, bool], None],
              on_quit: Callable[[], None]) -> None:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError


class EvdevBackend(InputBackend):
    """
    Linux-only, true press/release (KEY_* events) via /dev/input/event*.
    """
    def __init__(self):
        super().__init__(name="evdev")
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._dev = None

    def _pick_keyboard(self):
        from evdev import InputDevice, list_devices, ecodes
        
        paths = list_devices()
        candidates = []
        
        # We look for devices that have at least the WASD keys
        need = {ecodes.KEY_W, ecodes.KEY_A, ecodes.KEY_S, ecodes.KEY_D}

        for path in paths:
            try:
                dev = InputDevice(path)
                caps = dev.capabilities(verbose=False)
                keys = set(caps.get(ecodes.EV_KEY, []))
                if need.issubset(keys):
                    candidates.append(dev)
                else:
                    dev.close()
            except Exception:
                continue

        if not candidates:
            return None

        # --- Interactive Selection ---
        print("\n--- Available Keyboards ---")
        for i, dev in enumerate(candidates):
            print(f"[{i}] {dev.path} | {dev.name} | Phys: {dev.phys}")
        
        while True:
            try:
                selection = input(f"\nSelect keyboard index (0-{len(candidates)-1}): ")
                idx = int(selection)
                if 0 <= idx < len(candidates):
                    # Close the ones we didn't pick
                    chosen = candidates[idx]
                    for i, dev in enumerate(candidates):
                        if i != idx:
                            dev.close()
                    return chosen
                print("Invalid index.")
            except ValueError:
                print("Please enter a number.")
            except KeyboardInterrupt:
                return None 

    def start(self, loop, on_key, on_quit) -> None:
        from evdev import ecodes
        self._dev = self._pick_keyboard()
        if not self._dev: raise RuntimeError("No keyboard selected.")
        
        # REMOVE OR COMMENT OUT THE LINE BELOW:
        # self._dev.grab()  <-- This is what freezes your trackpad/mouse
        
        keymap = {
            ecodes.KEY_W: 'w', ecodes.KEY_A: 'a', 
            ecodes.KEY_S: 's', ecodes.KEY_D: 'd',
            ecodes.KEY_Q: 'q', ecodes.KEY_ESC: 'q'
        }

        def run():
            try:
                for event in self._dev.read_loop():
                    if self._stop.is_set(): break
                    if event.type == ecodes.EV_KEY:
                        if event.code in keymap:
                            if event.value == 2: continue # Ignore repeat events
                            k = keymap[event.code]
                            down = (event.value == 1)
                            if k == 'q' and down: 
                                loop.call_soon_threadsafe(on_quit)
                                break
                            loop.call_soon_threadsafe(on_key, k, down)
            except Exception: pass

        self._thread = threading.Thread(target=run, daemon=True)
        self._thread.start() 

    def stop(self) -> None:
        self._stop.set()
        try:
            if self._dev is not None:
                try:
                    self._dev.ungrab()
                except Exception:
                    pass
                try:
                    self._dev.close()
                except Exception:
                    pass
        finally:
            self._dev = None


class StdinToggleBackend(InputBackend):
    """
    Fallback backend: reads stdin bytes (raw mode) and TOGGLES keys on each press.
    This does NOT provide true press/release detection.
    """
    def __init__(self):
        super().__init__(name="stdin-toggle")
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()

    def start(self, loop, on_key, on_quit) -> None:
        if os.name != "posix":
            raise RuntimeError("stdin backend requires POSIX terminal.")

        import select

        def run():
            while not self._stop.is_set():
                r, _, _ = select.select([sys.stdin], [], [], 0.05)
                if not r:
                    continue
                ch = sys.stdin.read(1)
                if not ch:
                    continue
                c = ch.lower()
                if c == 'q':
                    loop.call_soon_threadsafe(on_quit)
                    break
                if c in ('w', 'a', 's', 'd'):
                    # toggle
                    loop.call_soon_threadsafe(on_key, c, not key_states[c])

        self._thread = threading.Thread(target=run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()


def choose_input_backend() -> InputBackend:
    # Prefer evdev on Linux for accurate press/release
    if sys.platform.startswith("linux"):
        try:
            import evdev  # noqa: F401
            return EvdevBackend()
        except Exception:
            return StdinToggleBackend()
    # Non-Linux: best effort toggle backend
    return StdinToggleBackend()


async def main():
    dev = await find_pico_device(timeout=10.0)
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

            # Discover services
            services = client.services
            print("=== Services/Characteristics discovered ===")
            for svc in services:
                print(f"Service: {svc.uuid}")
                for ch in svc.characteristics:
                    print(f"  Char: {ch.uuid}  props={ch.properties}")

            # Validate required characteristics exist
            uuids = {str(ch.uuid).lower() for svc in services for ch in svc.characteristics}
            if UU_NOTIFY.lower() not in uuids or UU_WRITE.lower() not in uuids:
                print("Notify/Write characteristics not found.")
                return

            ready = asyncio.Event()

            def on_notify(_sender, data: bytearray):
                try:
                    msg = data.decode(errors="ignore").strip()
                except Exception:
                    msg = repr(data)
                if msg.startswith("<READY>"):
                    ready.set()

            print("Starting notifications...")
            await client.start_notify(UU_NOTIFY, on_notify)
            print("Notifications enabled.")

            try:
                await asyncio.wait_for(ready.wait(), timeout=5.0)
                print("Device is READY.")
            except asyncio.TimeoutError:
                print("No <READY> yet; continuing.")

            print("\n" + "=" * 60)
            print("WASD Motor Controller (Terminal Input)")
            print("=" * 60)
            print("Hold W/A/S/D to drive. Press Q (or Esc) to quit.")
            print("=" * 60)
            print()

            quit_event = asyncio.Event()

            def on_key(k: str, down: bool):
                if k in key_states:
                    if key_states[k] != down:
                        key_states[k] = down
                        state_changed.set()

            def on_quit():
                quit_event.set()

            backend = choose_input_backend()
            if backend.name != "evdev":
                print("Input backend: stdin-toggle (press toggles ON/OFF).")
                print("For true press/release on Linux, install evdev and run with permission to read /dev/input/event*.")
            else:
                print("Input backend: evdev (true press/release).")

            with TerminalMode():
                try:
                    backend.start(asyncio.get_running_loop(), on_key, on_quit)
                except Exception as e:
                    print(f"Input backend error ({backend.name}): {e}")
                    print("Falling back to stdin-toggle.")
                    backend = StdinToggleBackend()
                    backend.start(asyncio.get_running_loop(), on_key, on_quit)

                last_cmd: Optional[str] = None
                last_send = 0.0

                print_status()

                try:
                    while not quit_event.is_set():
                        # Wait for state change or periodic resend
                        try:
                            await asyncio.wait_for(state_changed.wait(), timeout=0.2)
                            state_changed.clear()
                        except asyncio.TimeoutError:
                            pass

                        cmd = format_command(
                            key_states['w'], key_states['a'], key_states['s'], key_states['d']
                        )

                        # Send on change, plus a periodic refresh to reduce "stuck" risk
                        now = time.monotonic()
                        if cmd != last_cmd or (now - last_send) > 0.5:
                            await client.write_gatt_char(UU_WRITE, cmd.encode(), response=True)
                            last_cmd = cmd
                            last_send = now
                            print_status()
                finally:
                    backend.stop()

            # Turn off all motors on exit
            try:
                await client.write_gatt_char(UU_WRITE, b"WASD:0000", response=True)
            except Exception:
                pass

            print("\n\nStopped. Motors OFF.")

    except Exception as e:
        print(f"\nConnection error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    os.system("clear" if os.name == "posix" else "cls")
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        # Best-effort: nothing else to do here; BLE context will close if connected.
        print("\n\nGoodbye!")

