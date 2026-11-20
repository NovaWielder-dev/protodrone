from dataclasses import dataclass
import struct
from pynput import keyboard
import socket
import time

# ========== CONFIG ========== #
ESP_IP = "192.168.4.1"
ESP_PORT = 4210
LOOP_HZ = 50
DT = 1.0 / LOOP_HZ

# ========== COMMAND STATE ========== #
# Mirrors Receiver::ControlTargets


@dataclass 
class ControlTargets:
    armed: bool = False
    throttle: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    yaw_rate: float = 0.0

    def clamp_all(self) -> None:
        self.throttle = max(0.0, min(1.0, self.throttle))
        


# ========== WIRE PROTOCOL (CODEC) ========== #

MAGIC_1 = 0xAA
MAGIC_2 = 0x55

def pack_command(targets: ControlTargets) -> bytes:

    payload = struct.pack(
        "<2BB4f",
        MAGIC_1,
        MAGIC_2,
        1 if targets.armed else 0,
        targets.throttle,
        targets.pitch,
        targets.roll,
        targets.yaw_rate
    )

    checksum = 0
    for byte in payload:
        checksum ^= byte # XOR each byte

    return payload + struct.pack("<B", checksum)

# ========== KEYBOARD INPUT (EVENT LISTENER) ========== #

pressed_keys: set[str] = set()

def key_name(key) -> str:
    # Get string version of passed in keypress
    if hasattr(key, "char") and key.char is not None:
        return key.char.lower()
    return str(key).replace("Key.", "")

def on_press(key) -> None:
    pressed_keys.add(key_name(key))

def on_release(key) -> None:
    pressed_keys.discard(key_name(key))

# ========== LOOP ========== #

def run_loop() -> None:
    targets = ControlTargets()

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        next_tick = time.monotonic()

        try:
            while True:
                # 1. Read pressed_keys -> update targets
                # 2. pkt = pack_command(targets)

                packet = pack_command(targets)
                try:
                    sock.sendto(packet, (ESP_IP, ESP_PORT))
                except OSError as err:
                    print(f"Send error: {err}")

                # Print keypress + packet contents to console
                print(packet.hex(" "), "---> ", pressed_keys.copy())
                
                # Wait until next 20 ms tick without timing drift
                next_tick += DT
                sleep_time = next_tick - time.monotonic()
                if sleep_time > 0:
                    time.sleep(sleep_time)
        except KeyboardInterrupt:
            print("\nStopping")
        finally:
            # Safety: send disarm packet before socket closes
            targets.armed = False
            targets.throttle = 0.0
            try:
                sock.sendto(pack_command(targets), (ESP_IP, ESP_PORT))
            except OSError:
                pass

# ========== EXECUTION CALLS ========== #
if __name__ == "__main__":
    with keyboard.Listener(on_press=on_press, on_release=on_release):
        run_loop()