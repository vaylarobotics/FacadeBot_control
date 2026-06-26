import json
import sys
import time
import uselect
from machine import UART, Pin

# ── Hardware configuration ────────────────────────────────────────────────────
# Confirm GPIO pins match your physical wiring before flashing.
# RPi communication uses sys.stdin/stdout (UART0 is already owned by MicroPython REPL).
# UART2 (GPIO 16/17) is wired to the BusLinker V2.5 TTL header.
SERVO_IDS        = [1, 2, 3, 4]    # one servo per joint, base to end effector
HOME_POSITION    = 500              # center of 0–1000 range (= 120° physical)
POSITION_B       = [300, 600, 400, 700]  # target pose B, one value per servo ID in SERVO_IDS order
MOVE_DURATION_MS = 1000            # time in ms for each servo to reach home
BAUD_BUSLINKER   = 115200
TX2_PIN          = 17              # ESP32 GPIO17 → BusLinker TTL RX
RX2_PIN          = 16              # ESP32 GPIO16 ← BusLinker TTL TX

# ── Servo protocol constants ──────────────────────────────────────────────────
# LX-16A packet: 0x55 0x55 ID LEN CMD [PARAMS...] CHECKSUM
# LEN = num_params + 3 (covers CMD + PARAMS + CHECKSUM)
_CMD_MOVE_TIME_WRITE = 1
_POSITION_MIN        = 0
_POSITION_MAX        = 1000
_DURATION_MIN_MS     = 100
_DURATION_MAX_MS     = 5000
_INTER_SERVO_DELAY_MS = 20  # gap between back-to-back sends on the bus

uart_buslinker = UART(2, baudrate=BAUD_BUSLINKER, tx=Pin(TX2_PIN), rx=Pin(RX2_PIN))


def build_move_packet(servo_id, position, duration_ms):
    position    = max(_POSITION_MIN, min(_POSITION_MAX, position))
    duration_ms = max(_DURATION_MIN_MS, min(_DURATION_MAX_MS, duration_ms))

    pos_lo  = position    & 0xFF
    pos_hi  = (position    >> 8) & 0xFF
    time_lo = duration_ms & 0xFF
    time_hi = (duration_ms >> 8) & 0xFF

    length   = 7  # 4 params + 3
    checksum = (~(servo_id + length + _CMD_MOVE_TIME_WRITE
                  + pos_lo + pos_hi + time_lo + time_hi)) & 0xFF

    return bytes([0x55, 0x55, servo_id, length,
                  _CMD_MOVE_TIME_WRITE,
                  pos_lo, pos_hi, time_lo, time_hi,
                  checksum])


def move_all_to_home():
    for servo_id in SERVO_IDS:
        uart_buslinker.write(build_move_packet(servo_id, HOME_POSITION, MOVE_DURATION_MS))
        time.sleep_ms(_INTER_SERVO_DELAY_MS)


def move_to_position_b():
    for servo_id, position in zip(SERVO_IDS, POSITION_B):
        uart_buslinker.write(build_move_packet(servo_id, position, MOVE_DURATION_MS))
        time.sleep_ms(_INTER_SERVO_DELAY_MS)


def main():
    # poll with a timeout so Ctrl+C can always interrupt the loop
    poller = uselect.poll()
    poller.register(sys.stdin, uselect.POLLIN)

    print('{"status": "ready"}')

    while True:
        ready = poller.poll(100)  # wait up to 100 ms then loop — keeps Ctrl+C responsive
        if not ready:
            continue

        line = sys.stdin.readline()
        if not line:
            continue

        try:
            cmd = json.loads(line.strip())
        except ValueError:
            print('{"status": "error", "msg": "invalid json"}')
            continue

        if cmd.get("cmd") == "home":
            move_all_to_home()
            print('{"status": "ok"}')
        elif cmd.get("cmd") == "position_b":
            move_to_position_b()
            print('{"status": "ok"}')
        else:
            print('{"status": "error", "msg": "unknown command"}')


main()
