import json
import network
import socket
import time
from machine import UART, Pin

# ── Wi-Fi and TCP configuration ───────────────────────────────────────────────
# Fill in your network credentials and router gateway before flashing.
WIFI_SSID        = "TP-Link_08F3"       # replace with your network name
WIFI_PASSWORD    = "16288935"   # replace with your network password
STATIC_IP        = "192.168.1.100"        # fixed IP the ESP32 will claim — pick one
                                           # not already in use on your network
SUBNET_MASK      = "255.255.255.0"
GATEWAY          = "192.168.1.1"          # your router's IP — run `ip route` on RPi to confirm
DNS              = "8.8.8.8"
TCP_PORT         = 5000                   # port the ESP32 listens on for commands

# ── Hardware configuration ────────────────────────────────────────────────────
# Confirm GPIO pins match your physical wiring before flashing.
# UART2 (GPIO16/17) is wired to the BusLinker V2.5 TTL header.
SERVO_IDS        = [1, 2, 3, 4]    # one servo per joint, base to end effector
HOME_POSITION    = 500              # center of 0–1000 range (= 120° physical)
MOVE_DURATION_MS = 1000            # time in ms used by the home command
BAUD_BUSLINKER   = 115200
TX2_PIN          = 17              # ESP32 GPIO17 → BusLinker TTL RX
RX2_PIN          = 16              # ESP32 GPIO16 ← BusLinker TTL TX

# ── Servo protocol constants ──────────────────────────────────────────────────
# LX-16A packet: 0x55 0x55 ID LEN CMD [PARAMS...] CHECKSUM
# LEN = num_params + 3 (covers CMD + PARAMS + CHECKSUM)
_CMD_MOVE_TIME_WRITE  = 1
_POSITION_MIN         = 0
_POSITION_MAX         = 1000
_DURATION_MIN_MS      = 100
_DURATION_MAX_MS      = 5000
_INTER_SERVO_DELAY_MS = 20  # gap between back-to-back sends on the bus

uart_buslinker = UART(2, baudrate=BAUD_BUSLINKER, tx=Pin(TX2_PIN), rx=Pin(RX2_PIN))


def build_move_packet(servo_id: int, position: int, duration_ms: int) -> bytes:
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


def move_joints(positions: list, duration_ms: int) -> None:
    for servo_id, position in zip(SERVO_IDS, positions):
        uart_buslinker.write(build_move_packet(servo_id, position, duration_ms))
        time.sleep_ms(_INTER_SERVO_DELAY_MS)


def move_all_to_home() -> None:
    move_joints([HOME_POSITION] * len(SERVO_IDS), MOVE_DURATION_MS)


def connect_wifi() -> network.WLAN:
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    # Set static IP before connecting so the address is predictable
    wlan.ifconfig((STATIC_IP, SUBNET_MASK, GATEWAY, DNS))
    wlan.connect(WIFI_SSID, WIFI_PASSWORD)
    while not wlan.isconnected():
        time.sleep_ms(200)
    return wlan


def handle_client(conn: socket.socket) -> None:
    """Read JSON commands from one connected client and send responses until it disconnects."""
    conn.sendall(b'{"status": "ready"}\n')
    f = conn.makefile('rb')
    while True:
        line = f.readline()
        if not line:
            break  # client closed the connection

        try:
            cmd = json.loads(line.strip())
        except ValueError:
            conn.sendall(b'{"status": "error", "msg": "invalid json"}\n')
            continue

        if cmd.get("cmd") == "home":
            move_all_to_home()
            conn.sendall(b'{"status": "ok"}\n')
        elif cmd.get("cmd") == "move":
            positions = cmd.get("positions")
            duration_ms = cmd.get("duration_ms")
            if not isinstance(positions, list) or len(positions) != len(SERVO_IDS):
                conn.sendall(b'{"status": "error", "msg": "positions must be a list with one value per servo"}\n')
            elif not isinstance(duration_ms, int):
                conn.sendall(b'{"status": "error", "msg": "duration_ms must be an integer"}\n')
            else:
                move_joints(positions, duration_ms)
                conn.sendall(b'{"status": "ok"}\n')
        else:
            conn.sendall(b'{"status": "error", "msg": "unknown command"}\n')


def main() -> None:
    connect_wifi()

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # SO_REUSEADDR lets the ESP32 rebind immediately after a reset without
    # waiting for the OS to release the port (avoids "address already in use")
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('', TCP_PORT))
    server.listen(1)

    while True:
        conn, addr = server.accept()
        try:
            handle_client(conn)
        except OSError:
            pass  # network errors during a session — just accept the next client
        finally:
            conn.close()


main()
