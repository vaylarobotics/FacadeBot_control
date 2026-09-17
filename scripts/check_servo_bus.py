"""
Probe the LX-16A servo bus from the ESP32 and report which layer is broken.

Read-only: this script only ever sends the position-read command, so it cannot
move the arm. Run it when the arm does not respond but Wi-Fi does.

Why this runs ON the ESP32 over USB rather than on the RPi over Wi-Fi: a "move"
ack from the firmware proves only that the JSON arrived and the packets were
written to UART2 - move_joints() in esp32_firmware/main.py never checks that a
servo received anything. So an arm that does not move while TCP looks healthy
has a fault somewhere in ESP32 -> BusLinker -> servo, and putting Wi-Fi back in
the diagnostic path only adds a layer that is already known to work.

Unlike read_servo_position() in the firmware, which collapses every failure into
None, this prints the raw bytes received. The difference between "no bytes at
all", "only our own transmission echoed back" and "bytes that are not a valid
frame" is what tells the three layers apart.

Usage (board connected over USB):
    /home/harthik/.firmware/bin/mpremote connect /dev/ttyUSB0 run scripts/check_servo_bus.py

mpremote interrupts main.py to run this, so the firmware is not serving TCP
while the probe runs. Restart it afterwards:
    /home/harthik/.firmware/bin/mpremote connect /dev/ttyUSB0 reset
"""
import time
from machine import UART, Pin

# ── Settings you may want to change ───────────────────────────────────────────
# Probed ID range is deliberately wider than SERVO_IDS in esp32_firmware/main.py:
# a servo replaced during a rebuild answers to the factory default ID 1, and a
# mis-programmed one may sit anywhere, so a silent 1-4 is not proof of a dead bus.
PROBE_ID_MIN = 1
PROBE_ID_MAX = 8

# Set True to test the ESP32's own UART2 in isolation. Unplug the BusLinker and
# jumper GPIO17 directly to GPIO16 first. This is the only check that separates
# "this board is broken" from "the wiring is broken".
RUN_LOOPBACK_TEST = False

# ── Hardware configuration (must match esp32_firmware/main.py) ────────────────
SERVO_IDS             = [1, 2, 3, 4]  # the IDs the firmware actually drives, base to end effector
BAUD_BUSLINKER        = 115200
TX2_PIN               = 17            # ESP32 GPIO17 → BusLinker TTL RX
RX2_PIN               = 16            # ESP32 GPIO16 ← BusLinker TTL TX
_READ_TIMEOUT_MS      = 10            # ms to wait for the first RX byte
_READ_TIMEOUT_CHAR_MS = 5             # ms allowed between successive RX bytes
_INTER_SERVO_DELAY_MS = 20            # gap between back-to-back sends on the bus

# ── Servo protocol constants (LX-16A) ─────────────────────────────────────────
# Packet: 0x55 0x55 ID LEN CMD [PARAMS...] CHECKSUM, LEN = num_params + 3
_HEADER_BYTE          = 0x55
_CMD_POS_READ         = 0x1C
_READ_REQUEST_BYTES   = 6   # 0x55 0x55 ID 3 CMD CHECKSUM
_READ_RESPONSE_BYTES  = 8   # 0x55 0x55 ID 5 CMD POS_LO POS_HI CHECKSUM
_READ_RESPONSE_LEN    = 5   # the LEN field of a valid position-read response
_READ_MAX_BYTES       = 14  # _READ_RESPONSE_BYTES + the request if the BusLinker echoes TX

# LX-16A datasheet: raw 0-1000 spans the full 0-240 degrees of mechanical travel.
_POSITION_MAX_RAW     = 1000
_SERVO_ANGLE_MAX_DEG  = 240.0

# ── Loopback test constants ───────────────────────────────────────────────────
# Arbitrary pattern with both header bytes and high/low bit activity, so a stuck
# line reads back as obviously wrong rather than accidentally matching.
_LOOPBACK_PATTERN     = bytes([0x55, 0xAA, 0x00, 0xFF, 0x12, 0x34])
_LOOPBACK_SETTLE_MS   = 50

# ── Verdict codes ─────────────────────────────────────────────────────────────
_RESULT_ALIVE     = "ALIVE"
_RESULT_NO_BYTES  = "NO BYTES"
_RESULT_ECHO_ONLY = "ECHO ONLY"
_RESULT_GARBAGE   = "GARBAGE"


def _format_hex(raw: bytes) -> str:
    if not raw:
        return "(nothing)"
    return " ".join("%02X" % byte for byte in raw)


def build_read_packet(servo_id: int) -> bytes:
    length = 3  # 0 params + 3
    checksum = (~(servo_id + length + _CMD_POS_READ)) & 0xFF
    return bytes([_HEADER_BYTE, _HEADER_BYTE, servo_id, length, _CMD_POS_READ, checksum])


def find_response(raw: bytes, servo_id: int) -> int | None:
    """Return the position in raw counts if a valid frame for this ID is present."""
    for i in range(len(raw) - (_READ_RESPONSE_BYTES - 1)):
        if (raw[i] == _HEADER_BYTE
                and raw[i + 1] == _HEADER_BYTE
                and raw[i + 2] == servo_id
                and raw[i + 3] == _READ_RESPONSE_LEN
                and raw[i + 4] == _CMD_POS_READ):
            return (raw[i + 6] << 8) | raw[i + 5]
    return None


def classify(raw: bytes, request: bytes, servo_id: int) -> tuple[str, int | None]:
    if not raw:
        return _RESULT_NO_BYTES, None
    position_raw = find_response(raw, servo_id)
    if position_raw is not None:
        return _RESULT_ALIVE, position_raw
    if raw == request:
        return _RESULT_ECHO_ONLY, None
    return _RESULT_GARBAGE, None


def probe_servo(uart: UART, servo_id: int) -> tuple[str, int | None, bytes]:
    if uart.any():
        uart.read()  # drop stale bytes from a previous servo's late reply
    request = build_read_packet(servo_id)
    uart.write(request)
    raw = uart.read(_READ_MAX_BYTES) or b""
    result, position_raw = classify(raw, request, servo_id)
    return result, position_raw, raw


def run_loopback_test(uart: UART) -> None:
    print("Loopback test: BusLinker must be UNPLUGGED and GPIO17 jumpered to GPIO16.")
    if uart.any():
        uart.read()
    uart.write(_LOOPBACK_PATTERN)
    time.sleep_ms(_LOOPBACK_SETTLE_MS)
    raw = uart.read(len(_LOOPBACK_PATTERN)) or b""
    print("  sent    :", _format_hex(_LOOPBACK_PATTERN))
    print("  received:", _format_hex(raw))
    if raw == _LOOPBACK_PATTERN:
        print("  PASS - UART2 transmits and receives. The ESP32 is not the fault;")
        print("         look at the BusLinker, the TTL wiring or servo power.")
    elif not raw:
        print("  FAIL - nothing came back. Either the jumper is not actually")
        print("         bridging GPIO17 to GPIO16, or UART2 on this board is dead.")
    else:
        print("  FAIL - bytes came back but altered. Suspect a baud mismatch or a")
        print("         noisy/half-connected jumper.")


def print_verdict(results: dict) -> None:
    alive_ids = [servo_id for servo_id, (result, _) in results.items() if result == _RESULT_ALIVE]
    all_results = [result for result, _ in results.values()]

    print()
    print("Verdict:")
    if not alive_ids and all(result == _RESULT_NO_BYTES for result in all_results):
        print("  The bus is dead - not one byte came back from any ID.")
        print("  In likelihood order, check:")
        print("    1. Servo power rail at the BusLinker. The LX-16A needs its own 6-8.4 V")
        print("       supply. If only the ESP32's USB rail is live, Wi-Fi works perfectly")
        print("       while the bus is silent - exactly this symptom.")
        print("    2. Common ground between the ESP32 and the BusLinker. A missing ground")
        print("       kills both directions and leaves the ESP32 apparently healthy.")
        print("    3. TX/RX orientation: GPIO17 -> BusLinker TTL RX, GPIO16 <- BusLinker TTL TX.")
        print("    4. The BusLinker V2.5 mode jumper. In USB-passthrough mode the TTL header")
        print("       is inert.")
        print("  Then set RUN_LOOPBACK_TEST = True in this file to clear the ESP32 itself.")
    elif not alive_ids and _RESULT_ECHO_ONLY in all_results:
        print("  The ESP32 reaches the BusLinker (it echoed our transmission back), but no")
        print("  servo answered. Suspect servo power, or servos that no longer hold the IDs")
        print("  the firmware drives.")
    elif not alive_ids and _RESULT_GARBAGE in all_results:
        print("  Bytes are arriving but they are not valid frames. Suspect a baud mismatch")
        print("  between the ESP32 and the BusLinker, or electrical noise on the line.")
    else:
        missing_ids = [servo_id for servo_id in SERVO_IDS if servo_id not in alive_ids]
        unexpected_ids = [servo_id for servo_id in alive_ids if servo_id not in SERVO_IDS]
        print("  Servos answering:", alive_ids)
        if missing_ids:
            print("  MISSING - the firmware drives these but they did not answer:", missing_ids)
        if unexpected_ids:
            print("  UNEXPECTED - these answered but the firmware does not drive them:",
                  unexpected_ids)
            print("  The rebuild probably left servos on the wrong IDs. Either renumber them")
            print("  or update SERVO_IDS in esp32_firmware/main.py to match.")
        if not missing_ids and not unexpected_ids:
            print("  Bus is healthy: every driven servo answered and nothing unexpected did.")


def main() -> None:
    uart = UART(2, baudrate=BAUD_BUSLINKER, tx=Pin(TX2_PIN), rx=Pin(RX2_PIN),
                timeout=_READ_TIMEOUT_MS, timeout_char=_READ_TIMEOUT_CHAR_MS)

    if RUN_LOOPBACK_TEST:
        run_loopback_test(uart)
        return

    print("Probing servo IDs %d-%d at %d baud on UART2 (TX GPIO%d / RX GPIO%d)."
          % (PROBE_ID_MIN, PROBE_ID_MAX, BAUD_BUSLINKER, TX2_PIN, RX2_PIN))
    print("Firmware drives IDs %s. Nothing here moves the arm." % SERVO_IDS)
    print()

    results = {}
    for servo_id in range(PROBE_ID_MIN, PROBE_ID_MAX + 1):
        result, position_raw, raw = probe_servo(uart, servo_id)
        results[servo_id] = (result, position_raw)

        detail = ""
        if position_raw is not None:
            angle_deg = position_raw * _SERVO_ANGLE_MAX_DEG / _POSITION_MAX_RAW
            detail = "  position %d raw (%.1f deg)" % (position_raw, angle_deg)
        print("  ID %-3d %-10s raw: %s%s" % (servo_id, result, _format_hex(raw), detail))
        time.sleep_ms(_INTER_SERVO_DELAY_MS)

    print_verdict(results)


main()
