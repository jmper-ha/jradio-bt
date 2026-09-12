#!/usr/bin/env python3
"""Talk to a jradio-bt module from a PC, over its UART, standing in for the
host. The second implementation of the protocol on purpose: what the C code
encodes, this decodes, and the test vector below pins the two together.

    tools/jbt.py -p /dev/ttyUSB0 ping
    tools/jbt.py -p /dev/ttyUSB0 status
    tools/jbt.py -p /dev/ttyUSB0 name "jRadio кухня"
    tools/jbt.py -p /dev/ttyUSB0 pairing on
    tools/jbt.py -p /dev/ttyUSB0 mode off
    tools/jbt.py -p /dev/ttyUSB0 monitor        # print every frame, LOG included
    tools/jbt.py selftest                       # no board needed

The UART to the host is not the module's USB console: wire a USB-serial
adapter to the pins named in menuconfig (default TX 17 / RX 16), 921600 8N1.
"""

import argparse
import struct
import sys
import time

END, ESC, ESC_END, ESC_ESC = 0xC0, 0xDB, 0xDC, 0xDD
PROTOCOL_VERSION = 1
PAYLOAD_MAX = 520

MSG = {
    "SET_MODE": 0x01, "SET_NAME": 0x02, "PAIRING": 0x03, "SCAN": 0x04, "CONNECT": 0x05,
    "DISCONNECT": 0x06, "PASSTHROUGH": 0x07, "SET_VOLUME": 0x08, "I2S_FORMAT": 0x09,
    "GET_STATUS": 0x0A, "COVER_GET": 0x0B, "FORGET": 0x0C, "PING": 0x0D,
    "STATUS": 0x80, "MODE_ACK": 0x81, "TRACK": 0x82, "POSITION": 0x83, "PLAY_STATE": 0x84,
    "VOLUME": 0x85, "COVER_INFO": 0x86, "COVER_DATA": 0x87, "SCAN_RESULT": 0x88,
    "EVENT": 0x89, "LOG": 0x8A, "PONG": 0x8B, "ACK": 0xFE,
}
NAME = {v: k for k, v in MSG.items()}
FLAG_WANT_ACK, FLAG_IS_ACK = 0x01, 0x02
TAG = {0x01: "name", 0x02: "peer_name", 0x10: "title", 0x11: "artist", 0x12: "album",
       0x13: "genre", 0x14: "duration_ms", 0x15: "track_no", 0x16: "cover_hash", 0x20: "text"}
U32_TAGS = {0x14, 0x15, 0x16}
MODE = ["off", "sink", "source"]
CONN = ["none", "connecting", "connected", "pairing", "scanning"]
PLAY = ["stopped", "playing", "paused"]
CODEC = ["none", "sbc", "aac"]
RESULT = ["ok", "bad_arg", "busy", "unsupported", "failed"]
EVENT = ["connected", "disconnected", "pairing_failed", "i2s_clock_lost", "booted"]
LEVEL = {1: "E", 2: "W", 3: "I", 4: "D", 5: "V"}


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode(mtype: int, payload: bytes = b"", seq: int = 0, flags: int = 0) -> bytes:
    raw = bytes([mtype, flags, seq]) + struct.pack("<H", len(payload)) + payload
    raw += struct.pack("<H", crc16(raw))
    out = bytearray([END])
    for b in raw:
        if b == END:
            out += bytes([ESC, ESC_END])
        elif b == ESC:
            out += bytes([ESC, ESC_ESC])
        else:
            out.append(b)
    out.append(END)
    return bytes(out)


class Decoder:
    def __init__(self):
        self.buf = bytearray()
        self.escaping = False
        self.bad = False
        self.dropped = 0

    def feed(self, data: bytes):
        """Yields (type, flags, seq, payload) for every good frame in data."""
        for b in data:
            if b == END:
                frame = self._close()
                if frame is not None:
                    yield frame
                continue
            if self.escaping:
                self.escaping = False
                if b == ESC_END:
                    b = END
                elif b == ESC_ESC:
                    b = ESC
                else:
                    self.bad = True
                    continue
            elif b == ESC:
                self.escaping = True
                continue
            if not self.bad:
                self.buf.append(b)

    def _close(self):
        buf, bad, esc = bytes(self.buf), self.bad, self.escaping
        self.buf, self.bad, self.escaping = bytearray(), False, False
        if not buf and not bad and not esc:
            return None
        if bad or esc or len(buf) < 7:
            self.dropped += 1
            return None
        length = struct.unpack_from("<H", buf, 3)[0]
        if 5 + length + 2 != len(buf) or crc16(buf[: 5 + length]) != struct.unpack_from("<H", buf, 5 + length)[0]:
            self.dropped += 1
            return None
        return buf[0], buf[1], buf[2], buf[5 : 5 + length]


def tlv(payload: bytes, offset: int = 0) -> dict:
    out = {}
    while offset + 2 <= len(payload):
        tag, length = payload[offset], payload[offset + 1]
        value = payload[offset + 2 : offset + 2 + length]
        if len(value) < length:
            break
        name = TAG.get(tag, f"tag{tag:02x}")
        out[name] = struct.unpack("<I", value)[0] if tag in U32_TAGS and length == 4 else value.decode("utf-8", "replace")
        offset += 2 + length
    return out


def describe(mtype: int, flags: int, seq: int, payload: bytes) -> str:
    name = NAME.get(mtype, f"0x{mtype:02X}")
    try:
        if mtype == MSG["STATUS"]:
            mode, conn, peer, codec, rate, play, vol = struct.unpack_from("<BB6sBIBB", payload)
            peer_text = ":".join(f"{b:02X}" for b in peer)
            return (f"STATUS mode={MODE[mode]} conn={CONN[conn]} peer={peer_text} codec={CODEC[codec]} "
                    f"rate={rate} play={PLAY[play]} volume={vol} {tlv(payload, 15)}")
        if mtype == MSG["PONG"]:
            proto, major, minor, build = struct.unpack("<BBBH", payload)
            return f"PONG protocol={proto} firmware={major}.{minor} build {build}"
        if mtype == MSG["MODE_ACK"]:
            return f"MODE_ACK mode={MODE[payload[0]]} result={RESULT[payload[1]]}"
        if mtype == MSG["ACK"]:
            return f"ACK seq={payload[0]} result={RESULT[payload[1]]}"
        if mtype == MSG["EVENT"]:
            return f"EVENT {EVENT[payload[0]]} {tlv(payload, 1)}"
        if mtype == MSG["LOG"]:
            return f"LOG {LEVEL.get(payload[0], '?')} {tlv(payload, 1).get('text', '')}"
        if mtype == MSG["TRACK"]:
            return f"TRACK {tlv(payload)}"
        if mtype == MSG["POSITION"]:
            return f"POSITION {struct.unpack('<I', payload)[0]} ms"
        if mtype == MSG["PLAY_STATE"]:
            return f"PLAY_STATE {PLAY[payload[0]]}"
        if mtype == MSG["VOLUME"]:
            return f"VOLUME {payload[0]}"
        if mtype == MSG["COVER_INFO"]:
            size, kind, digest, w, h = struct.unpack("<IBIHH", payload)
            return f"COVER_INFO size={size} kind={kind} hash={digest:08x} {w}x{h}"
        if mtype == MSG["SCAN_RESULT"]:
            addr, rssi, cls = struct.unpack_from("<6sbI", payload)
            return f"SCAN_RESULT {':'.join(f'{b:02X}' for b in addr)} rssi={rssi} class={cls:06x} {tlv(payload, 11)}"
    except (struct.error, IndexError):
        pass
    return f"{name} flags={flags:02x} seq={seq} payload={payload.hex()}"


def open_port(port: str):
    try:
        import serial
    except ImportError:
        sys.exit("pyserial is needed: pip install pyserial")
    return serial.Serial(port, 921600, timeout=0.05)


def transact(ser, mtype: int, payload: bytes = b"", wait: float = 1.0, flags: int = 0, until=None):
    """Sends one frame and prints every frame that comes back within `wait`
    seconds; stops early when `until` (a message type) arrives."""
    ser.write(encode(mtype, payload, seq=1, flags=flags))
    decoder = Decoder()
    deadline = time.monotonic() + wait
    while time.monotonic() < deadline:
        for frame in decoder.feed(ser.read(4096)):
            print(describe(*frame))
            if until is not None and frame[0] == until:
                return True
    return until is None


def selftest():
    # The same vector the C test pins: PING, seq 1, no payload.
    assert crc16(b"123456789") == 0x29B1
    wire = encode(MSG["PING"], seq=1)
    assert wire == bytes([END, 0x0D, 0x00, 0x01, 0x00, 0x00, 0x46, 0x07, END]), wire.hex()
    payload = bytes([1, 2, END, ESC, 0])
    d = Decoder()
    frames = list(d.feed(b"\x00\xff" + encode(0x87, payload, seq=9, flags=FLAG_WANT_ACK) + encode(MSG["PING"])[1:]))
    assert frames == [(0x87, FLAG_WANT_ACK, 9, payload), (MSG["PING"], 0, 0, b"")], frames
    assert d.dropped == 1
    status = struct.pack("<BB6sBIBB", 1, 2, bytes.fromhex("3dab55fa58fc"), 1, 44100, 1, 100) + bytes([0x02, 5]) + b"phone"
    assert "conn=connected" in describe(MSG["STATUS"], 0, 0, status)
    print("jbt.py selftest passed")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default="/dev/ttyUSB0")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("selftest")
    sub.add_parser("ping")
    sub.add_parser("status")
    sub.add_parser("monitor")
    sub.add_parser("name").add_argument("text")
    sub.add_parser("pairing").add_argument("state", choices=["on", "off"])
    sub.add_parser("mode").add_argument("mode", choices=MODE)
    sub.add_parser("volume").add_argument("level", type=int)
    sub.add_parser("key").add_argument("key", choices=["play", "pause", "stop", "next", "prev", "ff", "rew"])
    args = ap.parse_args()

    if args.cmd == "selftest":
        selftest()
        return
    ser = open_port(args.port)
    if args.cmd == "ping":
        ok = transact(ser, MSG["PING"], until=MSG["PONG"])
        sys.exit(0 if ok else "no PONG - is the module powered, and are TX/RX crossed?")
    elif args.cmd == "status":
        transact(ser, MSG["GET_STATUS"], until=MSG["STATUS"])
    elif args.cmd == "name":
        text = args.text.encode("utf-8")[:255]
        transact(ser, MSG["SET_NAME"], bytes([0x01, len(text)]) + text, flags=FLAG_WANT_ACK, until=MSG["ACK"])
    elif args.cmd == "pairing":
        transact(ser, MSG["PAIRING"], bytes([1 if args.state == "on" else 0]), flags=FLAG_WANT_ACK, until=MSG["ACK"])
    elif args.cmd == "mode":
        transact(ser, MSG["SET_MODE"], bytes([MODE.index(args.mode)]), wait=3.0, until=MSG["MODE_ACK"])
    elif args.cmd == "volume":
        transact(ser, MSG["SET_VOLUME"], bytes([max(0, min(127, args.level))]), flags=FLAG_WANT_ACK, until=MSG["ACK"])
    elif args.cmd == "key":
        keys = ["play", "pause", "stop", "next", "prev", "ff", "rew"]
        transact(ser, MSG["PASSTHROUGH"], bytes([keys.index(args.key)]), flags=FLAG_WANT_ACK, until=MSG["ACK"])
    elif args.cmd == "monitor":
        decoder = Decoder()
        print("listening; Ctrl+C to stop")
        try:
            while True:
                for frame in decoder.feed(ser.read(4096)):
                    print(time.strftime("%H:%M:%S"), describe(*frame))
        except KeyboardInterrupt:
            print(f"\n{decoder.dropped} frame(s) dropped")


if __name__ == "__main__":
    main()
