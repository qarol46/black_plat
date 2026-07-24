#!/usr/bin/env python3
import argparse
import math
import socket
import struct
import sys
import time


PACKET_SIZE = 128
LIN_OFFSET = 10
ANG_OFFSET = 14
GEOM_OFFSET = 21


def build_packet(
    angle_deg: float,
    geo_mode: int = 1,
    ch_mode: int = 1,
    dev_id: int = 0,
    addr: int = 0x23,
    upper_addr: int = 0xAA,
) -> bytes:
    packet = bytearray(PACKET_SIZE)
    packet[0] = addr & 0xFF
    packet[1] = geo_mode & 0xFF
    packet[2] = ch_mode & 0xFF
    packet[3] = dev_id & 0xFF
    packet[127] = upper_addr & 0xFF

    struct.pack_into('<f', packet, LIN_OFFSET, 0.0)
    struct.pack_into('<f', packet, ANG_OFFSET, 0.0)
    struct.pack_into('<f', packet, GEOM_OFFSET, float(angle_deg))
    return bytes(packet)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Send a flipper angle to the lower level over UDP.',
    )
    parser.add_argument(
        'angle',
        type=float,
        nargs='?',
        help='Flipper angle. Degrees by default.',
    )
    parser.add_argument(
        '--units',
        choices=('deg', 'rad'),
        default='deg',
        help='Units for the angle argument.',
    )
    parser.add_argument(
        '--ip',
        default='192.168.3.5',
        help='Target IP address.',
    )
    parser.add_argument(
        '--port',
        type=int,
        default=4001,
        help='Target UDP port.',
    )
    parser.add_argument(
        '--count',
        type=int,
        default=10,
        help='How many identical packets to send.',
    )
    parser.add_argument(
        '--rate',
        type=float,
        default=20.0,
        help='Packet send rate in Hz when count > 1.',
    )
    parser.add_argument(
        '--min-deg',
        type=float,
        default=10.0,
        help='Minimum allowed angle in degrees.',
    )
    parser.add_argument(
        '--max-deg',
        type=float,
        default=350.0,
        help='Maximum allowed angle in degrees.',
    )
    parser.add_argument(
        '--geo-mode',
        type=int,
        default=1,
        help='Geometry mode byte.',
    )
    parser.add_argument(
        '--ch-mode',
        type=int,
        default=1,
        help='Chassis mode byte.',
    )
    parser.add_argument(
        '--dev-id',
        type=int,
        default=0,
        help='Device id byte.',
    )
    parser.add_argument(
        '--addr',
        type=lambda x: int(x, 0),
        default=0x23,
        help='Board address byte, accepts decimal or hex.',
    )
    parser.add_argument(
        '--upper-addr',
        type=lambda x: int(x, 0),
        default=0xAA,
        help='Upper-level address byte, accepts decimal or hex.',
    )
    return parser.parse_args()


def read_angle_from_stdin(units: str) -> float:
    prompt = f'Enter flipper angle [{units}]: '
    try:
        return float(input(prompt).strip())
    except EOFError as exc:
        raise ValueError('No angle provided') from exc


def normalize_angle(angle: float, units: str) -> float:
    if units == 'rad':
        return math.degrees(angle)
    return angle


def clamp_angle(angle_deg: float, min_deg: float, max_deg: float) -> float:
    if min_deg > max_deg:
        raise ValueError('min_deg must be <= max_deg')
    return max(min(angle_deg, max_deg), min_deg)


def main() -> int:
    args = parse_args()

    angle = args.angle if args.angle is not None else read_angle_from_stdin(args.units)
    angle_deg = normalize_angle(angle, args.units)
    clamped_angle_deg = clamp_angle(angle_deg, args.min_deg, args.max_deg)

    if clamped_angle_deg != angle_deg:
        print(
            f'Angle {angle_deg:.2f} deg is out of range, clamped to {clamped_angle_deg:.2f} deg.',
            file=sys.stderr,
        )

    if args.count <= 0:
        raise ValueError('count must be > 0')
    if args.rate <= 0.0:
        raise ValueError('rate must be > 0')

    packet = build_packet(
        clamped_angle_deg,
        geo_mode=args.geo_mode,
        ch_mode=args.ch_mode,
        dev_id=args.dev_id,
        addr=args.addr,
        upper_addr=args.upper_addr,
    )

    interval = 1.0 / args.rate
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        for idx in range(args.count):
            sock.sendto(packet, (args.ip, args.port))
            print(
                f'[{idx + 1}/{args.count}] sent angle={clamped_angle_deg:.2f} deg '
                f'to {args.ip}:{args.port}'
            )
            if idx + 1 < args.count:
                time.sleep(interval)

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
