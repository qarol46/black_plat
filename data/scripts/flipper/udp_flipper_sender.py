import socket
import struct
import time

UDP_IP = "192.168.3.5"
UDP_PORT = 4001
LOCAL_PORT = 4001

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", LOCAL_PORT))


def send_flipper_angle(angle,
                        lin_vel=0.0,
                        ang_vel=0.0):
    pkt = bytearray(128)

    # ─── заголовок ───
    pkt[0] = 0x23        # addr
    pkt[1] = 0x01        # geoMode = POSITION
    pkt[2] = 0x01        # chMode
    pkt[3] = 0x00        # devId

    # ─── скорости ───
    struct.pack_into("<f", pkt, 10, lin_vel)
    struct.pack_into("<f", pkt, 14, ang_vel)

    # ─── ГЛАВНОЕ: угол флипперов ───
    struct.pack_into("<f", pkt, 21, angle)

    # ─── хвост ───
    pkt[127] = 0xAA

    sock.sendto(pkt, (UDP_IP, UDP_PORT))


if __name__ == "__main__":
    print("Send single flipper angle")

    while True:
        send_flipper_angle(
            angle = 0  # радианы (≈28.6°) ИЛИ градусы — как в прошивке
        )
        time.sleep(0.02)
