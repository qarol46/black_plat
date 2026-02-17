/**
 *  udp_tracked.cpp   (лог TX / RX в см/с и град/с)
 *  --------------------------------------------------
 */

#include "ros2_control_t21_hardware/udp_tracked.hpp"

#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <cstring>
#include <cmath>

using namespace tracked_platform_udp;

/* --------------------  утилита для hexdump  -------------------- */
namespace
{
void dump_hex(const void *data, size_t len, std::ostream &out = std::cout)
{
  const uint8_t *p = static_cast<const uint8_t*>(data);
  out << std::hex << std::setfill('0');
  for (size_t i = 0; i < len; ++i)
  {
    if (i % 16 == 0) out << "\n  ";
    out << std::setw(2) << int(p[i]) << ' ';
  }
  out << std::dec << '\n';
}
} // unnamed namespace
/* ------------------------------------------------------------------ */


/* ─────────────────────── создание сокета ────────────────────────── */
EthTrackedSocket::EthTrackedSocket(const char *listen_ip,
                                   uint16_t     listen_prt,
                                   const char *remote_ip,
                                   uint16_t     remote_prt)
{
  sock_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_ == -1) { perror("socket"); return; }

  /* —— приём —— */
  cliaddr_.sin_family      = AF_INET;
  cliaddr_.sin_port        = htons(listen_prt);
  cliaddr_.sin_addr.s_addr = inet_addr(listen_ip);
  if (bind(sock_, reinterpret_cast<sockaddr*>(&cliaddr_), sizeof(cliaddr_)) < 0)
  {
    perror("bind"); close(sock_); sock_ = -1; return;
  }

  /* —— отправка —— */
  servaddr_.sin_family      = AF_INET;
  servaddr_.sin_port        = htons(remote_prt);
  servaddr_.sin_addr.s_addr = inet_addr(remote_ip);

  /* таймаут приёма 10 мс */
  timeval tv{0, 10000};
  setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  std::cout << "[UDP] listen " << listen_ip << ':' << listen_prt
            << "  →  remote " << remote_ip  << ':' << remote_prt
            << "  — socket ready\n";
}

/* ─────────────────────── деструктор ─────────────────────────────── */
EthTrackedSocket::~EthTrackedSocket()
{
  if (sock_ != -1 && close(sock_) == -1) perror("close");
}

/* ─────────────────────── отправка пакета ────────────────────────── */
void EthTrackedSocket::sendCommand(float lin_vel_cm_s, float ang_vel_deg_s,
                                   float geom_deg,  bool geom_pos_mode)
{
  if (sock_ == -1) return;

  // 2) формируем пакет
  Packet128 pkt{};                    // вся структура зануляется
  pkt.geoMode = geom_pos_mode ? 0x01 : 0x00;
  pkt.linVel  = lin_vel_cm_s;     // см/с
  pkt.angVel  = ang_vel_deg_s;    // град/с
  pkt.geomPos = geom_deg;         // градусы

  // 3) шлём по UDP
  ssize_t n = sendto(sock_, &pkt, sizeof(pkt), MSG_CONFIRM,
                     reinterpret_cast<sockaddr*>(&servaddr_),
                     sizeof(servaddr_));
  if (n < 0) { perror("sendto"); return; }

  // 4) лог
  std::cout << "[UDP-TX] lin="   << lin_vel_cm_s  << " см/с"
/*             << "  ang="          << ang_vel_deg_s << " °/с" */
            << "  geom="         << geom_deg      << "°"
/*             << "  geoMode="      << int(pkt.geoMode) */
            << '\n';
  dump_hex(&pkt, sizeof(pkt));
}

/* ─────────────────────── приём пакета ───────────────────────────── */
bool EthTrackedSocket::receiveState(Packet128 &state)
{
  if (sock_ == -1) return false;

  ssize_t n = recvfrom(sock_, &state, sizeof(state), MSG_WAITALL,
                       nullptr, nullptr);
  if (n != sizeof(state)) return false;

  /* —— лог приёма —— */
/*   std::cout << "[UDP-RX] lin="  << state.linVel  << " см/с"
            << "  ang="         << state.angVel  << " °/с"
            << "  geom="        << state.geomPos << "°"
            << "  geoMode="     << int(state.geoMode) << '\n';
  dump_hex(&state, sizeof(state)); */

  return true;
}
