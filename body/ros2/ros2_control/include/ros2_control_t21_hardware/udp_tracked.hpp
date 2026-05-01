/**
 *  udp_tracked.hpp
 *  ----------------
 *  Библиотека для обмена по UDP с гусеничной платформой.
 *  Формирует/разбирает 128-байтный пакет; линейная скорость - в см/с,
 *  угловая — в град/с, геометрия — в градусах.
 */
#pragma once
#include <cstdint>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

namespace tracked_platform_udp
{

// ─────────── 128-байтный пакет команд/состояния ───────────
#pragma pack(push, 1)
struct Packet128
{
  uint8_t  addr    = 0x23;   // [0]  адрес платы
  uint8_t  geoMode = 0x00;   // [1]  режим геометрии (0-PWM / 1-Controller)
  uint8_t  chMode  = 0x01;   // [2]  режим шасси   (0-PWM / 1-Controller)
  uint8_t  devId   = 0x00;   // [3]  номер устройства
  uint8_t  rsv1[6]{};        // [4-9]  резерв

  float    linVel  = 0.0f;   // [10-13] линейная скорость  (см/с)
  float    angVel  = 0.0f;   // [14-17] угловая скорость   (град/с)
  uint8_t  rsv2[3]{};        // [18-20] резерв

  float    geomPos = 0.0f;   // [21-24] положение геометрии (град)
  uint8_t  rsv3[4]{};        // [25-28] резерв
  uint8_t  rsv4[4]{};        // [29-32] резерв
  uint8_t  rsv5[10]{};       // [33-42] резерв

  uint8_t  cam[19]{};        // [43-61] резерв камеры
  uint8_t  future[65]{};     // [62-126] свободный резерв

  uint8_t  upperAddr = 0xAA; // [127] адрес верхнего уровня
};
#pragma pack(pop)

static_assert(sizeof(Packet128) == 128, "Packet size must be 128 bytes");

// ─────────── UDP-обёртка ───────────
class EthTrackedSocket
{
public:
  EthTrackedSocket(const char *listen_ip  = "0.0.0.0",
                   uint16_t     listen_prt = 4001,
                   const char *remote_ip  = "192.168.3.5",
                   uint16_t     remote_prt = 4001);
                   /*const char *remote_ip  = "127.0.0.1",
                   uint16_t     remote_prt = 8888);*/
  ~EthTrackedSocket();

  // lin_vel — см/с, ang_vel — град/с, geom_deg — градусы
  void sendCommand(float lin_vel, float ang_vel,
                   float geom_deg, bool geom_pos_mode);

  bool receiveState(Packet128 &state);   // true, если получен полный пакет

private:
  int           sock_{-1};
  sockaddr_in   cliaddr_{}, servaddr_{};
};

} // namespace tracked_platform_udp
