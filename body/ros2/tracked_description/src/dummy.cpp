/**
 *  t21_udp_emulator.cpp
 *  --------------------------------------------------------------------------
 *  Эмулятор гусеничной платформы Т-21.
 *  Принимает «командный» 128-байтный пакет (см/с, °/с, °),
 *  вычисляет обороты бортов (об/мин) и отсылает «пакет-состояние»
 *  того же размера, как будто это реальная платформа.
 *
 *  Сборка:
 *      g++ -std=c++17 -O2 t21_udp_emulator.cpp -o t21_udp_emulator
 *
 *  Запуск (listen_ip:port  remote_ip:port):
 *      ./t21_udp_emulator            # 0.0.0.0:4001 → 192.168.3.5:4001
 *      ./t21_udp_emulator 0.0.0.0 8888 127.0.0.1 8888
 *  --------------------------------------------------------------------------
 */
#include <arpa/inet.h>
#include <unistd.h>
#include <cmath>
#include <cstring>
#include <iostream>

/* ─────────── геометрия шасси ─────────── */
constexpr double R = 0.065;   // м  ─ радиус «колеса»
constexpr double L = 0.37;   // м  ─ база

/* ─────────── общий 128-байтный пакет ─────────── */
#pragma pack(push, 1)
struct Packet128
{
  uint8_t  addr    = 0x23;   // [0]  (в ответе будет 0xAA)
  uint8_t  geoMode = 0x01;   // [1]
  uint8_t  chMode  = 0x01;   // [2]
  uint8_t  devId   = 0x00;   // [3]
  uint8_t  rsv1[6]{};

  float    linVel  = 0.0f;   // [10-13]  TX: см/с,  RX: ωL об/мин
  float    angVel  = 0.0f;   // [14-17]  TX: °/с,   RX: ωR об/мин
  uint8_t  rsv2[3]{};

  float    geomPos = 0.0f;   // [21-24]  градус
  uint8_t  rsv3[4]{};
  uint8_t  rsv4[4]{};
  uint8_t  rsv5[10]{};

  uint8_t  cam[19]{};
  uint8_t  future[65]{};

  uint8_t  upperAddr = 0xAA; // [127]
};
#pragma pack(pop)

static_assert(sizeof(Packet128) == 128, "Packet must be 128 bytes");

/* ─────────── утилита hexdump ─────────── */
void dump_hex(const void* data, size_t len)
{
  const uint8_t* p = static_cast<const uint8_t*>(data);
  std::ios old_state(nullptr);
  old_state.copyfmt(std::cout);
  std::cout.setf(std::ios::hex, std::ios::basefield);
  std::cout.fill('0');
  for (size_t i = 0; i < len; ++i) {
    if (i % 16 == 0) std::cout << "\n  ";
    std::cout.width(2);
    std::cout << int(p[i]) << ' ';
  }
  std::cout << std::endl;
  std::cout.copyfmt(old_state);
}

/* ─────────── main ─────────── */
int main(int argc, char* argv[])
{
  /* ------------------- параметры запуска ------------------- */
  const char* listen_ip  = (argc > 1) ? argv[1] : "127.0.0.1";
  int         listen_prt = (argc > 2) ? std::stoi(argv[2]) : 8888;
  const char* remote_ip  = (argc > 3) ? argv[3] : "0.0.0.0";
  int         remote_prt = (argc > 4) ? std::stoi(argv[4]) : 4001;

  /* -------------------- сокет ------------------------------ */
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) { perror("socket"); return 1; }

  sockaddr_in srv{};
  srv.sin_family = AF_INET;
  srv.sin_addr.s_addr = inet_addr(listen_ip);
  srv.sin_port = htons(static_cast<uint16_t>(listen_prt));
  if (bind(sock, reinterpret_cast<sockaddr*>(&srv), sizeof(srv)) < 0) {
    perror("bind"); return 1;
  }

  sockaddr_in cli{};
  cli.sin_family = AF_INET;
  cli.sin_addr.s_addr = inet_addr(remote_ip);
  cli.sin_port = htons(static_cast<uint16_t>(remote_prt));

  std::cout << "T21 UDP emulator  "
            << listen_ip << ':' << listen_prt
            << "  →  " << remote_ip << ':' << remote_prt << std::endl;

  /* ------------------- цикл приёма/ответа ------------------ */
  Packet128 rx{}, tx{};
  while (true)
  {
    sockaddr_in peer{};
    socklen_t   plen = sizeof(peer);
    ssize_t n = recvfrom(sock, &rx, sizeof(rx), 0,
                         reinterpret_cast<sockaddr*>(&peer), &plen);
    if (n != sizeof(rx)) { if (n < 0) perror("recvfrom"); continue; }

    /* ----- лог приёма ----- */
    std::cout << "\n[RX] lin=" << rx.linVel << " см/с"
              << "  ang=" << rx.angVel << " °/с"
              << "  geom=" << rx.geomPos << "°";
    dump_hex(&rx, sizeof(rx));

    /* 1. перевод см/с → м/с, °/с → рад/с */
    double lin_si  = 0.01 * rx.linVel;                   // м/с
    double ang_si  = M_PI/180.0 * rx.angVel;             // рад/с

    /* 2. обратная дифф-кинематика → ωL/ωR (рад/с) */
    double omega_l = ((lin_si - 0.5 * ang_si * L) / R);
    double omega_r = ((lin_si + 0.5 * ang_si * L) / R);

    /* 3. рад/с → об/мин */
    double rad2rpm = 60.0 / (2.0 * M_PI); //нужно ли тут выставлять ntohs?
    float rpm_l = static_cast<float>(omega_l * rad2rpm);
    float rpm_r = static_cast<float>(omega_r * rad2rpm);

    /* 4. формируем пакет-ответ */
    tx = {};                       // зануляем
    tx.addr     = 0xAA;
    tx.linVel   = rpm_l;           // ωL
    tx.angVel   = rpm_r;           // ωR
    tx.geomPos  = rx.geomPos;      // просто эхо угла
    // остальные байты оставляем нулевыми

    /* ----- лог ответа ----- */
    std::cout << "\n[TX] ωL=" << rpm_l << " об/мин"
              << "  ωR=" << rpm_r << " об/мин"
              << "  geom=" << tx.geomPos << "°";
    dump_hex(&tx, sizeof(tx));

    /* 5. отправляем назад тому же отправителю */
    sendto(sock, &tx, sizeof(tx), 0,
           reinterpret_cast<sockaddr*>(&peer), plen);
  }
}
