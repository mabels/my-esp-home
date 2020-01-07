
#include "WiFiUdp.h"
#include "lwip/udp.h"
#include "lwip/inet.h"
#include "lwip/mem.h"
#if defined(ESP8266)
#include "ESP8266WiFi.h"
#include "include/UdpContext.h"
#else
class UdpContext {
  private: 
    uint16_t packetLen;
    uint8_t packet[1536];
  public:
    UdpContext(): packetLen(0) {
    }

    typedef std::function<void(void)> rxhandler_t;

    void ref() {
    }

    void unref() {
    }

    void onRx(rxhandler_t handler) {
    }

    bool listen(const ip_addr_t* addr, uint16_t port) {
      return false;
    }

    void send(IPAddress addr, int port) {
      udp.beginPacket(addr, port);
      udp.write(packet, packetLen);
      udp.endPacket();
    }

    void append(const char *buf, int len) {
      bool ret = true;
      if (packetLen + len > sizeof(packet)) {
        len = sizeof(packet) - packetLen;
        ret = false;
      }
      memcpy(packet + packetLen, buf, len);
      packetLen += len;
      return ret;
    }

    int read(char *buf, int len) {
      return 0;
    }

    uint32_t getRemoteAddress() const {
      return 0;
    }

    uint16_t getRemotePort() const {
      return 0;
    }

    void flush() {
      packetLen = 0;
    }

    bool next() {
      return false;
    }
};
#endif

