#include "esphome.h"
// #include "lwip.h"
// #include "lwip/udp.h"
#include "ESP8266WiFi.h"
#include "WiFiUdp.h"

#include "lwip/udp.h"
#include "lwip/inet.h"
#include "lwip/mem.h"
#include "include/UdpContext.h"

typedef union {
  uint16_t value;
  //  obj.addressable = (frameDescription & constants.ADDRESSABLE_BIT) !== 0;
  //  obj.tagged = (frameDescription & constants.TAGGED_BIT) !== 0;
  //  obj.origin = ((frameDescription & constants.ORIGIN_BITS) >> 14) !== 0;
  //  obj.protocolVersion = (frameDescription & constants.PROTOCOL_VERSION_BITS);
} FrameDescription;


typedef union {
  uint8_t value;
  //  obj.ackRequired = (frameAddressDescription & constants.ACK_REQUIRED_BIT) !== 0;
  //  obj.resRequired = (frameAddressDescription & constants.RESPONSE_REQUIRED_BIT) !== 0;
} FrameAddressDescription;

typedef enum {
  GetService = 2,
  StateService = 3,
  GetLabel = 23,
  StateLabel = 25,
  Acknowledgement = 45,
  SetPower = 117,
  StatePower = 118,
  GetDeviceChain = 701,
  StateDeviceChain = 702,
  GetTileState64 = 707,
  StateTileState64 = 711,
  SetTileState64 = 715
} LifxMsg;

namespace Packet {
#pragma pack(push, 1)
  typedef struct {
    /* frame */
    uint16_t size;
    uint16_t protocol:12;
    uint8_t  addressable:1;
    uint8_t  tagged:1;
    uint8_t  origin:2;
    uint32_t source;
    /* frame address */
    uint8_t  target[8];
    uint8_t  site[6];
    uint8_t  res_required:1;
    uint8_t  ack_required:1;
uint8_t  :6;
          uint8_t  sequence;
          /* protocol header */
          uint64_t time;
          uint16_t type;
uint16_t :16;
          /* variable length payload follows */
  } Header;


  typedef struct {
    char label[32];
  } StateLabel;

  typedef struct {
    uint16_t level;
    uint32_t duration;
  } SetPower;

  typedef struct {
    uint16_t level;
  } StatePower;

  typedef struct {
    int16_t accel_meas_x;
    int16_t accel_meas_y;
    int16_t accel_meas_z;
    int16_t reserved0;
    float user_x;
    float user_y;
    uint8_t width;
    uint8_t height;
    uint8_t reserved1;
    uint32_t device_version_vendor;
    uint32_t device_version_product;
    uint32_t device_version_version;
    uint64_t firmware_build;
    uint64_t reserved2;
    uint16_t firmware_version_minor;
    uint16_t firmware_version_major;
    uint32_t reserved3;
  } Tile;

  typedef struct StateDeviceChain {
    uint8_t start_index;
    Tile tile_devices[16];
    uint8_t total_count;

    StateDeviceChain(const std::vector<Tile> &ref):
      start_index(0),
      total_count(ref.size()) {

        std::copy(ref.begin(), ref.end(), tile_devices);
      }
  } StateDeviceChain;

  typedef struct {
    uint16_t hue;
    uint16_t saturation;
    uint16_t brightness;
    uint16_t kelvin;
  } HSBK;

  typedef struct {
    uint8_t tile_index;
    uint8_t length;
    uint8_t reserved;
    uint8_t x;
    uint8_t y;
    uint8_t width;
    uint32_t duration;
    HSBK colors[64];
  } SetTileState64;

  typedef struct {
    uint32_t signal;
    uint32_t tx;
    uint32_t rx;
    uint32_t reserved;
  } StateHostInfo;

  typedef struct {
    uint8_t service; // 5
    uint32_t port; // port
  } StateService;

  typedef struct {
    uint8_t tile_index;
    uint8_t length;
    uint8_t reserved;
    uint8_t x;
    uint8_t y;
    uint8_t width;
  } GetTileState64;

  typedef struct {
    uint8_t tile_index;
    uint8_t reserved;
    uint8_t x;
    uint8_t y;
    uint8_t width;
    HSBK colors[64];
  } StateTileState64;

  typedef struct {
    Packet::Header header;
    union msg {
      StateService stateService;
      StateLabel stateLabel;
      SetPower setPower;
      StatePower statePower;
      SetTileState64 setTileState64;
      GetTileState64 getTileState64;
      StateTileState64 stateTileState64;
    } msg;
  } Complete;

#pragma pack(pop)
}

typedef enum {
  FREE, RECV
} LifxPacketState;

class LifxLight : public PollingComponent, public Sensor {
  private:
    UdpContext* _conn;
    uint16_t port;
    int dropCount;
    LifxPacketState state;
    struct {
      Packet::Complete data;
      int size;
      uint32_t addr;
      uint16_t port;
    } recvPacket;
    Packet::Header header;
    Packet::StateHostInfo stateHostInfo;
    Packet::StateTileState64 stateTileState64;
  public:
    LifxLight():
      PollingComponent(1000),
      port(56700),
      dropCount(0),
      state(LifxPacketState::FREE)
  {
    memset(&header, 0, sizeof(header));
    memcpy(header.site, "LIFXV2", sizeof(header.site));
  }

    void appendLifxHeader(const Packet::Header &srcHeader, int type, int size) {
      header.size = size + sizeof(Packet::Header);
      header.type = type;
      header.protocol = 1024;
      header.addressable = true;
      header.tagged = false;
      header.origin = false;
      header.ack_required = false;
      header.res_required = false;
      header.sequence = srcHeader.sequence;
      header.source = srcHeader.source;
      /*
         FrameDescription frameDescription;
         uint32_t source;
         */
      for (int itfn = 0; itfn < 2; itfn++) {
        wifi_get_macaddr(itfn ? SOFTAP_IF : STATION_IF, header.target);
        if (header.target[0] || header.target[1] ||
            header.target[2] || header.target[3] ||
            header.target[4] || header.target[5])
        {
          break;
        }
      }
      /*
         FrameAddressDescription frameAddressDescription;
         */
      header.time = id(sntp_time).now().timestamp;
      _conn->append(reinterpret_cast<const char*>(&header), sizeof(Packet::Header));
    }

    void update() {
      /*
         ESP_LOGD("LifxLight", "update");
         struct ip_info ip_info;
         for (int itfn = 0; itfn < 2; itfn++)
         {
         }
         */
    }

    void updateUdp() {
      if (state != LifxPacketState::FREE) {
        dropCount++;
        return;
      }
      if (!_conn || !_conn->next()) {
        return;
      }
      recvPacket.size = _conn->read(reinterpret_cast<char *>(&(recvPacket.data)), sizeof(recvPacket.data));
      stateHostInfo.rx += recvPacket.size;
      recvPacket.addr = _conn->getRemoteAddress();
      recvPacket.port = _conn->getRemotePort();
      state = LifxPacketState::RECV;
    }
    void setup() override {
      _conn = new UdpContext;
      _conn->ref();
      ESP_LOGD("LifxLight", "start");
      if (!_conn->listen(IP_ADDR_ANY, port))
      {
        ESP_LOGE("LifxLight", "listen failed");
        return;
      }
      ESP_LOGD("LifxLight", "listen");
      // _conn->setMulticastTTL(MDNS_MULTICAST_TTL);
      _conn->onRx(std::bind(&LifxLight::updateUdp, this));
      // IPAddress broadcast(IPAddress(192, 168, 202, 255));
      // _conn->connect(broadcast, port);

    }

    void loop() override {
      // This will be called by App.loop()
      if (state == LifxPacketState::RECV) {
        if (recvPacket.size < sizeof(Packet::Header)) {
          ESP_LOGD("LifxLight", "illegal packet-%d-%d", ++dropCount, recvPacket.size);
          state = LifxPacketState::FREE;
          return;
        }
        ESP_LOGD("LifxLight", "recv-%d-%d %x:%d %d-%d", dropCount, recvPacket.size,
            recvPacket.addr, recvPacket.port,
            recvPacket.data.header.sequence, recvPacket.data.header.type);
        state = LifxPacketState::FREE;
        _conn->flush();
        switch (recvPacket.data.header.type) {
          case LifxMsg::GetService:
            {
              ESP_LOGD("LifxLight", "GetService");
              Packet::StateService one = { 1, port };
              sendStateService(recvPacket.data.header, one);
              _conn->send(IPAddress(recvPacket.addr), recvPacket.port);
              _conn->flush();
              Packet::StateService five = { 5, port };
              sendStateService(recvPacket.data.header, five);
              _conn->send(IPAddress(recvPacket.addr), recvPacket.port);
            }
            break;
          case LifxMsg::GetLabel:
            {
              ESP_LOGD("LifxLight", "GetLabel");
              Packet::StateLabel stateLabel;
              strncpy(stateLabel.label, App.get_name().c_str(), sizeof(stateLabel.label));
              sendStateLabel(recvPacket.data.header, stateLabel);
              _conn->send(IPAddress(recvPacket.addr), recvPacket.port);
            }
            break;
          case LifxMsg::GetDeviceChain:
            {
              ESP_LOGD("LifxLight", "GetDeviceChain");
              Packet::StateDeviceChain stateDeviceChain({
                  {
                  0, 0, 0,
                  0,
                  0.0, 0.0,
                  13, 4,
                  0,
                  4711, 4711, 4711,
                  0x47114711,
                  0,
                  11, 47,
                  0
                  }
                  });
              sendStateDeviceChain(recvPacket.data.header, stateDeviceChain);
              _conn->send(IPAddress(recvPacket.addr), recvPacket.port);
            }
            break;
          case LifxMsg::SetPower:
            ESP_LOGD("LifxLight", "SetPower:%d-%d-%d",
                recvPacket.data.header.res_required,
                recvPacket.data.msg.setPower.level,
                recvPacket.data.msg.setPower.duration);

            if (recvPacket.data.header.res_required) {
              Packet::StatePower statePower = { recvPacket.data.msg.setPower.level };
              sendStatePower(recvPacket.data.header, statePower);
              _conn->send(IPAddress(recvPacket.addr), recvPacket.port);
            }
            break;
          case LifxMsg::SetTileState64:
            ESP_LOGD("LifxLight", "SetTileState64:%d-%d-%d-%d-%d-%d",
                recvPacket.data.msg.setTileState64.tile_index,
                recvPacket.data.msg.setTileState64.length,
                recvPacket.data.msg.setTileState64.x,
                recvPacket.data.msg.setTileState64.y,
                recvPacket.data.msg.setTileState64.width,
                recvPacket.data.msg.setTileState64.duration);
            stateTileState64.tile_index = recvPacket.data.msg.setTileState64.tile_index;
            stateTileState64.reserved = recvPacket.data.msg.setTileState64.reserved;
            stateTileState64.x = recvPacket.data.msg.setTileState64.x;
            stateTileState64.y = recvPacket.data.msg.setTileState64.y;
            stateTileState64.width = recvPacket.data.msg.setTileState64.width;
            *(stateTileState64.colors) = *(recvPacket.data.msg.setTileState64.colors);
            break;

          case LifxMsg::GetTileState64:
            ESP_LOGD("LifxLight", "GetTileState64:%d-%d-%d-%d-%d",
                recvPacket.data.msg.getTileState64.tile_index,
                recvPacket.data.msg.getTileState64.length,
                recvPacket.data.msg.getTileState64.x,
                recvPacket.data.msg.getTileState64.y,
                recvPacket.data.msg.getTileState64.width);
            sendStateTileState64(recvPacket.data.header, stateTileState64);
            _conn->send(IPAddress(recvPacket.addr), recvPacket.port);
            break;
          break;

        }
        if (recvPacket.data.header.ack_required) {
          _conn->flush();
          sendNoData(recvPacket.data.header, LifxMsg::Acknowledgement);
          _conn->send(IPAddress(recvPacket.addr), recvPacket.port);
        }
      }
    }
    void sendStateTileState64(const Packet::Header &header, const Packet::StateTileState64 &sts) {
      ESP_LOGD("LifxLight", "sendStateTileState64:%d:%d", sizeof(Packet::Header), sizeof(sts));
      appendLifxHeader(header, LifxMsg::StateTileState64, sizeof(sts));
      _conn->append(reinterpret_cast<const char*>(&sts), sizeof(sts));
      stateHostInfo.tx += sizeof(Packet::Header) + sizeof(sts);
    }
    void sendNoData(const Packet::Header &header, LifxMsg type) {
      ESP_LOGD("LifxLight", "sendNoData:%d:%d", sizeof(Packet::Header), type);
      appendLifxHeader(header, LifxMsg::Acknowledgement, 0);
      stateHostInfo.tx += sizeof(Packet::Header);
    }
    void sendStatePower(const Packet::Header &header, const Packet::StatePower &sp) {
      ESP_LOGD("LifxLight", "sendStatePower:%d:%d", sizeof(Packet::Header), sizeof(sp));
      appendLifxHeader(header, LifxMsg::StatePower, sizeof(sp));
      _conn->append(reinterpret_cast<const char*>(&sp), sizeof(sp));
      stateHostInfo.tx += sizeof(Packet::Header) + sizeof(sp);
    }
    void sendStateService(const Packet::Header &header, const Packet::StateService &ss) {
      ESP_LOGD("LifxLight", "sendStateService:%d:%d", sizeof(Packet::Header), sizeof(ss));
      appendLifxHeader(header, LifxMsg::StateService, sizeof(ss));
      _conn->append(reinterpret_cast<const char*>(&ss), sizeof(ss));
      stateHostInfo.tx += sizeof(Packet::Header) + sizeof(ss);
    }

    void sendStateLabel(const Packet::Header &header, const Packet::StateLabel &sl) {
      ESP_LOGD("LifxLight", "sendStateLabel:%d:%d", sizeof(Packet::Header), sizeof(sl));
      appendLifxHeader(header, LifxMsg::StateLabel, sizeof(sl));
      _conn->append(reinterpret_cast<const char*>(&sl), sizeof(sl));
      stateHostInfo.tx += sizeof(Packet::Header) + sizeof(sl);
    }

    void sendStateDeviceChain(const Packet::Header &header, const Packet::StateDeviceChain &sdc) {
      ESP_LOGD("LifxLight", "sendStateDeviceChain:%d:%d", sizeof(Packet::Header), sizeof(sdc));
      appendLifxHeader(header, LifxMsg::StateDeviceChain, sizeof(sdc));
      _conn->append(reinterpret_cast<const char*>(&sdc), sizeof(sdc));
      stateHostInfo.tx += sizeof(Packet::Header) + sizeof(sdc);
    }

};
