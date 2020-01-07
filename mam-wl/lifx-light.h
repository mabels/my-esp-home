#include "esphome.h"
#include "udp-context.h"

#include <NeoPixelBus.h>
#include <NeoPixelBrightnessBus.h>

typedef enum {
  GetService = 2,
  StateService = 3,
  GetHostInfo = 12,
  StateHostInfo = 13,
  GetHostFirmware = 14,
  StateHostFirmware = 15,
  GetWifiInfo = 16,
  StateWifiInfo = 17,
  GetWifiFirmware = 18,
  StateWifiFirmware = 19,
  GetLabel = 23,
  SetLabel = 24,
  StateLabel = 25,
  GetVersion = 32,
  StateVersion = 33,
  Acknowledgement = 45,
  GetLocation = 48,
  SetLocation = 49,
  StateLocation = 50,
  GetGroup = 51,
  SetGroup = 52,
  StateGroup = 53,
  Get0x36 = 0x36,
  Set0x37 = 0x37,
  State0x38 = 0x38,
  Get = 101,
  SetColor = 102,
  State = 107,
  GetPower = 116,
  SetPower = 117,
  StatePower = 118,
  SetWaveformOptional = 119,
  GetDeviceChain = 701,
  StateDeviceChain = 702,
  GetTileState64 = 707,
  StateTileState64 = 711,
  SetTileState64 = 715,
  Get0x2ce = 0x2ce,
  State0x2d0 = 0x2d0,
  Get0x2d1 = 0x2d1,
  State0x2d3 = 0x2d3
} LifxMsg;

namespace Packet {
#pragma pack(push, 1)
  typedef struct Header {
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
    uint8_t  reserved0:6;
    uint8_t  sequence;
    /* protocol header */
    uint64_t time;
    uint16_t type;
    uint16_t reserved1:16;
    /* variable length payload follows */
    void init() {
      size = protocol = addressable = tagged = origin = source = res_required = 0;
      ack_required = reserved0 = sequence = time = type = reserved1 = 0;
      memcpy(site, "LIFXV2", sizeof(site));
#ifdef ARDUINO_ARCH_ESP32
      esp_efuse_mac_get_default(target);
#endif
#ifdef ARDUINO_ARCH_ESP8266
      WiFi.macAddress(target);
#endif
    }
  } Header;

  typedef struct {
    uint16_t hue;
    uint16_t saturation;
    uint16_t brightness;
    uint16_t kelvin;
    void init() {
      hue = saturation = brightness = kelvin = 0;
    }
  } HSBK;

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
    void init() {
      accel_meas_x = 0;
      accel_meas_y = 0;
      accel_meas_z = 0;
      reserved0 = 0;
      user_x = 0;
      user_y = 0;
      width = 0;
      height = 0;
      reserved1 = 0;
      device_version_vendor = 0;
      device_version_product = 0;
      device_version_version = 0;
      firmware_build = 0;
      reserved2 = 0;
      firmware_version_minor = 0;
      firmware_version_major = 0;
      reserved3 = 0;
    }
  } Tile;


  typedef struct State0x2d0 {
    uint8_t reserved[223 - sizeof(Header)];
    void init() {
      memset(reserved, 0, sizeof(reserved));
    }
  } State0x2d0;

  typedef struct State0x2d3 {
    // 0040   01 2d 78 47 f5 63 78 e0 01 00 d3 02 00 00 00 00
    // 0050   01 04 03 20 03
    uint16_t reserved;
    uint8_t one;
    uint8_t four;
    uint8_t three0;
    uint8_t twenty;
    uint8_t three1;
    void init() {
      reserved = 0;
      one = 1;
      four = 4;
      three0 = 3;
      twenty = 0x20;
      three1 = 3;
    }
  } State0x2d3;

  typedef struct SetWaveformOptional {
    uint8_t reserved; // unsigned 8-bit integer
    boolean transient; // boolean
    HSBK color;
    uint32_t period; //	unsigned 32-bit integer
    uint32_t cycles; //	32-bit float
    int16_t skew_ratio; //	signed 16-bit integer
    uint8_t waveform; //	unsigned 8-bit integer
    uint8_t set_hue; //	8-bit integer as 0 or 1
    uint8_t set_saturation; //	8-bit integer as 0 or 1
    uint8_t set_brightness; //	8-bit integer as 0 or 1
    uint8_t set_kelvin; //	8-bit integer as 0 or 1
    void init() {
      color.init();
      transient = false;
      reserved = period = cycles = skew_ratio = waveform = set_hue = set_saturation = set_brightness = set_kelvin = 0;
    }
  } SetWaveformOptional;

  typedef struct StateLabel {
    char label[32];
    void init() {
      memset(label, 0, sizeof(label));
    }
  } StateLabel;
  typedef StateLabel SetLabel;

  typedef struct {
    uint16_t level;
    uint32_t duration;
  } SetPower;

  typedef struct {
    uint16_t level;
  } StatePower;


  typedef struct StateDeviceChain {
    uint8_t start_index;
    Tile tile_devices[16];
    uint8_t total_count;

    void init() {
      start_index = total_count = 0;
      for (int i = 0; i < sizeof(tile_devices)/sizeof(Tile); ++i) {
        tile_devices[i].init();
      }
    }

    StateDeviceChain(const std::vector<Tile> &ref) {
      init();
      start_index = 0;
      total_count = ref.size();
      std::copy(ref.begin(), ref.end(), tile_devices);
    }
  } StateDeviceChain;

  typedef struct State {
    HSBK color;
    uint16_t reserved0;
    uint16_t power;
    char label[32];
    uint64_t reserved1;
    void init() {
      color.hue = 307;
      color.saturation = 100;
      color.brightness = 50;
      color.kelvin = 4000;
      reserved0 = 0;
      power = 0;
      memset(label, 0, sizeof(label));
      reserved1 = 0;
    }
  } State;

  typedef struct StateUuidLabel {
    union {
      uint8_t location[16];
      uint8_t group[16];
      uint8_t _0x38[16];
    };
    char label[32];
    uint64_t updated_at;
    void init() {
      memset(location, 0, sizeof(location));
      memset(label, 0, sizeof(label));
      updated_at = 0;
    }
  } StateUuidLabel;
  typedef StateUuidLabel StateLocation;
  typedef StateUuidLabel SetLocation;
  typedef StateUuidLabel StateGroup;
  typedef StateUuidLabel SetGroup;
  typedef StateUuidLabel State0x38;
  typedef StateUuidLabel Set0x37;

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
    float signal;
    uint32_t tx;
    uint32_t rx;
    uint16_t mcuTemerature;
    void init() {
      signal = 0;
      tx = rx = mcuTemerature = 0;
    }
  } StateHostInfo;
  typedef StateHostInfo StateWifiInfo;

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
    uint8_t reserved;
    HSBK color;
    uint32_t duration;
  } SetColor;

  typedef struct {
    uint32_t vendor;
    uint32_t product;
    uint32_t version;
  } StateVersion;

  typedef struct {
    uint64_t build;
    uint64_t reserved;
    uint16_t version_minor;
    uint16_t version_major;
  } StateWifiFirmware;
  typedef StateWifiFirmware  StateHostFirmware;

  typedef struct {
    uint8_t tile_index;
    uint8_t reserved;
    uint8_t x;
    uint8_t y;
    uint8_t width;
    HSBK colors[64];
    void init() {
      tile_index = reserved = x = y = width;
      for (int i = 0; i < sizeof(colors)/sizeof(HSBK); ++i) {
        colors[i].init();
      }
    }
  } StateTileState64;

  typedef struct Complete {
    Packet::Header header;
    union {
      StateService stateService;
      StateLabel stateLabel;
      SetPower setPower;
      StatePower statePower;
      StateLocation stateLocation;
      SetLabel setLabel;
      Set0x37 set0x37;
      SetLocation setLocation;
      StateGroup stateGroup;
      SetGroup setGroup;
      SetColor setColor;
      SetTileState64 setTileState64;
      GetTileState64 getTileState64;
      StateTileState64 stateTileState64;
      SetWaveformOptional setWaveformOptional;
    } msg;
  } Complete;


#pragma pack(pop)
}

typedef enum {
  FREE, RECV
} LifxPacketState;

typedef struct {
  Packet::StateLocation location;
  Packet::StateGroup group;
  Packet::StateLabel label;
  Packet::State0x38 state0x38;
  void init() {
    location.init();
    group.init();
    label.init();
    strncpy(label.label, App.get_name().c_str(), sizeof(label.label));
    state0x38.init();
  }
} LifxFlashState;

typedef struct {
  Packet::StateHostInfo hostInfo;
  Packet::StateTileState64 tileState64;
  Packet::State state;
  Packet::SetWaveformOptional waveFormOptional;
  void init() {
    hostInfo.init();
    tileState64.init();
    state.init();
    waveFormOptional.init();
  }
} RunState;

class LifxLight : public PollingComponent, public Sensor {
  private:
    const char *instance;
    UdpContext* _conn;
    const uint16_t port;
    int dropCount;
    LifxPacketState packetState;
    struct RecvPacket {
      Packet::Complete data;
      int size;
      uint32_t addr;
      uint16_t port;
    } recvPacket;

    Packet::Header header;
    RunState runState;

    ESPPreferenceObject rtc;
    LifxFlashState flashState;
    bool flashUpdate;
    const int hash;
    NeoPixelBus<NeoGrbwFeature, NeoEsp8266Uart1Sk6812Method> strip;
    //   NeoEsp8266UartMethodBase<
    //     NeoEsp8266UartSpeedSk6812,
    //     NeoEsp8266Uart<UartFeature0, NeoEsp8266UartContext>,
    //     NeoEsp8266UartNotInverted>
    //  > strip;
    // NeoPixelBrightnessBus<NeoGrbFeature, Neo800KbpsMethod> strip;
    //new NeoPixelBus<T_COLOR_FEATURE, T_METHOD>(count_pixels, pin))
    // neopixelbus::NeoPixelRGBWLightOutput<NeoEsp8266Uart1Sk6812Method> strip;
  public:
    LifxLight():
      PollingComponent(1000),
      instance("LifxLight"),
      _conn(0),
      port(56700),
      dropCount(0),
      packetState(LifxPacketState::FREE),
      hash(0x11f810CA),
      strip(4 * 13, 2)
  {
 /*
  #   default_transition_length: 1s
  #   type: GRBW
  #   pin: GPIO2
  #   variant: SK6812
  #   method: ESP8266_UART1
  #   num_leds: 52
  #   name: "leds"
  #   id: "leds
  #    NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);
  */
    // strip.add_leds(52, 2);
    // strip.set_pixel_order(neopixelbus::ESPNeoPixelOrder::GRBW);
    strip.Begin();
    strip.Show();

    header.init();
    runState.init();
    rtc = global_preferences.make_preference<LifxFlashState>(hash);
    if (!rtc.load(&flashState)) {
      ESP_LOGD(instance, "FlashInitialzed");
      flashState.init();
    } else {
      ESP_LOGD(instance, "FlashLoad");
    }
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
      header.time = srcHeader.time + 100; // id(sntp_time).now().timestamp;
      _conn->append(reinterpret_cast<const char*>(&header), sizeof(Packet::Header));
    }

    void update() {
      if (flashUpdate) {
        flashUpdate = false;
        ESP_LOGD(instance, "Write-Flash");
        rtc.save(&flashState);
      }
    }

    void updateUdp() {
      if (packetState != LifxPacketState::FREE) {
        dropCount++;
        return;
      }
      if (!_conn || !_conn->next()) {
        return;
      }
      recvPacket.size = _conn->read(reinterpret_cast<char *>(&(recvPacket.data)), sizeof(recvPacket.data));
      runState.hostInfo.rx += recvPacket.size;
      recvPacket.addr = _conn->getRemoteAddress();
      recvPacket.port = _conn->getRemotePort();
      packetState = LifxPacketState::RECV;
    }
    void setup() override {
      _conn = new UdpContext;
      _conn->ref();
      // ESP_LOGD(instance, "start");
      if (!_conn->listen(IP_ADDR_ANY, port))
      {
        ESP_LOGE(instance, "listen failed");
        return;
      }
      _conn->onRx(std::bind(&LifxLight::updateUdp, this));
    }

    void processLifx() {
      _conn->flush();
      switch (recvPacket.data.header.type) {
        case LifxMsg::GetService:
          {
            // ESP_LOGD(instance, "GetService");
            Packet::StateService one = { 1, port };
            sendStateService(recvPacket.data.header, one);
            _conn->send(IPAddress(recvPacket.addr), recvPacket.port);
            _conn->flush();
            Packet::StateService five = { 5, port };
            sendStateService(recvPacket.data.header, five);
            _conn->send(IPAddress(recvPacket.addr), recvPacket.port);
          }
          break;
        case LifxMsg::GetDeviceChain:
          {
            // ESP_LOGD(instance, "GetDeviceChain");
            Packet::StateDeviceChain stateDeviceChain({
                {
                0, 0, 0,
                0,
                0.0, 0.0,
                8, 8,
                50,
                1, 55, 10,
                0x157f1308f338ec00,
                0x157f1308f338ec00,
                50, 3,
                1
                }
                });
            sendStateDeviceChain(recvPacket.data.header, stateDeviceChain);
            _conn->send(IPAddress(recvPacket.addr), recvPacket.port);
          }
          break;
        case LifxMsg::SetPower:

          runState.state.power = recvPacket.data.msg.setPower.level;
          // strip.SetPixelColor(10, )
          // #define colorSaturation 255 // saturation of color constants
          // RgbColor red(colorSaturation, 0, 0);
          // RgbColor green(0, colorSaturation, 0);
          // RgbColor blue(0, 0, colorSaturation);
          // strip.SetBrightness(runState.state.power >> 8);
          {
            HsbColor hsb(runState.state.color.hue/65535.0,
                    runState.state.color.saturation/65535.0,
                    runState.state.color.brightness/65535.0);
            RgbwColor rgbw(hsb);
            ESP_LOGD(instance, "SetPower:%d-%d-%d:%d-%d-%d %d/%d/%d/%d",
              recvPacket.data.header.res_required,
              recvPacket.data.msg.setPower.level,
              recvPacket.data.msg.setPower.duration,
              runState.state.color.hue,
              runState.state.color.saturation,
              runState.state.color.brightness,
              rgbw.R, rgbw.G, rgbw.B, rgbw.W
              );
            for (int i = 0; i < 52; ++i) {
              strip.SetPixelColor(i, rgbw);
            }
            strip.Show();
          }
    /*
          id(leds).make_call()
              .set_state(recvPacket.data.msg.setPower.level > 0)
              .set_brightness(recvPacket.data.msg.setPower.level / 65536.0)
              // .set_transition_length(recvPacket.data.msg.setPower.duration)
              .set_transition_length(0)
              .perform();
    */
          if (!recvPacket.data.header.res_required) {
            break;
          }
        case LifxMsg::GetPower:
          {
            // ESP_LOGD(instance, "GetPower:%d", recvPacket.data.header.res_required);
            Packet::StatePower statePower = { runState.state.power };
            sendStatePower(recvPacket.data.header, statePower);
            _conn->send(IPAddress(recvPacket.addr), recvPacket.port);
          }
          break;
        case LifxMsg::SetTileState64:
          // ESP_LOGD(instance, "SetTileState64:%d-%d-%d-%d-%d-%d",
          //     recvPacket.data.msg.setTileState64.tile_index,
          //     recvPacket.data.msg.setTileState64.length,
          //     recvPacket.data.msg.setTileState64.x,
          //     recvPacket.data.msg.setTileState64.y,
          //     recvPacket.data.msg.setTileState64.width,
          //     recvPacket.data.msg.setTileState64.duration);
          runState.tileState64.tile_index = recvPacket.data.msg.setTileState64.tile_index;
          runState.tileState64.reserved = recvPacket.data.msg.setTileState64.reserved;
          runState.tileState64.x = recvPacket.data.msg.setTileState64.x;
          runState.tileState64.y = recvPacket.data.msg.setTileState64.y;
          runState.tileState64.width = recvPacket.data.msg.setTileState64.width;
          memcpy(runState.tileState64.colors, recvPacket.data.msg.setTileState64.colors,
              sizeof(runState.tileState64.colors));
          break;

        case LifxMsg::GetTileState64:
          // ESP_LOGD(instance, "GetTileState64:%d-%d-%d-%d-%d",
          //     recvPacket.data.msg.getTileState64.tile_index,
          //     recvPacket.data.msg.getTileState64.length,
          //     recvPacket.data.msg.getTileState64.x,
          //     recvPacket.data.msg.getTileState64.y,
          //     recvPacket.data.msg.getTileState64.width);

          sendStateTileState64(recvPacket.data.header, runState.tileState64);
          _conn->send(IPAddress(recvPacket.addr), recvPacket.port);
          break;

        case LifxMsg::GetHostFirmware:
          {
            // ESP_LOGD(instance, "GetHostFirmware---TODO");
            Packet::StateHostFirmware stateHostFirmware = {
              0x157f1308f338ec00,
              0,
              3,
              50
            };
            sendStateHostFirmware(recvPacket.data.header, stateHostFirmware);
            _conn->send(IPAddress(recvPacket.addr), recvPacket.port);
          }
          break;

        case LifxMsg::GetWifiFirmware:
          {
            // ESP_LOGD(instance, "GetWifiFirmware---TODO");
            Packet::StateWifiFirmware stateWifiFirmware = {
              0,
              0,
              0,
              0
            };
            sendStateWifiFirmware(recvPacket.data.header, stateWifiFirmware);
            _conn->send(IPAddress(recvPacket.addr), recvPacket.port);
          }
          break;

        case LifxMsg::GetVersion:
          {
            // ESP_LOGD(instance, "GetVersion---TODO-Static-TILE");
            Packet::StateVersion stateVersion = {
              0x00000001,
              // https://github.com/LIFX/products/blob/master/products.json
              /* LIFX - LIFX Tile */
              0x00000037,
              /* LIFX - LIFX Z 2 */
              // 0x00000020,
              0x00000300
            };
            sendStateVersion(recvPacket.data.header, stateVersion);
            _conn->send(IPAddress(recvPacket.addr), recvPacket.port);
          }
          break;
        case LifxMsg::SetColor:
          // ESP_LOGD(instance, "SetColor---TODO:%d-%d-%d-%d:%d",
          //     recvPacket.data.msg.setColor.color.hue,
          //     recvPacket.data.msg.setColor.color.saturation,
          //     recvPacket.data.msg.setColor.color.brightness,
          //     recvPacket.data.msg.setColor.color.kelvin,
          //     recvPacket.data.msg.setColor.duration);
          runState.state.color.hue = recvPacket.data.msg.setColor.color.hue;
          runState.state.color.saturation = recvPacket.data.msg.setColor.color.saturation;
          runState.state.color.brightness = recvPacket.data.msg.setColor.color.brightness;
          runState.state.color.kelvin = recvPacket.data.msg.setColor.color.kelvin;
          {
            HsbColor hsb(runState.state.color.hue/65535.0,
                    runState.state.color.saturation/65535.0,
                    runState.state.color.brightness/65535.0);
            RgbwColor rgbw(hsb);
            for (int i = 0; i < 52; ++i) {
              strip.SetPixelColor(i, rgbw);
            }
            strip.Show();
            // id(leds).make_call()
            //     .set_transition_length(0)
            //     .set_red(rgbw.R/255.0)
            //     .set_green(rgbw.G/255.0)
            //     .set_blue(rgbw.B/255.0)
            //     .set_white(rgbw.W/255.0)
            //     .set_color_temperature(recvPacket.data.msg.setColor.color.kelvin)
            //     .perform();
          }
          if (!recvPacket.data.header.res_required) {
            break;
          }
        case LifxMsg::Get:
          {
            // ESP_LOGD(instance, "Get");
            strncpy(runState.state.label, flashState.label.label, sizeof(runState.state.label));
            sendState(recvPacket.data.header, runState.state);
            _conn->send(IPAddress(recvPacket.addr), recvPacket.port);
          }
          break;
        case LifxMsg::GetHostInfo:
          {
            // ESP_LOGD(instance, "GetHostInfo---TODO");
            sendStateHostWifiInfo(recvPacket.data.header, LifxMsg::StateHostInfo, runState.hostInfo);
            _conn->send(IPAddress(recvPacket.addr), recvPacket.port);
          }
          break;
        case LifxMsg::GetWifiInfo:
          {
            // ESP_LOGD(instance, "GetWifiInfo---TODO");
            sendStateHostWifiInfo(recvPacket.data.header, LifxMsg::StateWifiInfo, runState.hostInfo);
            _conn->send(IPAddress(recvPacket.addr), recvPacket.port);
          }
          break;
        case LifxMsg::Get0x2ce:
          {
            // ESP_LOGD(instance, "Get0x2ce---TODO");
            Packet::State0x2d0 state;
            state.init();
            sendState0x2d0(recvPacket.data.header, state);
            _conn->send(IPAddress(recvPacket.addr), recvPacket.port);
          }
          break;

        case LifxMsg::Get0x2d1:
          {
            // ESP_LOGD(instance, "Get0x2d1---TODO");
            Packet::State0x2d3 state;
            state.init();
            sendState0x2d3(recvPacket.data.header, state);
            _conn->send(IPAddress(recvPacket.addr), recvPacket.port);
          }
          break;

        case LifxMsg::SetLabel:
          {
            // ESP_LOGD(instance, "SetLabel---%s", recvPacket.data.msg.setLabel.label);
            if (strncmp(recvPacket.data.msg.setLabel.label, flashState.label.label,
                  sizeof(recvPacket.data.msg.setLabel.label))) {
              strncpy(flashState.label.label, recvPacket.data.msg.setLabel.label,
                  sizeof(flashState.label.label));
              flashUpdate = true;
            }
          }
          if (!recvPacket.data.header.res_required) {
            break;
          }
        case LifxMsg::GetLabel:
          {
            // ESP_LOGD(instance, "GetLabel--%s", flashState.label.label);
            sendStateLabel(recvPacket.data.header, flashState.label);
            _conn->send(IPAddress(recvPacket.addr), recvPacket.port);
          }
          break;

        case LifxMsg::Set0x37:
          {
            // ESP_LOGD(instance, "Set0x37---%s", recvPacket.data.msg.set0x37.label);
            if (strncmp(recvPacket.data.msg.set0x37.label, flashState.state0x38.label,
                  sizeof(recvPacket.data.msg.set0x37.label)) ||
                memcmp(recvPacket.data.msg.set0x37.location, flashState.state0x38.location,
                  sizeof(recvPacket.data.msg.set0x37.location))) {
              strncpy(flashState.state0x38.label, recvPacket.data.msg.set0x37.label,
                  sizeof(flashState.state0x38.label));
              memcpy(flashState.state0x38.location, recvPacket.data.msg.set0x37.location,
                  sizeof(flashState.state0x38.location));
              flashState.state0x38.updated_at = recvPacket.data.msg.set0x37.updated_at;
              flashUpdate = true;
            }
          }
          if (!recvPacket.data.header.res_required) {
            break;
          }
        case LifxMsg::Get0x36:
          {
            // ESP_LOGD(instance, "Get0x36---%s", flashState.state0x38.label);
            sendStateUuidLabel(recvPacket.data.header, LifxMsg::State0x38, flashState.state0x38);
            _conn->send(IPAddress(recvPacket.addr), recvPacket.port);
          }
          break;

        case LifxMsg::SetWaveformOptional:
          {
            memcpy(&runState.waveFormOptional, &recvPacket.data.msg.setWaveformOptional, sizeof(runState.waveFormOptional));
          }
          break;

        case LifxMsg::SetLocation:
          {
            // ESP_LOGD(instance, "SetLocation---%s", recvPacket.data.msg.setLocation.label);
            if (strncmp(recvPacket.data.msg.setLocation.label, flashState.location.label,
                  sizeof(recvPacket.data.msg.setLocation.label)) ||
                memcmp(recvPacket.data.msg.setLocation.location, flashState.location.location,
                  sizeof(recvPacket.data.msg.setLocation.location))) {
              strncpy(flashState.location.label, recvPacket.data.msg.setLocation.label,
                  sizeof(flashState.location.label));
              memcpy(flashState.location.location, recvPacket.data.msg.setLocation.location,
                  sizeof(flashState.location.location));
              flashState.location.updated_at = recvPacket.data.msg.setLocation.updated_at;
              flashUpdate = true;
            }
          }
          if (!recvPacket.data.header.res_required) {
            break;
          }
        case LifxMsg::GetLocation:
          {
            // ESP_LOGD(instance, "GetLocation---%s", flashState.location.label);
            sendStateUuidLabel(recvPacket.data.header, LifxMsg::StateLocation, flashState.location);
            _conn->send(IPAddress(recvPacket.addr), recvPacket.port);
          }
          break;

        case LifxMsg::SetGroup:
          {
            // ESP_LOGD(instance, "SetGroup---%s", recvPacket.data.msg.setGroup.label);
            if (strncmp(recvPacket.data.msg.setGroup.label, flashState.group.label,
                  sizeof(recvPacket.data.msg.setGroup.label)) ||
                memcmp(recvPacket.data.msg.setGroup.group, flashState.group.group,
                  sizeof(recvPacket.data.msg.setGroup.group))) {
              strncpy(flashState.group.label, recvPacket.data.msg.setGroup.label,
                  sizeof(flashState.group.label));
              memcpy(flashState.group.group, recvPacket.data.msg.setGroup.group,
                  sizeof(flashState.group.group));
              flashState.group.updated_at = recvPacket.data.msg.setGroup.updated_at;
              flashUpdate = true;
            }
          }
          if (!recvPacket.data.header.res_required) {
            break;
          }
        case LifxMsg::GetGroup:
          {
            // ESP_LOGD(instance, "GetGroup---%s", flashState.group.label);
            sendStateUuidLabel(recvPacket.data.header, LifxMsg::StateGroup, flashState.group);
            _conn->send(IPAddress(recvPacket.addr), recvPacket.port);
          }
          break;
        default:
          ESP_LOGD(instance, "recv-%d-%d %x:%d %d-%d %d-%d", dropCount, recvPacket.size,
              recvPacket.addr, recvPacket.port,
              recvPacket.data.header.sequence, recvPacket.data.header.type,
              recvPacket.data.header.res_required,
              recvPacket.data.header.ack_required);
          break;

      }
    }
    void loop() override {
      // This will be called by App.loop()
      if (packetState == LifxPacketState::RECV) {
        if (recvPacket.size < sizeof(Packet::Header)) {
          ESP_LOGE(instance, "illegal packet-%d-%d", ++dropCount, recvPacket.size);
        } else {
          processLifx();
          if (recvPacket.data.header.ack_required) {
            _conn->flush();
            sendNoData(recvPacket.data.header, LifxMsg::Acknowledgement);
            _conn->send(IPAddress(recvPacket.addr), recvPacket.port);
          }
        }
        packetState = LifxPacketState::FREE;
      }
    }

    void sendStateHostWifiInfo(const Packet::Header &header, LifxMsg msg, const Packet::StateWifiInfo &swf) {
      // ESP_LOGD(instance, "sendStateHostWifiInfo:%d:%d:%d", sizeof(Packet::Header), msg, sizeof(swf));
      appendLifxHeader(header, msg, sizeof(swf));
      _conn->append(reinterpret_cast<const char*>(&swf), sizeof(swf));
      runState.hostInfo.tx += sizeof(Packet::Header) + sizeof(swf);
    }

    void sendStateHostFirmware(const Packet::Header &header, const Packet::StateHostFirmware &swf) {
      sendStateFirmware(header, LifxMsg::StateHostFirmware, swf);
    }
    void sendStateWifiFirmware(const Packet::Header &header, const Packet::StateWifiFirmware &swf) {
      sendStateFirmware(header, LifxMsg::StateWifiFirmware, swf);
    }

    void sendStateUuidLabel(const Packet::Header &header, LifxMsg msg, const Packet::StateUuidLabel &swf) {
      // ESP_LOGD(instance, "sendStatUuidLabel:%d:%d:%d", sizeof(Packet::Header), msg, sizeof(swf));
      appendLifxHeader(header, msg, sizeof(swf));
      _conn->append(reinterpret_cast<const char*>(&swf), sizeof(swf));
      runState.hostInfo.tx += sizeof(Packet::Header) + sizeof(swf);
    }

    void sendStateFirmware(const Packet::Header &header, LifxMsg msg, const Packet::StateWifiFirmware &swf) {
      // ESP_LOGD(instance, "sendStatFirmwaree:%d:%d:%d", sizeof(Packet::Header), msg, sizeof(swf));
      appendLifxHeader(header, msg, sizeof(swf));
      _conn->append(reinterpret_cast<const char*>(&swf), sizeof(swf));
      runState.hostInfo.tx += sizeof(Packet::Header) + sizeof(swf);
    }

    void sendState(const Packet::Header &header, const Packet::State &s) {
      // ESP_LOGD(instance, "sendState:%d:%d", sizeof(Packet::Header), sizeof(s));
      appendLifxHeader(header, LifxMsg::State, sizeof(s));
      _conn->append(reinterpret_cast<const char*>(&s), sizeof(s));
      runState.hostInfo.tx += sizeof(Packet::Header) + sizeof(s);
    }
    void sendState0x2d0(const Packet::Header &header, const Packet::State0x2d0 &s) {
      // ESP_LOGD(instance, "sendState0x2d0:%d:%d", sizeof(Packet::Header), sizeof(s));
      appendLifxHeader(header, LifxMsg::State0x2d0, sizeof(s));
      _conn->append(reinterpret_cast<const char*>(&s), sizeof(s));
      runState.hostInfo.tx += sizeof(Packet::Header) + sizeof(s);
    }
    void sendState0x2d3(const Packet::Header &header, const Packet::State0x2d3 &s) {
      // ESP_LOGD(instance, "sendState0x2d3:%d:%d", sizeof(Packet::Header), sizeof(s));
      appendLifxHeader(header, LifxMsg::State0x2d3, sizeof(s));
      _conn->append(reinterpret_cast<const char*>(&s), sizeof(s));
      runState.hostInfo.tx += sizeof(Packet::Header) + sizeof(s);
    }
    void sendStateVersion(const Packet::Header &header, const Packet::StateVersion &sv) {
      // ESP_LOGD(instance, "sendStateVersion:%d:%d", sizeof(Packet::Header), sizeof(sv));
      appendLifxHeader(header, LifxMsg::StateVersion, sizeof(sv));
      _conn->append(reinterpret_cast<const char*>(&sv), sizeof(sv));
      runState.hostInfo.tx += sizeof(Packet::Header) + sizeof(sv);
    }

    void sendStateTileState64(const Packet::Header &header, const Packet::StateTileState64 &sts) {
      // ESP_LOGD(instance, "sendStateTileState64:%d:%d", sizeof(Packet::Header), sizeof(sts));
      appendLifxHeader(header, LifxMsg::StateTileState64, sizeof(sts));
      _conn->append(reinterpret_cast<const char*>(&sts), sizeof(sts));
      runState.hostInfo.tx += sizeof(Packet::Header) + sizeof(sts);
    }
    void sendNoData(const Packet::Header &header, LifxMsg type) {
      // ESP_LOGD(instance, "sendNoData:%d:%d", sizeof(Packet::Header), type);
      appendLifxHeader(header, LifxMsg::Acknowledgement, 0);
      runState.hostInfo.tx += sizeof(Packet::Header);
    }
    void sendStatePower(const Packet::Header &header, const Packet::StatePower &sp) {
      // ESP_LOGD(instance, "sendStatePower:%d:%d", sizeof(Packet::Header), sizeof(sp));
      appendLifxHeader(header, LifxMsg::StatePower, sizeof(sp));
      _conn->append(reinterpret_cast<const char*>(&sp), sizeof(sp));
      runState.hostInfo.tx += sizeof(Packet::Header) + sizeof(sp);
    }
    void sendStateService(const Packet::Header &header, const Packet::StateService &ss) {
      // ESP_LOGD(instance, "sendStateService:%d:%d", sizeof(Packet::Header), sizeof(ss));
      appendLifxHeader(header, LifxMsg::StateService, sizeof(ss));
      _conn->append(reinterpret_cast<const char*>(&ss), sizeof(ss));
      runState.hostInfo.tx += sizeof(Packet::Header) + sizeof(ss);
    }

    void sendStateLabel(const Packet::Header &header, const Packet::StateLabel &sl) {
      // ESP_LOGD(instance, "sendStateLabel:%d:%d", sizeof(Packet::Header), sizeof(sl));
      appendLifxHeader(header, LifxMsg::StateLabel, sizeof(sl));
      _conn->append(reinterpret_cast<const char*>(&sl), sizeof(sl));
      runState.hostInfo.tx += sizeof(Packet::Header) + sizeof(sl);
    }

    void sendStateDeviceChain(const Packet::Header &header, const Packet::StateDeviceChain &sdc) {
      // ESP_LOGD(instance, "sendStateDeviceChain:%d:%d", sizeof(Packet::Header), sizeof(sdc));
      appendLifxHeader(header, LifxMsg::StateDeviceChain, sizeof(sdc));
      _conn->append(reinterpret_cast<const char*>(&sdc), sizeof(sdc));
      runState.hostInfo.tx += sizeof(Packet::Header) + sizeof(sdc);
    }

};
