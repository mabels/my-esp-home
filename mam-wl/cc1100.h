#include "esphome.h"

#define CC1100_PARTNUM          0x30    // Current version number
#define CC1100_VERSION          0x31    // Current version number

//Read from or write to register from the SCP1000:
// static unsigned int readRegister(byte thisRegister, int bytesToRead) {
//   byte inByte = 0;           // incoming byte from the SPI
//   unsigned int result = 0;   // result to return
//   // SCP1000 expects the register name in the upper 6 bits
//   // of the byte. So shift the bits left by two bits:
//   // thisRegister = thisRegister << 2;
//   // now combine the address and the command into one byte
//   // byte dataToSend = thisRegister & READ;
//   // Serial.println(thisRegister, BIN);
//   // take the chip select low to select the device:
//   digitalWrite(chipSelectPin, LOW);
//   // send the device the register you want to read:
//   SPI.transfer(dataToSend);
//   // send a value of 0 to read the first byte returned:
//   result = SPI.transfer(0x00);
//   // decrement the number of bytes left to read:
//   bytesToRead--;
//   // if you still have another byte to read:
//   if (bytesToRead > 0) {
//     // shift the first byte left, then get the second byte:
//     result = result << 8;
//     inByte = SPI.transfer(0x00);
//     // combine the byte you just got with the previous one:
//     result = result | inByte;
//     // decrement the number of bytes left to read:
//     bytesToRead--;
//   }
//   // take the chip select high to de-select:
//   digitalWrite(chipSelectPin, HIGH);
//   // return the result:
//   return (result);
// }


class CC1100Component : public PollingComponent, public TextSensor {
 private:
   int cnt;
   uint8_t chipSelectPin;
 public:
  CC1100Component(): PollingComponent(1000), cnt(4711), chipSelectPin(D8) {
  }
  void setup() override {
    // SPI.pins(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
    SPI.begin();

    // readRegister();
    // publish_state(this->cnt++);
    // This will be called by App.setup()
  }

  void update() override {

    digitalWrite(this->chipSelectPin, LOW);
    SPI.transfer(CC1100_PARTNUM);
    auto partnum = SPI.transfer(0x00);
    digitalWrite(this->chipSelectPin, HIGH);
    delay(1);

    digitalWrite(this->chipSelectPin, LOW);
    SPI.transfer(CC1100_VERSION);
    auto version = SPI.transfer(0x00);
    digitalWrite(this->chipSelectPin, HIGH);

    char out[48];
    sprintf(out, "{\"partnum\":%d,\"version\":%d}", partnum, version);
    this->publish_state(out);
  }
};

