#include <Wire.h>

#define TCA_ADDR 0x70
#define AS5600_ADDR 0x36
#define ENCODER_CHANNEL 0  // Only one encoder connected here

void tcaSelect(uint8_t channel) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);  // Select the I2C channel
  Wire.endTransmission();
}

int readAS5600() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0E); // Angle register (MSB)
  Wire.endTransmission();

  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() == 2) {
    int angle_raw = (Wire.read() << 8) | Wire.read();
    return angle_raw;
  }

  return -1; // Error
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  tcaSelect(ENCODER_CHANNEL);  // Select the channel once in setup
}

void loop() {
  int angle = readAS5600();
  if (angle >= 0) {
    float rad = (angle * 2.0 * PI) / 4096.0;
    
    // Map from [0, 2π) to [-π, +π)
    if (rad >= PI) {
      rad -= 2.0 * PI;
    }

    Serial.println(rad);
  } else {
    Serial.println("Read error");
  }

  delay(10); // ~100 Hz update rate
}