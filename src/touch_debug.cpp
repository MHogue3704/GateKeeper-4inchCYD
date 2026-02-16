#include <Arduino.h>
#include <SPI.h>
#include "../include/TFT_eSPI_Setup.h"

// XPT2046 Register Commands
#define XPT2046_CMD_READ_X   0xD0
#define XPT2046_CMD_READ_Y   0x90
#define XPT2046_CMD_READ_Z1  0xB0
#define XPT2046_CMD_READ_Z2  0xC0

void testTouchSPI() {
  Serial.println("\n=== XPT2046 SPI Debug ===");
  Serial.printf("Testing SPI communication on GPIO %d (CS), GPIO %d (IRQ)\n", TOUCH_CS, TOUCH_IRQ);
  
  // Configure pins
  pinMode(TOUCH_CS, OUTPUT);
  digitalWrite(TOUCH_CS, HIGH);
  
  pinMode(TOUCH_IRQ, INPUT);   // TOUCH_IRQ (should go LOW when touched)
  
  Serial.println("Pin setup complete");
  Serial.printf("TOUCH_CS (GPIO %d) = %d (should be HIGH)\n", TOUCH_CS, digitalRead(TOUCH_CS));
  Serial.printf("TOUCH_IRQ (GPIO %d) = %d\n", TOUCH_IRQ, digitalRead(TOUCH_IRQ));
  
  // Initialize SPI for touch
#ifdef USE_HSPI_PORT
  static SPIClass touchSPI(HSPI);
  touchSPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TOUCH_CS);
  touchSPI.beginTransaction(SPISettings(2500000, MSBFIRST, SPI_MODE0));
#else
  SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TOUCH_CS);
  SPI.beginTransaction(SPISettings(2500000, MSBFIRST, SPI_MODE0));
#endif
  
  // Try reading X coordinate
  Serial.println("\nAttempting to read X coordinate...");
  digitalWrite(TOUCH_CS, LOW);
  delayMicroseconds(10);
  
  uint8_t cmd = XPT2046_CMD_READ_X;
  uint16_t response = 0;
  
  // Send command and read response
  #ifdef USE_HSPI_PORT
  response = touchSPI.transfer16(cmd << 8);
  #else
  response = SPI.transfer16(cmd << 8);
  #endif
  
  digitalWrite(TOUCH_CS, HIGH);
  #ifdef USE_HSPI_PORT
  touchSPI.endTransaction();
  #else
  SPI.endTransaction();
  #endif
  
  Serial.printf("Response: 0x%04X\n", response);
  Serial.printf("Response bytes: High=0x%02X, Low=0x%02X\n", (response >> 8) & 0xFF, response & 0xFF);
  
  if (response == 0x0000 || response == 0xFFFF) {
    Serial.println("[ERROR] Got all zeros or all ones - SPI communication failed!");
    Serial.println("Possible causes:");
    Serial.println("  1. MOSI/MISO/SCLK pins not connected");
    Serial.println("  2. Touch controller not powered (check 3.3V)");
    Serial.println("  3. Wrong CS pin (should be GPIO 33)");
    Serial.println("  4. Touch controller hardware failure");
  } else {
    Serial.println("[OK] Got valid response - touch controller responding!");
  }
  
  Serial.println("======================\n");
}

// Call this from setup() if touch not working
// Uncomment in main: testTouchSPI();
