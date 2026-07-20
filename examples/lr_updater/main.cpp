#include <Mesh.h>
#include <Arduino.h>

#include <RadioLib.h>
#include <helpers/radiolib/RadioLibWrappers.h>
#include <ThinkNodeM9Board.h>
#include <helpers/radiolib/CustomLR1110Wrapper.h>

#define RADIOLIB_LR1110_FIRMWARE_0402
#include <modules/LR11x0/LR11x0_firmware.h>

ThinkNodeM9Board board;

SPIClass customSPI(FSPI); 
SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0);
Module module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY,customSPI, spiSettings);
RADIO_CLASS radio (&module);
WRAPPER_CLASS radio_driver(radio, board);

#ifndef LORA_CR
  #define LORA_CR 5
#endif

void printVersions();
bool enterLR1110Bootloader();
int lr_setup();

void setup() {
  setCpuFrequencyMhz(80);

  Serial.begin(115200);
  delay(1000);

  board.begin();
  customSPI.begin(P_LORA_SCLK, P_LORA_MISO, P_LORA_MOSI, P_LORA_NSS);

  enterLR1110Bootloader();
  digitalWrite(18, LOW);

  delay(5000);

  int state;
  // state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, RADIOLIB_LR11X0_LORA_SYNC_WORD_PRIVATE, LORA_TX_POWER, 16, tcxo);
  // if (state != RADIOLIB_ERR_NONE) {
  //    Serial.print("ERROR: radio init failed: ");
  //    Serial.println(state);
  //    while(1) {delay(10);}
  // }
  //int16_t state = radio.modSetup(RADIOLIB_LR11X0_PACKET_TYPE_LORA);

  state = lr_setup();
// print the firmware versions before the update
  printVersions();

  // prompt the user
  Serial.println(F("[LR1110] Send any character to start the update"));
  while(!Serial.available()) { delay(1); }

  // upload update into LR11x0 non-volatile memory
  Serial.print(F("[LR1110] Updating firmware, this may take several seconds ... "));
  state = radio.updateFirmware(lr11xx_firmware_image, RADIOLIB_LR11X0_FIRMWARE_IMAGE_SIZE);
  
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); }
  }

  // print the firmware versions after the update
  printVersions();
  
}

void loop() {
}

/* 
 * Sets up LR11x0 chip, mostly code stripped out from
 * LR11x0::begin procedure
 */

int lr_setup() {
  int state;

#ifdef LR11X0_DIO3_TCXO_VOLTAGE
  float tcxo = LR11X0_DIO3_TCXO_VOLTAGE;
#else
  float tcxo = 1.6f;
#endif

  module.init();
  module.hal->pinMode(module.getIrq(), module.hal->GpioModeInput);
  module.hal->pinMode(module.getGpio(), module.hal->GpioModeInput);
  module.spiConfig.cmds[RADIOLIB_MODULE_SPI_COMMAND_READ] = RADIOLIB_LR11X0_CMD_READ_REG_MEM;
  module.spiConfig.cmds[RADIOLIB_MODULE_SPI_COMMAND_WRITE] = RADIOLIB_LR11X0_CMD_WRITE_REG_MEM;
  module.spiConfig.cmds[RADIOLIB_MODULE_SPI_COMMAND_NOP] = RADIOLIB_LR11X0_CMD_NOP;
  module.spiConfig.cmds[RADIOLIB_MODULE_SPI_COMMAND_STATUS] = RADIOLIB_LR11X0_CMD_GET_STATUS;
  module.spiConfig.widths[RADIOLIB_MODULE_SPI_WIDTH_ADDR] = Module::BITS_32;
  module.spiConfig.widths[RADIOLIB_MODULE_SPI_WIDTH_STATUS] = Module::BITS_8;
  radio.gnss = false;

  state = radio.reset();
  RADIOLIB_ASSERT(state);

  delay(10);

  state = radio.standby();
  RADIOLIB_ASSERT(state);

  if(tcxo > 0.0f) {
    state = radio.setTCXO(tcxo);
    RADIOLIB_ASSERT(state);
  }

  state = radio.config(RADIOLIB_LR11X0_PACKET_TYPE_LORA);
  RADIOLIB_ASSERT(state);

  return state;
}

bool enterLR1110Bootloader() {
  Serial.println(F("[LR1110] Forcing into bootloader mode via BUSY LOW..."));

  // 1. BUSY -> OUTPUT + LOW
  pinMode(P_LORA_BUSY, OUTPUT);
  pinMode(P_LORA_RESET, OUTPUT);
  delay(10);

  digitalWrite(P_LORA_BUSY, LOW);

  // 2. RESET for 10ms
  digitalWrite(P_LORA_RESET, LOW);
  delay(10);
  digitalWrite(P_LORA_RESET, HIGH);

  delay(500); // 

  // 3. Release BUSY as input
  pinMode(P_LORA_BUSY, INPUT);
  delay(100); // Attente de stabilisation de 100ms

  // 4. Wait for the chip to be ready
  uint32_t startTime = millis();
  while (digitalRead(P_LORA_BUSY) == HIGH) {
    if (millis() - startTime > 1000) { // 1s timeout
      Serial.println(F("ERROR : BUSY stuck to HIGH after reset."));
      return false;
    }
    delay(1);
  }

  Serial.println(F("LR1110 in bootloader mode !"));
  return true;
}

void printVersions() {
  LR11x0VersionInfo_t version;
  Serial.print(F("[LR1110] Reading firmware versions ... "));

  uint8_t hardware, device, fwMajor, fwMinor;

  radio.reset();
  int16_t state = radio.getVersion(&hardware, &device, &fwMajor, &fwMinor);
  
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));

    Serial.print(F("[LR1110] Device: "));
    Serial.println(device);

    Serial.print(F("[LR1110] Base firmware: "));
    Serial.print(fwMajor);
    Serial.print('.');
    Serial.println(fwMinor);
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) { delay(10); } 
  }

}