#include <Arduino.h>
#include "target.h"
#include <helpers/sensors/MicroNMEALocationProvider.h>

WIO1110DevBoard board;

RADIO_CLASS radio = new Module(LORA_NSS, LORA_DIO_1, LORA_RESET, LORA_BUSY, SPI);

WRAPPER_CLASS radio_driver(radio, board);

VolatileRTCClock rtc_clock;
MicroNMEALocationProvider nmea = MicroNMEALocationProvider(Serial1);
WIO1110SensorManager sensors = WIO1110SensorManager(nmea);

#ifndef LORA_CR
  #define LORA_CR      5
#endif

#ifdef RF_SWITCH_TABLE
static const uint32_t rfswitch_dios[Module::RFSWITCH_MAX_PINS] = {
  RADIOLIB_LR11X0_DIO5,
  RADIOLIB_LR11X0_DIO6,
  RADIOLIB_NC,
  RADIOLIB_NC,
  RADIOLIB_NC
};

static const Module::RfSwitchMode_t rfswitch_table[] = {
  // mode                 DIO5  DIO6 
  { LR11x0::MODE_STBY,   {LOW,  LOW  }},  
  { LR11x0::MODE_RX,     {HIGH, LOW  }},
  { LR11x0::MODE_TX,     {HIGH, HIGH }},
  { LR11x0::MODE_TX_HP,  {LOW,  HIGH }},
  { LR11x0::MODE_TX_HF,  {LOW,  LOW  }}, 
  { LR11x0::MODE_GNSS,   {LOW,  LOW  }},
  { LR11x0::MODE_WIFI,   {LOW,  LOW  }},  
  END_OF_MODE_TABLE,
};
#endif

bool radio_init() {
  //rtc_clock.begin(Wire);
  
#ifdef LR11X0_DIO3_TCXO_VOLTAGE
  float tcxo = LR11X0_DIO3_TCXO_VOLTAGE;
#else
  float tcxo = 1.6f;
#endif

  SPI.setPins(LORA_MISO, LORA_SCLK, LORA_MOSI);
  SPI.begin();
  int status = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, RADIOLIB_LR11X0_LORA_SYNC_WORD_PRIVATE, LORA_TX_POWER, 8, tcxo);
  if (status != RADIOLIB_ERR_NONE) {
    Serial.print("ERROR: radio init failed: ");
    Serial.println(status);
    return false;  // fail
  }
  
  radio.setCRC(1);

#ifdef RF_SWITCH_TABLE
  radio.setRfSwitchTable(rfswitch_dios, rfswitch_table);
#endif
#ifdef RX_BOOSTED_GAIN
  radio.setRxBoostedGainMode(RX_BOOSTED_GAIN);
#endif

  return true;  // success
}

uint32_t radio_get_rng_seed() {
  return radio.random(0x7FFFFFFF);
}

void radio_set_params(float freq, float bw, uint8_t sf, uint8_t cr) {
  radio.setFrequency(freq);
  radio.setSpreadingFactor(sf);
  radio.setBandwidth(bw);
  radio.setCodingRate(cr);
}

void radio_set_tx_power(uint8_t dbm) {
  radio.setOutputPower(dbm);
}

mesh::LocalIdentity radio_new_identity() {
  RadioNoiseListener rng(radio);
  return mesh::LocalIdentity(&rng);  // create new random identity
}

void WIO1110SensorManager::start_gps() {
  gps_active = true;
  //_nmea->begin();
  // this init sequence should be better 
  // comes from seeed examples and deals with all gps pins
  // pinMode(GPS_EN, OUTPUT);
  // digitalWrite(GPS_EN, HIGH);
  // delay(10);
  // pinMode(GPS_VRTC_EN, OUTPUT);
  // digitalWrite(GPS_VRTC_EN, HIGH);
  // delay(10);
       
  // pinMode(GPS_RESET, OUTPUT);
  // digitalWrite(GPS_RESET, HIGH);
  // delay(10);
  // digitalWrite(GPS_RESET, LOW);
       
  // pinMode(GPS_SLEEP_INT, OUTPUT);
  // digitalWrite(GPS_SLEEP_INT, HIGH);
  // pinMode(GPS_RTC_INT, OUTPUT);
  // digitalWrite(GPS_RTC_INT, LOW);
  // pinMode(GPS_RESETB, INPUT_PULLUP);
}

void WIO1110SensorManager::sleep_gps() {
  gps_active = false;
  // digitalWrite(GPS_VRTC_EN, HIGH);
  // digitalWrite(GPS_EN, LOW);
  // digitalWrite(GPS_RESET, HIGH);
  // digitalWrite(GPS_SLEEP_INT, HIGH);
  // digitalWrite(GPS_RTC_INT, LOW);
  // pinMode(GPS_RESETB, OUTPUT);
  // digitalWrite(GPS_RESETB, LOW);
  // //_nmea->stop();
}

void WIO1110SensorManager::stop_gps() {
  gps_active = false;
  // digitalWrite(GPS_VRTC_EN, LOW);
  // digitalWrite(GPS_EN, LOW);
  // digitalWrite(GPS_RESET, HIGH);
  // digitalWrite(GPS_SLEEP_INT, HIGH);
  // digitalWrite(GPS_RTC_INT, LOW);
  // pinMode(GPS_RESETB, OUTPUT);
  // digitalWrite(GPS_RESETB, LOW);
  // //_nmea->stop();
}


bool WIO1110SensorManager::begin() {
  // init GPS
  Serial1.begin(115200);

  // make sure gps pin are off
  // digitalWrite(GPS_VRTC_EN, LOW);
  // digitalWrite(GPS_RESET, LOW);
  // digitalWrite(GPS_SLEEP_INT, LOW);
  // digitalWrite(GPS_RTC_INT, LOW);
  // pinMode(GPS_RESETB, OUTPUT);
  // digitalWrite(GPS_RESETB, LOW);

  return true;
}

bool WIO1110SensorManager::querySensors(uint8_t requester_permissions, CayenneLPP& telemetry) {
  if (requester_permissions & TELEM_PERM_LOCATION) {   // does requester have permission?
    telemetry.addGPS(TELEM_CHANNEL_SELF, node_lat, node_lon, node_altitude);
  }
  return true;
}

void WIO1110SensorManager::loop() {
  static long next_gps_update = 0;

  _nmea->loop();

  if (millis() > next_gps_update) {
    if (_nmea->isValid()) {
      node_lat = ((double)_nmea->getLatitude())/1000000.;
      node_lon = ((double)_nmea->getLongitude())/1000000.;
      node_altitude = ((double)_nmea->getAltitude()) / 1000.0;
      //Serial.printf("lat %f lon %f\r\n", _lat, _lon);
    }
    next_gps_update = millis() + 1000;
  }
}

int WIO1110SensorManager::getNumSettings() const { return 1; }  // just one supported: "gps" (power switch)

const char* WIO1110SensorManager::getSettingName(int i) const {
  return i == 0 ? "gps" : NULL;
}
const char* WIO1110SensorManager::getSettingValue(int i) const {
  if (i == 0) {
    return gps_active ? "1" : "0";
  }
  return NULL;
}
bool WIO1110SensorManager::setSettingValue(const char* name, const char* value) {
  if (strcmp(name, "gps") == 0) {
    if (strcmp(value, "0") == 0) {
      sleep_gps(); // sleep for faster fix !
    } else {
      start_gps();
    }
    return true;
  }
  return false;  // not supported
}
