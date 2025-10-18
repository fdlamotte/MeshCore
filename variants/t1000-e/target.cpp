#include <Arduino.h>
#include "t1000e_sensors.h"
#include "target.h"
#include <helpers/sensors/location/MicroNMEAGpsDriver.h>

T1000eBoard board;

RADIO_CLASS radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, SPI);

WRAPPER_CLASS radio_driver(radio, board);

VolatileRTCClock rtc_clock;
MicroNMEAGpsDriver nmea = MicroNMEAGpsDriver(Serial1);
T1000SensorManager sensors = T1000SensorManager(nmea);

#ifdef DISPLAY_CLASS
  NullDisplayDriver display;
#endif

#ifndef LORA_CR
  #define LORA_CR      5
#endif

#ifdef RF_SWITCH_TABLE
static const uint32_t rfswitch_dios[Module::RFSWITCH_MAX_PINS] = {
  RADIOLIB_LR11X0_DIO5,
  RADIOLIB_LR11X0_DIO6,
  RADIOLIB_LR11X0_DIO7,
  RADIOLIB_LR11X0_DIO8, 
  RADIOLIB_NC
};

static const Module::RfSwitchMode_t rfswitch_table[] = {
  // mode                 DIO5  DIO6  DIO7  DIO8
  { LR11x0::MODE_STBY,   {LOW,  LOW,  LOW,  LOW  }},  
  { LR11x0::MODE_RX,     {HIGH, LOW,  LOW,  HIGH }},
  { LR11x0::MODE_TX,     {HIGH, HIGH, LOW,  HIGH }},
  { LR11x0::MODE_TX_HP,  {LOW,  HIGH, LOW,  HIGH }},
  { LR11x0::MODE_TX_HF,  {LOW,  LOW,  LOW,  LOW  }}, 
  { LR11x0::MODE_GNSS,   {LOW,  LOW,  HIGH, LOW  }},
  { LR11x0::MODE_WIFI,   {LOW,  LOW,  LOW,  LOW  }},  
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

  SPI.setPins(P_LORA_MISO, P_LORA_SCLK, P_LORA_MOSI);
  SPI.begin();
  int status = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, RADIOLIB_LR11X0_LORA_SYNC_WORD_PRIVATE, LORA_TX_POWER, 16, tcxo);
  if (status != RADIOLIB_ERR_NONE) {
    Serial.print("ERROR: radio init failed: ");
    Serial.println(status);
    return false;  // fail
  }
  
  radio.setCRC(2);
  radio.explicitHeader();

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

bool T1000SensorManager::begin() {
  // init GPS
  Serial1.begin(115200);
  digitalWrite(GPS_VRTC_EN, HIGH);
  location_provider.begin();
  return true;
}

bool T1000SensorManager::querySensors(uint8_t requester_permissions, CayenneLPP& telemetry) {
  if (requester_permissions & TELEM_PERM_LOCATION) {   // does requester have permission?
    telemetry.addGPS(TELEM_CHANNEL_SELF, node_lat, node_lon, node_altitude);
  }
  if (requester_permissions & TELEM_PERM_ENVIRONMENT) {
    telemetry.addLuminosity(TELEM_CHANNEL_SELF, t1000e_get_light());
    telemetry.addTemperature(TELEM_CHANNEL_SELF, t1000e_get_temperature());
  }
  return true;
}

void T1000SensorManager::loop() {
  static long next_gps_update = 0;

  location_provider.loop();

  node_lat = location_provider.node_lat;
  node_lon = location_provider.node_lon;
  node_altitude = location_provider.node_altitude;
}

int T1000SensorManager::getNumSettings() const { 
  int settings = 0;
  if (location_provider.isDetected()) settings++;  // only show GPS setting if GPS is detected
  return settings;
}

const char* T1000SensorManager::getSettingName(int i) const {
  int settings = 0;
  if (location_provider.isDetected() && i == settings++) {
    return "gps";
  }
  return NULL;
}

const char* T1000SensorManager::getSettingValue(int i) const {
  int settings = 0;
    if (location_provider.isDetected() && i == settings++) {
      return location_provider.isEnabled() ? "1" : "0";
    }
  return NULL;
}

bool T1000SensorManager::setSettingValue(const char* name, const char* value) {
  if (location_provider.isDetected() && strcmp(name, "gps") == 0) {
    location_provider.setEnabled(strcmp(value, "0") != 0);
    return true;
  }
  if (location_provider.isDetected() && strcmp(name, "gps_interval") == 0) {
    location_provider.setCycleInterval(atoi(value));
    return true;
  }
  return false;  // not supported
}
