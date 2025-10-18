#pragma once

#define RADIOLIB_STATIC_ONLY 1
#include <RadioLib.h>
#include <helpers/radiolib/RadioLibWrappers.h>
#include "T1000eBoard.h"
#include <helpers/radiolib/CustomLR1110Wrapper.h>
#include <helpers/ArduinoHelpers.h>
#include <helpers/SensorManager.h>
#include <helpers/sensors/location/LocationProvider.h>
#include <helpers/sensors/location/GpsDriver.h>
#ifdef DISPLAY_CLASS
  #include "NullDisplayDriver.h"
#endif

class T1000SensorManager: public SensorManager {
  LocationProvider location_provider;

public:
  T1000SensorManager(GpsDriver &nmea) : location_provider(&nmea) { }
  bool begin() override;
  bool querySensors(uint8_t requester_permissions, CayenneLPP& telemetry) override;
  void loop() override;
  int getNumSettings() const override;
  const char* getSettingName(int i) const override;
  const char* getSettingValue(int i) const override;
  bool setSettingValue(const char* name, const char* value) override;
  LocationProvider* getGps() { return &location_provider; }
};

#ifdef DISPLAY_CLASS
  extern NullDisplayDriver display;
#endif

extern T1000eBoard board;
extern WRAPPER_CLASS radio_driver;
extern VolatileRTCClock rtc_clock;
extern T1000SensorManager sensors;

bool radio_init();
uint32_t radio_get_rng_seed();
void radio_set_params(float freq, float bw, uint8_t sf, uint8_t cr);
void radio_set_tx_power(uint8_t dbm);
mesh::LocalIdentity radio_new_identity();
