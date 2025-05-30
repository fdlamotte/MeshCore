#pragma once

#define RADIOLIB_STATIC_ONLY 1
#include <RadioLib.h>
#include <helpers/RadioLibWrappers.h>
#include <helpers/CustomLR1110Wrapper.h>
#include <helpers/ArduinoHelpers.h>
#include <helpers/SensorManager.h>
#include <helpers/sensors/LocationProvider.h>

class WIO1110DevBoard : public mesh::MainBoard {
protected:
  uint8_t startup_reason;
  uint8_t btn_prev_state;

public:
  void begin() {
    startup_reason = BD_STARTUP_NORMAL;
    btn_prev_state = HIGH;

    sd_power_mode_set(NRF_POWER_MODE_LOWPWR);

  #if defined(PIN_BOARD_SDA) && defined(PIN_BOARD_SCL)
    Wire.setPins(PIN_BOARD_SDA, PIN_BOARD_SCL);
  #endif

//    Wire.begin();

    delay(10);   // give sx1262 some time to power up
  }
  uint16_t getBattMilliVolts() override { return 0; }

  uint8_t getStartupReason() const override { return startup_reason; }

  const char* getManufacturerName() const override {
    return "Seeed Wio-WM1110 Dev Board";
  }

  void reboot() override {
    NVIC_SystemReset();
  }
};

class WIO1110SensorManager: public SensorManager {
  bool gps_active = false;
  LocationProvider * _nmea;

  void start_gps();
  void sleep_gps();
  void stop_gps();
public:
  WIO1110SensorManager(LocationProvider &nmea): _nmea(&nmea) { }
  bool begin() override;
  bool querySensors(uint8_t requester_permissions, CayenneLPP& telemetry) override;
  void loop() override;
  int getNumSettings() const override;
  const char* getSettingName(int i) const override;
  const char* getSettingValue(int i) const override;
  bool setSettingValue(const char* name, const char* value) override;
};

#ifdef DISPLAY_CLASS
  extern NullDisplayDriver display;
#endif

extern WIO1110DevBoard board;
extern WRAPPER_CLASS radio_driver;
extern VolatileRTCClock rtc_clock;
extern WIO1110SensorManager sensors;

bool radio_init();
uint32_t radio_get_rng_seed();
void radio_set_params(float freq, float bw, uint8_t sf, uint8_t cr);
void radio_set_tx_power(uint8_t dbm);
mesh::LocalIdentity radio_new_identity();
