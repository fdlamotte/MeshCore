#include "CommonRadioPrefs.h"
#include "TxtDataHelpers.h"
#include "Utils.h"

bool CommonRadioPrefs::handleCommand(const char* command, uint32_t sender_timestamp, char* reply) {
  if (strcmp(command, "get radio") == 0) {
    char freq[16], bw[16];
    strcpy(freq, StrHelper::ftoa(getFreq()));
    strcpy(bw, StrHelper::ftoa3(getBandwidth()));
    sprintf(reply, "> %s,%s,%d,%d", freq, bw, (uint32_t)getSpreadFactor(), (uint32_t)getCodingRate());
    return true;
  }
  if (memcmp(command, "set radio ", 10) == 0) {
    char tmp[132];
    strcpy(tmp, &command[10]);
    const char *parts[4];
    int num = mesh::Utils::parseTextParts(tmp, parts, 4);
    float freq  = num > 0 ? strtof(parts[0], nullptr) : 0.0f;
    float bw    = num > 1 ? strtof(parts[1], nullptr) : 0.0f;
    uint8_t sf  = num > 2 ? atoi(parts[2]) : 0;
    uint8_t cr  = num > 3 ? atoi(parts[3]) : 0;
    if (freq >= 150.0f && freq <= 2500.0f && sf >= 5 && sf <= 12 && cr >= 5 && cr <= 8 && bw >= 7.0f && bw <= 500.0f) {
      setSpreadFactor(sf);
      setCodingRate(cr);
      setFreq(freq);
      setBandwidth(bw);
      // NOTE: savePrefs() should be handled by caller
      strcpy(reply, "OK - reboot to apply");
    } else {
      strcpy(reply, "Error, invalid radio params");
    }
    return true;
  }
  if (strcmp(command, "get dutycycle") == 0) {
    float dc = 100.0f / (getAirtimeFactor() + 1.0f);
    int dc_int = (int)dc;
    int dc_frac = (int)((dc - dc_int) * 10.0f + 0.5f);
    sprintf(reply, "> %d.%d%%", dc_int, dc_frac);
    return true;
  }
  if (memcmp(command, "set dutycycle ", 14) == 0) {
    float dc = atof(&command[14]);
    if (dc < 1 || dc > 100) {
      strcpy(reply, "ERROR: dutycycle must be 1-100");
    } else {
      setAirtimeFactor((100.0f / dc) - 1.0f);
      // NOTE: savePrefs() should be handled by caller
      float actual = 100.0f / (getAirtimeFactor() + 1.0f);
      int a_int = (int)actual;
      int a_frac = (int)((actual - a_int) * 10.0f + 0.5f);
      sprintf(reply, "OK - %d.%d%%", a_int, a_frac);
    }
    return true;
  }
  return false; // not handled
}
