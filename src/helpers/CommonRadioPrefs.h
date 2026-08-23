#include "ConfigSerializer.h"

class CommonRadioPrefs : public ConfigSerializer {
  bool _is_dirty = false;
protected:
  CommonRadioPrefs() { }
public:
  void markDirty() { _is_dirty = true; }
  void clearDirty() { _is_dirty = false; }
  bool isDirty() const { return _is_dirty; }

  virtual float getFreq() const = 0;
  virtual void setFreq(float f) = 0;

  virtual float getBandwidth() const = 0;
  virtual void setBandwidth(float bw) = 0;

  virtual uint8_t getSpreadFactor() const = 0;
  virtual void setSpreadFactor(uint8_t sf) = 0;

  virtual uint8_t getCodingRate() const = 0;
  virtual void setCodingRate(uint8_t cr) = 0;

  virtual float getAirtimeFactor() const = 0;
  virtual void setAirtimeFactor(float af) = 0;

    //   //def("cad", _parent->cad_enabled);
    //   //def("int_thr", _parent->interference_threshold);
    //   def("rxgain", _parent->rx_boosted_gain);
    // #if 0
    //   // NOTE: these cannot be set (yet) so don't load/save until we can.
    //   //       also, fem_rxgain WAS mapped to wrong JSON property previously
    //   def("fem_rxgain", _parent->radio_fem_rxgain);
    //   def("fem_txgain", _parent->radio_fem_txgain);
    // #endif
    //   def("tx", _parent->tx_power_dbm);
    //   def("rxdelay", _parent->rx_delay_base);
    //   //def("f_txdelay", _parent->tx_delay_factor);   currently hard-coded
    //   //def("d_txdelay", _parent->direct_tx_delay_factor);  currently hard-coded
    //   //def("agc_int", _parent->agc_reset_interval);
    //   def("hash_mode", _parent->path_hash_mode);
    //   def("multi_ack", _parent->multi_acks);

  bool handleCommand(const char* command, uint32_t sender_timestamp, char* reply);
};
