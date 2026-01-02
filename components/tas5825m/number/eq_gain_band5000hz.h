#pragma once

#include "esphome/components/number/number.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "../tas5825m.h"

namespace esphome::tas5825m {

class EqGainBand5000hz : public number::Number, public Component, public Parented<Tas5825mComponent> {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }

 protected:
  void control(float value) override;

  ESPPreferenceObject pref_;
};

}  // namespace esphome::tas5825m
