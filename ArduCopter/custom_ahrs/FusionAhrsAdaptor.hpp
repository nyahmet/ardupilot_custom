#pragma once

#include "fusionAhrs/Fusion.h"
#include <AP_Math/AP_Math.h>

class FusionAhrsAdaptor
{
public:
  float getFusionRoll();
  float getFusionPitch();
  float getFusionYaw();

  void initialize();
  void update(Vector3f acc, Vector3f gyr, Vector3f mag);

private:
  FusionAhrs m_ahrs;
  FusionOffset m_offset;

  static constexpr float DIVISION_GRAVITY_CONSTANT = 0.101972F;

  float m_fusion_gain = 0.1F;
  float m_gyroscope_range = 2000.0F;
  float m_acceleration_rejection = 30.0F;
  float m_magnetometer_rejection = 30.0F;
  uint8_t m_recovery_trigger_period = 960;
  uint16_t m_sampling_freq = 400;
  float m_sampling_period_s = 0.0025F;

  float m_declination = 5.0F;

  FusionEuler m_eulerAngle;
};
