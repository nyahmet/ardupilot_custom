#include "FusionAhrsAdaptor.hpp"
#include "fusionAhrs/FusionMath.h"

void FusionAhrsAdaptor::initialize()
{
  FusionAhrsInitialise(&m_ahrs);
  FusionOffsetInitialise(&m_offset, m_sampling_freq);
}

void FusionAhrsAdaptor::update(Vector3f acc, Vector3f gyr, Vector3f mag)
{
  const FusionAhrsSettings settings = {
    .convention = FusionConventionNed,
    .gain = m_fusion_gain,
    .gyroscopeRange = m_gyroscope_range,
    .accelerationRejection = m_acceleration_rejection,
    .magneticRejection = m_magnetometer_rejection,
    .recoveryTriggerPeriod = static_cast<unsigned int>(m_recovery_trigger_period * m_sampling_freq),
  };
  FusionAhrsSetSettings(&m_ahrs, &settings);

  FusionVector gyroscope = { gyr.x, gyr.y, gyr.z };
  FusionVector accelerometer = { acc.x * DIVISION_GRAVITY_CONSTANT, acc.y * DIVISION_GRAVITY_CONSTANT, acc.z * DIVISION_GRAVITY_CONSTANT };
  FusionVector magnetometer = { mag.x, mag.y, mag.z };

  gyroscope = FusionOffsetUpdate(&m_offset, gyroscope);

  FusionAhrsUpdate(&m_ahrs, gyroscope, accelerometer, magnetometer, m_sampling_period_s);

  m_eulerAngle = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&m_ahrs));
}

float FusionAhrsAdaptor::getFusionRoll()
{
  return m_eulerAngle.angle.roll;
}

float FusionAhrsAdaptor::getFusionPitch()
{
  return m_eulerAngle.angle.pitch;
}

float FusionAhrsAdaptor::getFusionYaw()
{
  return m_eulerAngle.angle.yaw + m_declination;
}
