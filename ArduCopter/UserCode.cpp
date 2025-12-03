#include "Copter.h"
#include "custom_ahrs/FusionAhrsAdaptor.hpp"

static FusionAhrsAdaptor my_fusion;

static float my_calc_roll_rad = 0.0f;
static float my_calc_pitch_rad = 0.0f;
static float my_calc_yaw_rad = 0.0f;

static uint32_t last_run_us = 0;


void run_my_estimation(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dt)
{

    Vector3f acc_vec(ax, ay, az);
    Vector3f gyr_vec(gx, gy, gz);
    Vector3f mag_vec(mx, my, mz);

    my_fusion.update(acc_vec, gyr_vec, mag_vec);

    
    my_calc_roll_rad  = radians(my_fusion.getFusionRoll());
    my_calc_pitch_rad = radians(my_fusion.getFusionPitch());
    my_calc_yaw_rad   = radians(my_fusion.getFusionYaw());
}

void Copter::userhook_init()
{
    my_calc_roll_rad = 0.0f;
    my_calc_pitch_rad = 0.0f;
    my_calc_yaw_rad = 0.0f;
    last_run_us = AP_HAL::micros();

    my_fusion.initialize();
    
    hal.console->printf("UserCode: FusionAhrsAdaptor Initialized.\n");
}

void Copter::userhook_FastLoop()
{
    uint32_t now = AP_HAL::micros();
    float dt = (now - last_run_us) * 1.0e-6f;
    last_run_us = now;
    if (dt > 0.1f || dt <= 0.0f) dt = 0.0025f;

    Vector3f accel = ins.get_accel();
    Vector3f gyro = ins.get_gyro();
    
    Vector3f mag_field;
    if (compass.use_for_yaw()) {
        mag_field = compass.get_field(); 
    } else {
        mag_field.zero();
    }

    run_my_estimation(
        accel.x, accel.y, accel.z,
        gyro.x, gyro.y, gyro.z,
        mag_field.x, mag_field.y, mag_field.z,
        dt
    );
}

void Copter::userhook_50Hz()
{

    AP::logger().Write_Debug("MyRoll", my_calc_roll_rad);
    AP::logger().Write_Debug("APRoll", ahrs.get_roll());

    AP::logger().Write_Debug("MyPitch", my_calc_pitch_rad);
    AP::logger().Write_Debug("APPitch", ahrs.get_pitch());

    AP::logger().Write_Debug("MyYaw", my_calc_yaw_rad);
    AP::logger().Write_Debug("APYaw", ahrs.get_yaw());
}

void Copter::userhook_MediumLoop() {}
void Copter::userhook_SlowLoop() {}
void Copter::userhook_SuperSlowLoop() {}
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag) {}
void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag) {}
void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag) {}