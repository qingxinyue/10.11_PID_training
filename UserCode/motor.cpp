//
// Created by qingx on 2026/10/11.
//

#include "motor.hpp"
motor::motor(Type_e type,float ratio,ControlMethod_e controlMethod,PID ppid,PID spid,uint8_t motor_id, float intensity_max):
    type_(type),ratio_(ratio),controlMethod_(controlMethod),ppid_(ppid),spid_(spid),
    motor_id_(motor_id),intensity_max_(intensity_max)
{
    ppid.reset();
    spid.reset();
}

float motor::angle() const
{
    return angle_;
}

float motor::speedRpm() const
{
    return speedRpm_;
}

bool motor::hasFeedBack() const
{
    return received_;
}

bool motor::feedbackValid(uint32_t now_ms) const
{
    return received_ && (now_ms-last_rx_ms_)<=100;
}

void motor::setPosition(float target_position,float feedforward_speed,float feedforward_intensity)
{
    controlMethod_=POSITION_SPEED;
    target_angle_=target_position;
    feedforward_speed_=feedforward_speed;
    feedforward_intensity_=feedforward_intensity;
}

void motor::setSpeed(float target_speed,float feedforward_intensity)
{
    controlMethod_=SPEED;
    target_speed_=target_speed;
    feedforward_speed_=0.0f;
    feedforward_intensity_=feedforward_intensity;
}

void motor::setIntensity(float intensity)
{
    controlMethod_=TORQUE;
    target_intensity_=intensity;
    feedforward_speed_=0.0f;
    feedforward_intensity_=0.0f;
}

void motor::resetMotor()
{
    ppid_.reset();
    spid_.reset();
    intensity_=0.0f;
}

void motor::setEnabled(bool enabled)
{
    if (enabled_!=enabled) resetMotor();
    enabled_=enabled;
    if (!enabled_) setTxIntensity(0,motor_id_);
}

void motor::canRxMsgCallback(const uint8_t rx_data[8])
{
    uint16_t angle_raw = (static_cast<uint16_t>(rx_data[0]) << 8) | static_cast<uint16_t>(rx_data[1]);
    ecd_angle_ = static_cast<float>(angle_raw) / kEncoderRange* 360.0f;
    int16_t raw_speedRpm_ = (static_cast<int16_t>(rx_data[2]) << 8) | static_cast<int16_t>(rx_data[3]);
    speedRpm_ = static_cast<float>(raw_speedRpm_);
    int16_t raw_currentAmps_ = (static_cast<int16_t>(rx_data[4]) << 8) | static_cast<int16_t>(rx_data[5]);
    currentAmps_ = static_cast<float>(raw_currentAmps_);

    last_rx_ms_ = HAL_GetTick();
    if (!received_)
    {
        received_ = true;
        last_ecd_ = ecd_angle_;
        return;
    }

    float delta = ecd_angle_ - last_ecd_;
    if (delta >180.0f) delta -= 360.0f;
    if (delta <-180.0f) delta += 360.0f;
    angle_ += delta/ratio_;
    last_ecd_ = ecd_angle_;
}

void motor::setTxIntensity(int16_t intensity, uint8_t motor_id)
{
    if (motor_id < 1 ||motor_id > 4) return;
    const uint16_t raw=static_cast<uint16_t>(intensity);

    const unsigned offset =2*(motor_id-1);
    tx_data_[offset] = static_cast<uint8_t>(raw>> 8);
    tx_data_[offset+1] = static_cast<uint8_t>(raw& 0xFF);
}

uint8_t* motor::getTxData()
{
    return tx_data_;
}

void motor::handle()
{
    uint32_t now =HAL_GetTick();

    if (!enabled_ || !feedbackValid(now))
    {
        resetMotor();
        intensity_ = 0;
        setTxIntensity(0,motor_id_);
        return;
    }

    const float fdb_angle = angle();
    const float fdb_speed = speedRpm()*6.0f /ratio_;

    switch (controlMethod_)
    {
    case POSITION_SPEED:
        {
            float speed_ref =ppid_.calc(target_angle_,fdb_angle)+feedforward_speed_;
            speed_ref =ppid_.limit(speed_ref, -ppid_.out_max_, ppid_.out_max_);
            intensity_ =spid_.calc(speed_ref,fdb_speed)+feedforward_intensity_;
            break;
        }
    case SPEED:
        {
            intensity_ =spid_.calc(target_speed_,fdb_speed)+feedforward_intensity_;
            break;
        }
    case TORQUE:
        {
            intensity_=target_intensity_;
            break;
        }
    default:
        {
            intensity_=0.0f;
            break;
        }
    }
    intensity_=spid_.limit(intensity_, -intensity_max_,intensity_max_);
    setTxIntensity(static_cast<int16_t>(intensity_),motor_id_);
}

