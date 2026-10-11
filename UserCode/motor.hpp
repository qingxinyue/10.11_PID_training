//
// Created by qingx on 2026/10/11.
//

#ifndef INC_1011_PID_MOTOR_HPP
#define INC_1011_PID_MOTOR_HPP
#include "main.h"
#include "PID.hpp"
#include "motor.hpp"

class motor
{
public:
    typedef enum Type{
        M3508,
        M2006,
    } Type_e;

    typedef enum ControlMethod
    {
        POSITION_SPEED,
        SPEED,
        TORQUE,
    } ControlMethod_e;
public:
    explicit motor(Type_e type,float ratio,ControlMethod_e controlMethod,PID ppid,PID spid,uint8_t motor_id_,float intensity_max_);

    void canRxMsgCallback(const uint8_t rx_data[8]);
    float angle() const;
    float speedRpm() const;
    float currentAmps() const;
    float temperature() const;
    bool hasFeedBack() const;

    void handle();
    void setPosition(float target_position,float feedforward_speed, float feedforward_intensity);
    void setSpeed(float target_speed, float feedforward_intensity);
    void setIntensity(float intensity);
    void resetMotor();
    void setTxIntensity(int16_t intensity, uint8_t motor_id);
    bool feedbackValid(uint32_t now_ms) const;
    void setEnabled(bool enabled);
    uint8_t* getTxData();

private:
    const float ratio_;
    float ecd_angle_=0.0f;
    float speedRpm_=0.0f;
    float currentAmps_=0.0f;

    float angle_=0.0f;
    float last_ecd_=0.0f;
    PID ppid_,spid_;
    Type_e type_;
    ControlMethod_e controlMethod_;

    float target_angle_=0.0f;
    float fdb_angle_=0.0f;
    float target_speed_=0.0f;
    float fdb_speed_=0.0f;
    float feedforward_speed_=0.0f;
    float target_intensity_=0.0f;
    float feedforward_intensity_=0.0f;
    float intensity_=0.0f;

    bool enabled_=false;
    bool received_=false;
    uint32_t last_rx_ms_=0;
    uint8_t motor_id_;
    float intensity_max_;

    uint8_t tx_data_[8];
    static constexpr uint16_t kEncoderRange =8192;

    float gravity_zero_offset_deg_ = 0.0f;
    float gravity_ff_max_ = 0.0f; 
};


#endif //INC_1011_PID_MOTOR_HPP
