/*
 * CannonControl.cpp
 *
 *  Created on: Jun 25, 2024
 *      Author: eriks
 */

#include "main.h"
#include "CannonControl.h"
#include "pid.h"
#include "MathUtils.h"

#include <algorithm>
#include <cmath>
#include <array>

const double kMinReloadValue=1000.0;
const double kMaxReloadValue=20000.0;

struct MotorControlParams {
	double pixel_count=0.0;	//
	double timer_reload_value=2500.0;
	double pixel_count_setpoint=1024.0;
	const double kp=1.0;
	const double ki=0.5;
	const double kd=0.0;
};


const float kMinPwm=0.0f;
const float kMaxPwm=140.0f;

static MotorControlParams motor_params_;


static PID motor_pid_;


void CannonControlInit() {
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MotorControl_GPIO_Port, MotorControl_Pin, GPIO_PIN_SET);		// turn on the motor
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//	reload_value_ = __HAL_TIM_GET_AUTORELOAD(&htim6);
	HAL_TIM_Base_Start_IT(&htim6);
	motor_pid_ = PID(&motor_params_.pixel_count, &motor_params_.timer_reload_value, &motor_params_.pixel_count_setpoint,
					 motor_params_.kp, motor_params_.ki, motor_params_.kd, REVERSE);
	motor_pid_.SetOutputLimits(kMinReloadValue, kMaxReloadValue);
	motor_pid_.SetMode(AUTOMATIC);
}

void CannonControlMain() {
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
	HAL_Delay(500);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {	// interrupt for once per revolution
// this until we get the real tachometer working
	static int tri_divider=0;
	tri_divider++;
	if(tri_divider<6) return;
	tri_divider=0;
// end fake tachometer

	motor_pid_.Compute();
	motor_params_.pixel_count=0;

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	motor_params_.pixel_count+=1.0;
	const double kNumBands=4.0;
	double high_val=motor_params_.pixel_count_setpoint / kNumBands;
	double answer = std::fmod(motor_params_.pixel_count, high_val);
	double pixel_percent=0.0;
	if(answer < (high_val/2.0)) pixel_percent=100.0;

//	if(motor_params_.pixel_count < (motor_params_.pixel_count_setpoint/2.0)) pixel_percent=100.0;




	uint8_t pulsewidth = kMinPwm+ static_cast<uint8_t>(pixel_percent*(kMaxPwm-kMinPwm));
	pulsewidth = std::clamp(pulsewidth, static_cast<uint8_t>(kMinPwm), static_cast<uint8_t>(kMaxPwm));

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulsewidth);
	__HAL_TIM_SET_AUTORELOAD(&htim6,  static_cast<uint16_t>(motor_params_.timer_reload_value));
}
