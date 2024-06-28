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
#include "uart.h"

#include <algorithm>
#include <cmath>
#include <array>

struct MotorControlParams {
	double pixel_count=0.0;	//
	double timer_reload_value=2500.0;
	double pixel_count_setpoint=1024.0;
	const double kp=1.0;
	const double ki=0.5;
	const double kd=0.0;
};

const double kMinReloadValue=1000.0;
const double kMaxReloadValue=20000.0;
const float kMinPwm=0.0f;
const float kMaxPwm=200.0f;
const double kMinMotorFrequency=25.0;


static uint32_t revolution_counter_=0;
static double motor_frequency_=0.0;
static MotorControlParams motor_params_;
static double red_modulator_=0.0;
static double green_modulator_=0.0;
static double blue_modulator_=0.0;


static PID motor_pid_;


void CannonControlInit() {
	UartBoot();
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
	for(int i=0;i<5;i++) {										// turn on the motor.  Super Slow enable PWM'ing permits slower accereration so the motor controller doesn't throw an error
		HAL_GPIO_WritePin(MotorEnable_GPIO_Port, MotorEnable_Pin, GPIO_PIN_SET);
		HAL_Delay(1500);
		HAL_GPIO_WritePin(MotorEnable_GPIO_Port, MotorEnable_Pin, GPIO_PIN_RESET);
		HAL_Delay(500);
	}
	HAL_GPIO_WritePin(MotorEnable_GPIO_Port, MotorEnable_Pin, GPIO_PIN_SET);		// turn on the motor
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim6);
	motor_pid_ = PID(&motor_params_.pixel_count, &motor_params_.timer_reload_value, &motor_params_.pixel_count_setpoint,motor_params_.kp, motor_params_.ki, motor_params_.kd, REVERSE);
	motor_pid_.SetOutputLimits(kMinReloadValue, kMaxReloadValue);
	motor_pid_.SetMode(AUTOMATIC);
}

void CannonControlMain() {	// idle loop
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

	revolution_counter_++;
	motor_pid_.Compute();
	motor_params_.pixel_count=0;
	red_modulator_+=1.0;
	green_modulator_+=0.6;
	blue_modulator_+=4.2;
	if(red_modulator_>=motor_params_.pixel_count_setpoint) red_modulator_=0.0;
	if(green_modulator_>=motor_params_.pixel_count_setpoint) green_modulator_=0.0;
	if(blue_modulator_>=motor_params_.pixel_count_setpoint) blue_modulator_=0.0;
}

void CalculatePixel() {					// runs at each pixel
	motor_params_.pixel_count+=1.0;
	double red_positioner = std::fmod(motor_params_.pixel_count+red_modulator_,motor_params_.pixel_count_setpoint);
	double green_positioner = std::fmod(motor_params_.pixel_count+green_modulator_,motor_params_.pixel_count_setpoint);
	double blue_positioner = std::fmod(motor_params_.pixel_count+blue_modulator_,motor_params_.pixel_count_setpoint);
	float red_portion=0.0f;
	float green_portion=0.0f;
	float blue_portion=0.0f;
	if(red_positioner<(motor_params_.pixel_count_setpoint*.3)) red_portion=1.0f;
	if((green_positioner>=(motor_params_.pixel_count_setpoint*.25)) && (green_positioner<(motor_params_.pixel_count_setpoint*.6))) green_portion=1.0f;
	if((blue_positioner>=(motor_params_.pixel_count_setpoint*.5)) && (blue_positioner<(motor_params_.pixel_count_setpoint*.9))) blue_portion=1.0f;


//	const double kNumBands=4.0;
//	double high_val=motor_params_.pixel_count_setpoint / kNumBands;
//	double answer = std::fmod(motor_params_.pixel_count, high_val);
//	double pixel_percent=0.0;
//	if(answer < (high_val/2.0)) pixel_percent=100.0;
//	if(motor_params_.pixel_count < (motor_params_.pixel_count_setpoint/2.0)) pixel_percent=100.0;
	uint8_t red_pulsewidth = kMinPwm+ static_cast<uint8_t>(red_portion*(kMaxPwm-kMinPwm));
	red_pulsewidth = std::clamp(red_pulsewidth, static_cast<uint8_t>(kMinPwm), static_cast<uint8_t>(kMaxPwm));

	uint8_t green_pulsewidth = kMinPwm+ static_cast<uint8_t>(green_portion*(kMaxPwm-kMinPwm));
	green_pulsewidth = std::clamp(green_pulsewidth, static_cast<uint8_t>(kMinPwm), static_cast<uint8_t>(kMaxPwm));

	uint8_t blue_pulsewidth = kMinPwm+ static_cast<uint8_t>(blue_portion*(kMaxPwm-kMinPwm));
	blue_pulsewidth = std::clamp(blue_pulsewidth, static_cast<uint8_t>(kMinPwm), static_cast<uint8_t>(kMaxPwm));

	if(motor_frequency_ < kMinMotorFrequency) {
		red_pulsewidth=0;
		green_portion=0;
		blue_portion=0;
	}
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, red_pulsewidth);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, green_pulsewidth);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, blue_pulsewidth);
	__HAL_TIM_SET_AUTORELOAD(&htim6,  static_cast<uint16_t>(motor_params_.timer_reload_value));
}

void CalculateMotorFrequency() {	// runs at 100Hz
	static uint32_t last_revolution_counter=revolution_counter_;
	static uint32_t last_millis = HAL_GetTick();
	uint32_t revs = revolution_counter_ - last_revolution_counter;
	if(revs==0) return;
	last_revolution_counter+=revs;
	uint32_t millis = HAL_GetTick();
	uint32_t period_ms = std::max(millis - last_millis,1UL);
	last_millis=millis;
	double period_per_rev = static_cast<double>(period_ms) / static_cast<double>(revs);
	motor_frequency_ = MathUtilsIIRFilter(motor_frequency_, 1000.0 / static_cast<double>(period_per_rev), 0.1);
	uint8_t buffer[256];

//	uint32_t numbytes = sprintf((char*)buffer,"%f\n", motor_frequency_);
//	uint32_t numbytes = UartDMARead(kUartCom, buffer, sizeof(buffer));
//	if(numbytes>0) UartDMAWrite(kUartCom,buffer, numbytes);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance==TIM6) {
		CalculatePixel();
		return;
	}
	if(htim->Instance==TIM7) {
		CalculateMotorFrequency();
		return;
	}

}
