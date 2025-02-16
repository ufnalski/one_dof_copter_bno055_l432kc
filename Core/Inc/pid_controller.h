/*
 * pid_controller.h
 *
 *  Created on: Feb 12, 2025
 *      Author:
 */

/* pid_controller.h */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

typedef struct
{
	float kp;
	float ki;
	float kd;
	float tau;
	float dt;
	float integrator;
	float differentiator;
	float previous_error;
	float previous_feedback;
	float previous_integrator;
	float output;
	float output_min;
	float output_max;
	float integral_min; // not used in PID_Init_Bartek_s_Lab
	float integral_max; // not used in PID_Init_Bartek_s_Lab
} PID_t;

void PID_Init_Phil_s_Lab(PID_t *_pid, float _kp, float _ki, float _kd,
		float _tau, float _output_min, float _output_max, float _integral_min,
		float _integral_max, float _dt);

void PID_Init_Bartek_s_Lab(PID_t *_pid, float _kp, float _ki, float _kd,
		float _tau, float _output_min, float _output_max, float _dt);

float PID_Controller_Phil_s_Lab(PID_t *_pid, float _reference, float _feedback);

float PID_Controller_Bartek_s_Lab(PID_t *_pid, float _reference,
		float _feedback);

#endif // PID_CONTROLLER_H
