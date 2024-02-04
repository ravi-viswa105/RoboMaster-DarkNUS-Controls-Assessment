/*
 * movement_control_task.c
 *
 *  Created on: Jan 19, 2021
 *      Author: Hans Kurnia
 */

#include "board_lib.h"
#include "robot_config.h"
#include "motor_config.h"
#include "motor_control.h"
#include "arm_math.h"
#include "movement_control_task.h"

extern EventGroupHandle_t chassis_event_group;

extern chassis_control_t chassis_ctrl_data;
extern motor_data_t can_motors[24];
extern referee_limit_t referee_limiters;
extern speed_shift_t gear_speed;
float g_chassis_yaw = 0;

float motor_yaw_mult[4];

extern QueueHandle_t telem_motor_queue;

void movement_control_task(void *argument) {
	TickType_t start_time;
	//initialise in an array so it's possible to for-loop it later
	motor_yaw_mult[0] = FR_YAW_MULT;
	motor_yaw_mult[1] = FL_YAW_MULT;
	motor_yaw_mult[2] = BL_YAW_MULT;
	motor_yaw_mult[3] = BR_YAW_MULT;
	while (1) {

#ifndef CHASSIS_MCU

		EventBits_t motor_bits;
		//wait for all motors to have updated data before PID is allowed to run
		motor_bits = xEventGroupWaitBits(chassis_event_group, 0b1111, pdTRUE,
				pdTRUE,
				portMAX_DELAY);
		if (motor_bits == 0b1111) {
			status_led(3, on_led);
			start_time = xTaskGetTickCount();
			if (chassis_ctrl_data.enabled) {
				chassis_motion_control(can_motors + FR_MOTOR_ID - 1,
						can_motors + FL_MOTOR_ID - 1,
						can_motors + BL_MOTOR_ID - 1,
						can_motors + BR_MOTOR_ID - 1);
			} else {
				can_motors[FR_MOTOR_ID - 1].rpm_pid.output = 0;
				can_motors[FL_MOTOR_ID - 1].rpm_pid.output = 0;
				can_motors[BL_MOTOR_ID - 1].rpm_pid.output = 0;
				can_motors[BR_MOTOR_ID - 1].rpm_pid.output = 0;
				g_chassis_yaw = 0;

				//change CAN messages to a seperate task? so it doesn't fill up CAN transmitter
				motor_send_can(can_motors, FR_MOTOR_ID, FL_MOTOR_ID,
						BL_MOTOR_ID,
						BR_MOTOR_ID);
			}
#else
		chassis_MCU_send_CAN();
#endif
			status_led(3, off_led);
		} else {
			//motor timed out
			can_motors[FR_MOTOR_ID - 1].rpm_pid.output = 0;
			can_motors[FL_MOTOR_ID - 1].rpm_pid.output = 0;
			can_motors[BL_MOTOR_ID - 1].rpm_pid.output = 0;
			can_motors[BR_MOTOR_ID - 1].rpm_pid.output = 0;
			motor_send_can(can_motors, FR_MOTOR_ID, FL_MOTOR_ID, BL_MOTOR_ID,
					BR_MOTOR_ID);
		}
		//clear bits if it's not already cleared
		xEventGroupClearBits(chassis_event_group, 0b1111);
		//delays task for other tasks to run
		vTaskDelayUntil(&start_time, CHASSIS_DELAY);
	}
	osThreadTerminate(NULL);
}
void chassis_MCU_send_CAN() {

	uint8_t CAN_send_data[8] = { 0, };
	memcpy(CAN_send_data, &chassis_ctrl_data, sizeof(CAN_send_data));
	CAN_TxHeaderTypeDef CAN_tx_message;
	uint32_t send_mail_box;
	CAN_tx_message.IDE = CAN_ID_STD;
	CAN_tx_message.RTR = CAN_RTR_DATA;
	CAN_tx_message.DLC = 0x08;

	if (chassis_ctrl_data.enabled) {
		CAN_tx_message.StdId = 0x111;
		HAL_CAN_AddTxMessage(&hcan2, &CAN_tx_message, CAN_send_data,
				&send_mail_box);
	} else {
		g_chassis_yaw = 0;
		CAN_tx_message.StdId = 0x100;
		HAL_CAN_AddTxMessage(&hcan2, &CAN_tx_message, CAN_send_data,
				&send_mail_box);
	}
}

void chassis_motion_control(motor_data_t *motorfr, motor_data_t *motorfl,
		motor_data_t *motorbl, motor_data_t *motorbr) {
	static uint32_t prev_time;
	//get the angle between the gun and the chassis
	//so that movement is relative to gun, not chassis
	float rel_angle = can_motors[YAW_MOTOR_ID - 1].angle_data.adj_ang;
	float translation_rpm[4] = { 0, };
	;
	float yaw_rpm[4] = { 0, };
	float total_power = 0;
	int32_t chassis_rpm = LV1_MAX_SPEED;
	int32_t chassis_current = LV1_MAX_CURRENT;
	if (referee_limiters.robot_level == 1) {
		chassis_rpm = LV1_MAX_SPEED;
		chassis_current = LV1_MAX_CURRENT;
	} else if (referee_limiters.robot_level == 2) {
		chassis_rpm = LV2_MAX_SPEED;
		chassis_current = LV2_MAX_CURRENT;
	} else if (referee_limiters.robot_level == 3) {
		chassis_rpm = LV3_MAX_SPEED;
		chassis_current = LV3_MAX_CURRENT;
	}
//	chassis_current *= referee_limiters.wheel_power_limit
//			* referee_limiters.wheel_buffer_limit * gear_speed.accel_mult;
	chassis_current =
			(chassis_current > M3508_MAX_OUTPUT) ?
					M3508_MAX_OUTPUT : chassis_current;
	chassis_rpm = (chassis_rpm > M3508_MAX_RPM) ? M3508_MAX_RPM : chassis_rpm;
	//rotate angle of the movement :)
	//MA1513/MA1508E is useful!!
	float rel_forward = ((-chassis_ctrl_data.horizontal * sin(-rel_angle))
			+ (chassis_ctrl_data.forward * cos(-rel_angle)));
	float rel_horizontal = ((-chassis_ctrl_data.horizontal * cos(-rel_angle))
			+ (chassis_ctrl_data.forward * -sin(-rel_angle)));
	float rel_yaw = chassis_ctrl_data.yaw;


	uint32_t micros_time_diff = get_microseconds() - prev_time;
	prev_time = get_microseconds();
	float yaw_max_accel = CHASSIS_MAX_YAW_ACCEL * micros_time_diff / TIMER_FREQ;
	float lin_max_accel = CHASSIS_MAX_ACCEL * micros_time_diff / TIMER_FREQ ;
	static float prev_fwd;
	static float prev_horz;
	static float prev_yaw;


	swerve_turn(0, (rel_yaw - 0.785), chassis_rpm * 0.25);
	swerve_turn(1, (rel_yaw + 0.785), chassis_rpm);
	swerve_turn(2, (rel_yaw - 0.785), chassis_rpm);
	swerve_turn(3, (rel_yaw + 0.785), chassis_rpm * 0.25);



	translation_rpm[0] = ((rel_forward * FR_VY_MULT)
			+ (rel_horizontal * FR_VX_MULT));
	yaw_rpm[0] =
			rel_yaw * motor_yaw_mult[0] * CHASSIS_YAW_MAX_RPM;
	translation_rpm[1] = ((rel_forward * FL_VY_MULT)
			+ (rel_horizontal * FL_VX_MULT));
	yaw_rpm[1] =
			rel_yaw * motor_yaw_mult[1] * CHASSIS_YAW_MAX_RPM;
	translation_rpm[2] = ((rel_forward * BL_VY_MULT)
			+ (rel_horizontal * BL_VX_MULT));
	yaw_rpm[2] =
			rel_yaw * motor_yaw_mult[2] * CHASSIS_YAW_MAX_RPM;
	translation_rpm[3] = ((rel_forward * BR_VY_MULT)
			+ (rel_horizontal * BR_VX_MULT));
	yaw_rpm[3] =
			rel_yaw * motor_yaw_mult[3] * CHASSIS_YAW_MAX_RPM;

	float rpm_mult = 0.0;
	float yaw_scale = 1;
	float trans_scale = 1;
	for (uint8_t i = 0; i < 4; i++) {
		float abs_total = fabs(translation_rpm[i] + yaw_rpm[i]);
		float abs_yaw = fabs(yaw_rpm[i]);
		float abs_trans = fabs(translation_rpm[i]);
		if (abs_total > 1) {
			if (abs_trans < CHASSIS_TRANS_PRIO) {
				if (1 - abs_trans < yaw_scale) {
					yaw_scale = (1 - abs_trans) / abs_yaw;
				}
			} else if (abs_yaw < CHASSIS_YAW_PRIO) {
				if ((1 - abs_yaw) < trans_scale) {
					trans_scale = (1 - abs_yaw) / abs_trans;
				}
			} else {
				if ((CHASSIS_YAW_PRIO / abs_yaw) < yaw_scale) {
					yaw_scale = CHASSIS_YAW_PRIO / abs_yaw;
				}
				if (translation_rpm[i] > CHASSIS_TRANS_PRIO) {
					if (CHASSIS_TRANS_PRIO / abs_trans < trans_scale) {
						trans_scale = CHASSIS_TRANS_PRIO / abs_trans;
					}
				}
			}
		}
	}
	g_chassis_yaw = yaw_scale;

	for (uint8_t j = 0; j < 4; j++) {
		translation_rpm[j] = (translation_rpm[j] * trans_scale
				* gear_speed.trans_mult
				+ yaw_rpm[j] * yaw_scale * gear_speed.spin_mult) * chassis_rpm;
	}

//	motorfr->rpm_pid.max_out = chassis_current;
//	motorfl->rpm_pid.max_out = chassis_current;
//	motorbl->rpm_pid.max_out = chassis_current;
//	motorbr->rpm_pid.max_out = chassis_current;

	speed_pid(translation_rpm[0], motorfr->raw_data.rpm,
			&motorfr->rpm_pid);
	total_power += fabs(motorfr->rpm_pid.output);
	speed_pid(translation_rpm[1], motorfl->raw_data.rpm,
			&motorfl->rpm_pid);
	total_power += fabs(motorfl->rpm_pid.output);
	speed_pid(translation_rpm[2], motorbl->raw_data.rpm,
			&motorbl->rpm_pid);
	total_power += fabs(motorbl->rpm_pid.output);
	speed_pid(translation_rpm[3], motorbr->raw_data.rpm,
			&motorbr->rpm_pid);
	total_power += fabs(motorbr->rpm_pid.output);

//	if (total_power > (referee_limiters.wheel_power_limit * power_multiplier))
//	{
//		float power_ratio = referee_limiters.wheel_power_limit/total_power;
//		motorfr->rpm_pid.output *= power_ratio;
//		motorfl->rpm_pid.output *= power_ratio;
//		motorbl->rpm_pid.output *= power_ratio;
//		motorbr->rpm_pid.output *= power_ratio;
//	}

	motor_send_can(can_motors, FR_MOTOR_ID, FL_MOTOR_ID, BL_MOTOR_ID,
	BR_MOTOR_ID);
}

