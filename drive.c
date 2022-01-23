#include <stdio.h>
#include "coroutine.h"
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"
#include <stdlib.h>

#define LAP_POS1 0
#define LAP_POS2 0
#define LAP_POS3 1

#include <unistd.h>
#define Sleep(msec) usleep((msec)*1000)

#define L_MOTOR_PORT OUTPUT_C
#define L_MOTOR_EXT_PORT EXT_PORT__NONE_
#define R_MOTOR_PORT OUTPUT_B
#define R_MOTOR_EXT_PORT EXT_PORT__NONE_
#define OBS_MOTOR_PORT OUTPUT_A
#define OBS_MOTOR_EXT_PORT EXT_PORT__NONE_
#define IR_CHANNEL 0

#define SPEED_LINEAR 75	  /* Motor speed for linear motion, in percents */
#define SPEED_CIRCULAR 50 /* ... for circular motion */
#define HISTORY_LENGTH 200

#define WHEEL_DIAMETER 5.6
#define PI 3.141592
#define WHEEL_CONSTANT ((PI * WHEEL_DIAMETER) / 360)
#define DISTANCE_CONSTANT (1 / WHEEL_CONSTANT)

#define _DISTANCE_MAX 260 /* 10 cm */
#define _DISTANCE_MIN 200 /* 5 cm */

#define BACKWARDS_STEP 1 /* 5 cm */
#define FORWARD_STEP 3
#define DISTANCE_MIN_ANGLE_CORR 240
#define COMP_VALUE 25

#define ANGLE_OUTLIER 360
#define DISTANCE_OBSTACLE 100

#define LS1 20
#define SS1 20
#define LS2 20

#define ANGLE_THRESHOLD 5

/* Threshold distances for each sector */
#define START_DISTANCE 50
#define FORWARD1_DISTANCE 15
#define FORWARD1_DISTANCE_START 40
#define FORWARD2_DISTANCE 145
#define FORWARD3_DISTANCE 40
#define FORWARD4_DISTANCE 40
#define FORWARD5_DISTANCE 30
#define FORWARD6_DISTANCE 120

int max_speed; /* Motor maximal speed */

#define DEGREE_TO_COUNT(d) ((d)*260 / 90)

int app_alive, n_lap;

int DISTANCE_MAX = _DISTANCE_MAX; /* 10 cm */
int DISTANCE_MIN = _DISTANCE_MIN; /* 5 cm */

enum
{
	MODE_REMOTE, /* IR remote control */
	MODE_AUTO,	 /* Self-driving */
};

int mode; /* Driving mode */

enum
{
	MOVE_NONE,
	MOVE_FORWARD,
	MOVE_FORWARD_CORRECTION,
	MOVE_BACKWARD,
	TURN_LEFT,
	TURN_RIGHT,
	TURN_ANGLE,
	TURN_ANGLE_COMPASS,
	STEP_BACKWARD,
};

enum
{
	STATE_START,
	STATE_FORWARD1,
	STATE_STILL,
	STATE_FORWARD2,
	STATE_FORWARD3,
	STATE_FORWARD3_5,
	STATE_DROP_OBS,
	STATE_FORWARD4,
	STATE_FORWARD5,
	STATE_FORWARD6,
	STATE_PROXIMITY_CORRECTION,
	STATE_ANGLE_CORRECTION,
	STATE_REG_LAP,
	STATE_PROXIMITY_CORRECTION_BEFORE_ANGLE,
	STATE_PROXIMITY_OBSTACLE,
	STATE_GYRO_CAL_BUTTON
};

int moving;	 /* Current moving */
int command; /* Command for the 'drive' coroutine */
int angle;	 /* Angle of rotation */
int gyro_angle, gyro_zero_angle;
int proximity;
float walked_dist;
int start_degrees_wheel;
int rel_walk;
int state, next_state, next_state1;
int expected_angle;
int lap;
int touch_pressed;
int compass_value;
int left_right;
int lateral_tras;

uint8_t ir, gyro, touch, compass; /* Sequence numbers of sensors */
enum
{
	L,
	R
};
uint8_t motor[3] = {DESC_LIMIT, DESC_LIMIT, DESC_LIMIT}; /* Sequence numbers of motors */
uint8_t obs_motor;

static void _set_mode(int value)
{
	if (value == MODE_AUTO)
	{
		/* IR measuring of distance */
		set_sensor_mode_inx(ir, LEGO_EV3_IR_IR_PROX);
		mode = MODE_AUTO;
	}
	else
	{
		/* IR remote control */
		set_sensor_mode_inx(ir, LEGO_EV3_IR_IR_REMOTE);
		mode = MODE_REMOTE;
	}
}

static void _run_forever(int l_speed, int r_speed)
{
	set_tacho_speed_sp(motor[L], l_speed);
	set_tacho_speed_sp(motor[R], r_speed);
	multi_set_tacho_command_inx(motor, TACHO_RUN_FOREVER);
}

static void _run_to_rel_pos(int l_speed, int l_pos, int r_speed, int r_pos)
{
	set_tacho_speed_sp(motor[L], l_speed);
	set_tacho_speed_sp(motor[R], r_speed);
	set_tacho_position_sp(motor[L], l_pos);
	set_tacho_position_sp(motor[R], r_pos);
	multi_set_tacho_command_inx(motor, TACHO_RUN_TO_REL_POS);
}

static void _run_timed(int l_speed, int r_speed, int ms)
{
	set_tacho_speed_sp(motor[L], l_speed);
	set_tacho_speed_sp(motor[R], r_speed);
	multi_set_tacho_time_sp(motor, ms);
	multi_set_tacho_command_inx(motor, TACHO_RUN_TIMED);
}

static int _is_running(void)
{
	FLAGS_T state = TACHO_STATE__NONE_;

	get_tacho_state_flags(motor[L], &state);
	if (state != TACHO_STATE__NONE_)
		return (1);
	get_tacho_state_flags(motor[R], &state);
	if (state != TACHO_STATE__NONE_)
		return (1);

	return (0);
}

static void _stop(void)
{
	multi_set_tacho_command_inx(motor, TACHO_STOP);
}

static void _get_tacho_position(int *pos)
{
	get_tacho_position(motor[R], pos);
}

int app_init(void)
{
	char s[16];

	if (ev3_search_tacho_plugged_in(L_MOTOR_PORT, L_MOTOR_EXT_PORT, motor + L, 0))
	{
		get_tacho_max_speed(motor[L], &max_speed);
		/* Reset the motor */
		set_tacho_command_inx(motor[L], TACHO_RESET);
	}
	else
	{
		printf("LEFT motor (%s) is NOT found.\n", ev3_port_name(L_MOTOR_PORT, L_MOTOR_EXT_PORT, 0, s));
		/* Inoperative without left motor */
		return (0);
	}
	if (ev3_search_tacho_plugged_in(R_MOTOR_PORT, R_MOTOR_EXT_PORT, motor + R, 0))
	{
		/* Reset the motor */
		set_tacho_command_inx(motor[R], TACHO_RESET);
	}
	else
	{
		printf("RIGHT motor (%s) is NOT found.\n", ev3_port_name(R_MOTOR_PORT, R_MOTOR_EXT_PORT, 0, s));
		/* Inoperative without right motor */
		return (0);
	}
	mode = MODE_AUTO;

	ir = 3;

	if (ev3_search_sensor(LEGO_EV3_US, &ir, 0))
	{
		printf(" use the IR sensor.\n");
	}

	if (ev3_search_sensor(LEGO_EV3_GYRO, &gyro, 0))
	{
		printf(" use the GYRO sensor.\n");
	}

	if (ev3_search_sensor(LEGO_EV3_TOUCH, &touch, 0))
	{
		printf(" use the TOUCH sensor.\n");
	}

	if (ev3_search_sensor(HT_NXT_COMPASS, &compass, 0))
	{
		printf(" use the COMPASS sensor.\n");
	}
	lap = 0;

	n_lap = 0;
	left_right = 1;
	printf("Init State: %d\n", state);
	_get_tacho_position(&start_degrees_wheel);	 /* Reset the travelled distance to 0 */
	get_sensor_value(0, gyro, &gyro_zero_angle); /* Reset the starting angle to 0 */

#if LAP_POS3
	state = STATE_FORWARD2;
	gyro_zero_angle += 180;
#else
	state = STATE_START;
#endif

	return (1);
}

int state_forward(int proximity, float walked, float walked_thresh, int state_within_thresh, int state_after_tresh);
int state_forward_no_prox_check(int proximity, float walked, float walked_thresh, int state_within_thresh, int state_after_tresh);
int get_average_sensor2(uint8_t sensor, int samples, int thresh)
{

	int array_value[samples];
	int value = 0, average = 0, i, j;
	int moda = 0, counter_m = 0;
	get_sensor_value(0, sensor, &value);
	average += value;

	printf("\n");
	for (i = 0; i < samples; i++)
	{
		get_sensor_value(0, sensor, &value);
		array_value[i] = value;
		average = (average + value) / 2;
		printf(" %d", value);
	}
	printf("\n");
	for (i = 0; i < samples; i++)
	{
		moda = array_value[i];
		counter_m = 0;
		for (int j = 0; j < samples; j++)
		{

			if (array_value[j] > moda - thresh && array_value[j] < moda + thresh)
			{
				counter_m++;
			}
		}

		if (counter_m >= samples / 2)
		{
			for (i = 0; i < samples; i++)
			{
				if (!(array_value[i] > moda - thresh && array_value[i] < moda + thresh))
				{
					printf("OUTLIER %d, MODA %d\n", array_value[i], moda);
					array_value[i] = moda;
				}
			}
			break;
		}
	}

	average = 0;

	for (i = 0; i < samples; i++)
	{
		average += array_value[i];
	}

	average /= samples;
	printf("AVERAGE %d\n", average);
	return average;
}

int get_average_sensor(uint8_t sensor, int samples)
{

	int value = 0, average = 0, i;

	get_sensor_value(0, sensor, &value);
	average += value;

	for (i = 0; i < samples; i++)
	{
		get_sensor_value(0, sensor, &value);
		average = (average + value) / 2;
	}
	return average;
}

CORO_CONTEXT(drive);
CORO_CONTEXT(get_proximity);
CORO_CONTEXT(get_distance);
CORO_CONTEXT(get_angle);
CORO_CONTEXT(get_touch);
CORO_CONTEXT(DFA);

CORO_DEFINE(get_proximity)
{
	CORO_BEGIN();
	while (1)
	{
		proximity = get_average_sensor(ir, 10) % 2550;
		CORO_YIELD();
	}
	CORO_END();
}

CORO_DEFINE(get_distance)
{
	CORO_LOCAL int degrees_wheel_now;
	CORO_BEGIN();
	while (1)
	{
		_get_tacho_position(&degrees_wheel_now);
		walked_dist = (float)(degrees_wheel_now - start_degrees_wheel) * WHEEL_CONSTANT;
		CORO_YIELD();
	}
	CORO_END();
}

CORO_DEFINE(get_angle)
{
	CORO_LOCAL int temp_angle;
	CORO_BEGIN();
	for (;;)
	{
		temp_angle = get_average_sensor(gyro, 5);
		// get_sensor_value(0, gyro, &temp_angle);
		gyro_angle = -(temp_angle - gyro_zero_angle);
		// printf("Gyro angle: %d\n", gyro_angle);
		CORO_YIELD();
	}
	CORO_END();
}

CORO_DEFINE(get_touch)
{
	CORO_BEGIN();
	for (;;)
	{
		get_sensor_value(0, touch, &touch_pressed);
		CORO_YIELD();
	}
	CORO_END();
}

CORO_DEFINE(DFA)
{

	CORO_LOCAL int prev_angle, save_walk, save_walk2, save_walk3, pos, r, d, sign, max_speed, i, save_dist, prev_angle0, bumps, save_angle, f1_walk, f2_walk;

	CORO_BEGIN();

	while (1)
	{
		switch (state)
		{

		case STATE_START:
			lateral_tras = 7;
			printf("STATE START\n");
			expected_angle = 0 + lap;
			state = state_forward(proximity, walked_dist, START_DISTANCE, STATE_START, STATE_FORWARD1);
			if (state == STATE_FORWARD1)
			{
				_get_tacho_position(&start_degrees_wheel);
			}
			if (state != STATE_START)
				break;
			rel_walk = (int)(200 * DISTANCE_CONSTANT);
			command = MOVE_FORWARD;
			break;

		case STATE_GYRO_CAL_BUTTON:

			printf("STATE_GYRO_CAL_BUTTON\n");

			while (!touch_pressed)
			{
				printf("touch pressed %d\n", touch_pressed);

				rel_walk = (int)(20 * DISTANCE_CONSTANT);
				command = MOVE_FORWARD_CORRECTION;
				CORO_WAIT(command == MOVE_NONE);
			}

			lap = gyro_angle;

			_get_tacho_position(&start_degrees_wheel);

			state = STATE_FORWARD1;

			printf(" exiting STATE_GYRO_CAL_BUTTON\n");

			break;

		case STATE_FORWARD1:
			lateral_tras = 0;
			printf("STATE_FORWARD1 angle:%d\n", gyro_angle);
			expected_angle = 92 + lap;
#if LAP_POS1
			f1_walk = n_lap == 0 ? FORWARD1_DISTANCE_START : FORWARD1_DISTANCE;
#elif LAP_POS2
			f1_walk = n_lap == 0 ? FORWARD1_DISTANCE_START - 15 : FORWARD1_DISTANCE;
#endif
			state = state_forward(proximity, walked_dist, f1_walk, STATE_FORWARD1, STATE_FORWARD2);
			/* should enter here */
			if (state == STATE_FORWARD2)
			{
				_get_tacho_position(&start_degrees_wheel);
				left_right = 1;
			}
			rel_walk = (int)(200 * DISTANCE_CONSTANT);
			command = MOVE_FORWARD;
			break;

		case STATE_FORWARD2:
			lateral_tras = 0;
			printf("STATE_FORWARD2\n");
			expected_angle = 181 + lap;
#if LAP_POS3
			f2_walk = n_lap == 0 ? 60 : FORWARD2_DISTANCE;
#else
			f2_walk = FORWARD2_DISTANCE;
#endif
			state = state_forward_no_prox_check(proximity, walked_dist, f2_walk, STATE_FORWARD2, STATE_FORWARD5);
			if (state == STATE_FORWARD5)
			{
				_get_tacho_position(&start_degrees_wheel);
				left_right = 1;
				break;
			}
			rel_walk = (int)(200 * DISTANCE_CONSTANT);
			command = MOVE_FORWARD;

			break;

		case STATE_DROP_OBS:
			printf("STATE_DROP_OBS\n");
#if LAP_POS3
			if (n_lap == 1)
#else
			if (n_lap == 0)
#endif
			{

				if (ev3_search_tacho(LEGO_EV3_M_MOTOR, &obs_motor, 0))
				{

					get_tacho_max_speed(obs_motor, &max_speed);
					set_tacho_speed_sp(obs_motor, max_speed / 5);
					set_tacho_ramp_up_sp(obs_motor, 500);
					set_tacho_ramp_down_sp(obs_motor, 500);
					set_tacho_position_sp(obs_motor, -100);

					set_tacho_command_inx(obs_motor, TACHO_RUN_TO_REL_POS);

					Sleep(500);

					set_tacho_position_sp(obs_motor, 100);
					set_tacho_command_inx(obs_motor, TACHO_RUN_TO_REL_POS);
				}
				else
				{
					printf("LEGO_EV3_M_MOTOR is NOT found\n");
				}
			}
			left_right = -1;
			state = STATE_REG_LAP;
			_get_tacho_position(&start_degrees_wheel);

			break;

		case STATE_FORWARD5:
			lateral_tras = 0;

			printf("STATE_FORWARD5\n");
			expected_angle = 270 + lap;
			state = state_forward(proximity, walked_dist, FORWARD5_DISTANCE, STATE_FORWARD5, STATE_FORWARD6);
			if (state == STATE_FORWARD6)
			{
				_get_tacho_position(&start_degrees_wheel);
				left_right = 1;
				break;
			}
			rel_walk = (int)(200 * DISTANCE_CONSTANT);
			command = MOVE_FORWARD;
			break;

		case STATE_FORWARD6:
			lateral_tras = 7;

			printf("STATE_FORWARD6\n");
			expected_angle = 366 + lap;
			state = state_forward_no_prox_check(proximity, walked_dist, FORWARD6_DISTANCE, STATE_FORWARD6, STATE_DROP_OBS);
			rel_walk = (int)(400 * DISTANCE_CONSTANT);
			command = MOVE_FORWARD;
			/*
			 * This is because the motors think that they have not finished running
			 * but when checking in the state_forward_no_prox_check they have walked
			 * the right distance, and so it switches to the next state. However, the
			 * command does not switch to MOVE_NONE, so moving will always be equal to
			 * command and the drive coroutine will be stuck on the corowait moving!=command.
			 */
			if (state == STATE_DROP_OBS)
			{
				printf("Entering state forward move none\n");
				command = MOVE_NONE;
				left_right = 1;
			}
			break;

		/* Lap is finished, set the state to START*/
		case STATE_REG_LAP:
			printf("FINISHED LAP %d\n", lap / 360);
			state = STATE_GYRO_CAL_BUTTON;
			lap += 360;
			n_lap++;
			_get_tacho_position(&start_degrees_wheel);

			break;

			/*
			 *  In case the data of the IR sensor which is not comprehended in the range settled in design mode
			 *  this method is called.
			 *
			 *  If the proximity is greater to the object than expected, it means we have to get much closer,
			 *  otherwise we have to get further back. In order to do that, either we calculate a dynamic value
			 *  based on the distance max and the proximity or a we use fixed distance.
			 *
			 *  Testing it, we observed that the second case is more precise, so we assume that is the best option.
			 *  When we say that is it more precise, we mean that the is less likely for the robot to hit an object
			 *  before realizing that the proximity condition is not valid anymore.
			 *
			 *  Finally we change the state to the one setted in the previous method (which called STATE_PROXIMITY_CORRECTION).
			 *  For instance, STATE_START, state_forward, STATE_PROXIMITY_CORRECTION, STATE_FORWARD1
			 */

		case STATE_PROXIMITY_CORRECTION:
			printf("STATE: PROXIMITY CORRECTION\nActual proximity: %d\n", proximity);
			while (proximity >= DISTANCE_MAX /* || proximity <= DISTANCE_MIN*/)
			{

				while (proximity >= DISTANCE_MAX)
				{
					save_dist = proximity;
					printf("Getting closer of %d\n proximity %d, DISTANCE_MAX %d\n", (proximity - DISTANCE_MAX) / 10, proximity, DISTANCE_MAX);
					rel_walk = (int)(FORWARD_STEP * DISTANCE_CONSTANT);
					if (rel_walk == 0)
						rel_walk = 1;
					command = MOVE_FORWARD_CORRECTION;
					CORO_WAIT(command == MOVE_NONE);
					if (save_dist == proximity)
					{
						proximity = DISTANCE_MIN - 1;
						break;
					}
				}
				// proximity is updated here
				/*while (proximity <= DISTANCE_MIN)
				{
					printf("Getting further of %d", BACKWARDS_STEP);
					rel_walk = (int)((-BACKWARDS_STEP) * DISTANCE_CONSTANT);
					command = MOVE_FORWARD_CORRECTION;
					CORO_WAIT(command == MOVE_NONE);
				}*/
			}

			printf("Exiting proximity correction\n");
			_get_tacho_position(&start_degrees_wheel);
			state = next_state;

			break;

			/*
			 * This state is called whenever the robot get stack to an object or whatever item there is
			 * in the circuit. In this case, the first thing we want to do is gettin further from the obstacle with the
			 * inner while iteration. After fixing this issue, we move the state to the next one, which is
			 * the turning one.
			 */

		case STATE_PROXIMITY_CORRECTION_BEFORE_ANGLE:
			printf("STATE: PROXIMITY CORRECTION BEFORE ANGLE\nActual proximity: %d\n", proximity);
			/*	while (proximity <= DISTANCE_MIN_ANGLE_CORR)
				{
					while (proximity <= DISTANCE_MIN_ANGLE_CORR)
					{
						printf("Getting further of %d", BACKWARDS_STEP);
						rel_walk = (int)((-BACKWARDS_STEP) * DISTANCE_CONSTANT);
						command = MOVE_FORWARD_CORRECTION;
						CORO_WAIT(command == MOVE_NONE);
					}
				}*/

			printf("Exiting proximity correction\n");
			state = next_state;
			break;

			/*
			 *  In case the angle data is not comprehended in the range settled in design mode
			 *  this method is called. In fact, it is aimed to correct the angle of the movement of the robot.
			 *  Doing this, we can check wherever it is going straight or not.
			 *
			 *  First thing to be done is to save the data regarding the revolutions the motor has completed a movement.
			 *  Given this data we can lately understand how many cm did the robot do while fixing the angle.
			 *
			 *  At every while cycle, until the angle is not in the range settled in design mode, we want the robot
			 *  to rotate for a specific angle, given by (expected_angle - gyro_angle) / 2.
			 *
			 *  For instance, when we have;
			 * 		- expected_angle = 30
			 * 		- gyro_angle = 10
			 *
			 *  The robot will rotate by (30-10)/2 = 10, getting to an angle of 20.
			 *  After the first cycle we will have now this data;
			 * 		- expected_angle = 30
			 * 		- gyro_angle = 20
			 *
			 *  After the robot finishes to turn, we want it to stop moving in order to understand clearly
			 *  what kind of operation it will have to do.
			 *
			 *  Finally, when the angle of the motor is within the range, we want to set again our reference,
			 *  which is stored in the variable axle_zero_angle. Here we add the difference between the number of revolutions
			 *  done after the angle correction and before. This number is giving us the information regarding the new angle
			 *  which is detected as the 'correct' one that is within the range.
			 *
			 */

		case STATE_ANGLE_CORRECTION:
			printf("STATE: ANGLE CORRECTION\nActual angle: %d, Expected angle: %d\n", gyro_angle, expected_angle);

			_get_tacho_position(&save_walk);

			if ((expected_angle - gyro_angle) > ANGLE_OUTLIER)
				break;

			while (gyro_angle - expected_angle < -ANGLE_THRESHOLD || gyro_angle - expected_angle > ANGLE_THRESHOLD)
			{
				prev_angle0 = prev_angle;
				prev_angle = gyro_angle;
				printf("Turning this amount : %d = %d/2", (expected_angle - gyro_angle), (expected_angle - gyro_angle) / 2);
				angle = (expected_angle - gyro_angle) / 2;
				command = TURN_ANGLE;
				CORO_WAIT(command == MOVE_NONE);
				/* if angle has not changed, correct proximity */
				if (prev_angle0 == prev_angle && /*prev_angle == (gyro_angle - 1) || prev_angle == (gyro_angle + 1) ||*/ prev_angle == (gyro_angle))
				{
					printf("BUG HEREEE");
					rel_walk = (int)((-5) * DISTANCE_CONSTANT);
					command = MOVE_FORWARD_CORRECTION;
					CORO_WAIT(command == MOVE_NONE);
				}
			}
			printf("Exiting angle correction\n");
			_get_tacho_position(&save_walk2);
			start_degrees_wheel = start_degrees_wheel + (save_walk2 - save_walk);
			printf("AXLE DISTANCE2 : AXLE DISTANCE %d %g\n", start_degrees_wheel, walked_dist * DISTANCE_CONSTANT);

			state = next_state;
			break;

		case STATE_PROXIMITY_OBSTACLE:
			printf("STATE: STATE_PROXIMITY_OBSTACLE\n");
			if (bumps == 1)
			{
				rel_walk = (int)((lateral_tras)*DISTANCE_CONSTANT);
				command = MOVE_FORWARD_CORRECTION;
				CORO_WAIT(command == MOVE_NONE);
				expected_angle = save_angle;
				state = STATE_ANGLE_CORRECTION;
				bumps = 0;
				_get_tacho_position(&pos);
				start_degrees_wheel += pos - save_walk3;
				next_state = next_state1;
				left_right *= -1;
				break;
			}
			else
			{
				bumps = 0;
				_get_tacho_position(&save_walk3);

				while (proximity <= DISTANCE_OBSTACLE)
				{
					printf("Getting further from obstacle %d", BACKWARDS_STEP);
					rel_walk = (int)((-10) * DISTANCE_CONSTANT);
					command = MOVE_FORWARD_CORRECTION;
					CORO_WAIT(command == MOVE_NONE);
					bumps++;
					printf("BUMPS: %d \n", bumps);
					if (bumps == 1)
					{
						break;
					}
				}

				if (bumps == 1)
				{
					save_angle = gyro_angle;
					printf("LEFT RIGHT %d", left_right);
					expected_angle = gyro_angle + 90 * left_right;
					next_state1 = next_state;
					next_state = STATE_PROXIMITY_OBSTACLE;
					state = STATE_ANGLE_CORRECTION;
					break;
				}
			}
			state = next_state;
			break;

		case STATE_STILL:
			command = MOVE_NONE;
			break;

		default:
			break;
		}

		CORO_YIELD();
	}

	CORO_END();
}
/* Coroutine of control the motors */
CORO_DEFINE(drive)
{
	CORO_LOCAL int speed_linear, speed_circular;
	CORO_LOCAL int _wait_stopped;

	CORO_BEGIN();
	speed_linear = max_speed * SPEED_LINEAR / 100;
	speed_circular = max_speed * SPEED_CIRCULAR / 100;

	for (;;)
	{
		/* Waiting new command */

		CORO_WAIT(moving != command);
		printf("Moving %d, Command %d\n", moving, command);

		_wait_stopped = 0;
		switch (command)
		{
		case MOVE_NONE:
			_stop();
			printf("Stop\n");
			_wait_stopped = 1;
			break;

		case MOVE_FORWARD_CORRECTION:
			_run_to_rel_pos(speed_linear, rel_walk, speed_linear, rel_walk);
			_wait_stopped = 1;
			break;

		case MOVE_FORWARD:
			printf("MOVE_FORward: rel walk %d\n", rel_walk);
			_run_to_rel_pos(speed_linear, rel_walk, speed_linear, rel_walk);

			break;

		case TURN_ANGLE:
			if (angle >= 0)
			{
				_run_to_rel_pos(speed_circular, DEGREE_TO_COUNT(0), speed_circular, DEGREE_TO_COUNT(angle));
			}
			else
			{
				_run_to_rel_pos(speed_circular, DEGREE_TO_COUNT(-angle), speed_circular, DEGREE_TO_COUNT(0));
			}
			_wait_stopped = 1;
			break;

		case TURN_ANGLE_COMPASS:
			_run_to_rel_pos(speed_circular, DEGREE_TO_COUNT(-angle), speed_circular, DEGREE_TO_COUNT(angle));
			_wait_stopped = 1;
			break;

		case STEP_BACKWARD:
			_run_timed(speed_linear, speed_linear, 1000);
			_wait_stopped = 1;
			break;
		}
		moving = command;

		if (_wait_stopped)
		{
			/* Waiting the command is completed */
			CORO_WAIT(!_is_running());
			printf("is running exit\n");
			printf("Moving %d, Command %d\n\n", moving, command);

			command = moving = MOVE_NONE;
		}
	}
	CORO_END();
}

int main(void)
{
	printf("Waiting the EV3 brick online...\n");
	if (ev3_init() < 1)
		return (1);

	printf("*** ( EV3 ) Hello! ***\n");
	ev3_sensor_init();
	ev3_tacho_init();

	setbuf(stdout, 0);

	app_alive = app_init();

	while (app_alive)
	{
		CORO_CALL(get_proximity);
		CORO_CALL(get_distance);
		CORO_CALL(get_angle);
		CORO_CALL(get_touch);
		CORO_CALL(DFA);
		CORO_CALL(drive);

		// Sleep(50);
	}
	ev3_uninit();
	printf("*** ( EV3 ) Bye! ***\n");

	return (0);
}

/*
 * State forward is used in order to move forward the robot.
 * The parameters are;
 * 		- proximity: int value, it represent the distance between the IR sensor and the first obstacle in real time
 * 		- walked: float value, it represent the distance it actually walk
 * 		- walked_thresh: float value, it represent the distance we would like it to walk
 * 		- state_within_thresh: current status
 * 		- state_after_tresh: next status
 *
 * In the difference between the actual angle of the motors (gyro_angle) and the expected one (expected_angle)
 * is not between a certain range settled in design mode, it means that the robot is moving not straight, so we need to fix it.
 * Before fixing it, using STATE_PROXIMITY_CORRECTION_BEFORE_ANGLE, we set the next state for the second curve.
 *
 * Finally, in case the distance he walked is greater than the one it should walk, we stop him and if the proximity
 * from the IR sensor to the first obstacle is either too big or too low, we move the state to STATE_PROXIMITY_CORRECTION, fixing
 * the distance to the obstacle. After solving this issue, we move to the next state, which ends the FORWARD1 method.
 */

int state_forward(int proximity, float walked, float walked_thresh, int state_within_thresh, int state_after_tresh)
{
	if (gyro_angle - expected_angle < -ANGLE_THRESHOLD || gyro_angle - expected_angle > ANGLE_THRESHOLD)
	{

		command = MOVE_NONE;
		next_state = state_within_thresh;
		return STATE_ANGLE_CORRECTION;
	}

	printf("Walked %f\n", walked);
	if (walked >= walked_thresh)
	{
		command = MOVE_NONE;
		if (proximity >= DISTANCE_MAX || proximity <= DISTANCE_MIN)
		{
			next_state = state_after_tresh;
			left_right = STATE_FORWARD4 == state ? -1 : 1;
			_get_tacho_position(&start_degrees_wheel);
			return STATE_PROXIMITY_CORRECTION;
		}
		left_right = STATE_FORWARD4 == state ? -1 : 1;
		_get_tacho_position(&start_degrees_wheel);
		return state_after_tresh;
	}

	if (proximity <= DISTANCE_OBSTACLE)
	{
		next_state = state_within_thresh;
		return STATE_PROXIMITY_OBSTACLE;
	}

	return state_within_thresh;
}

/*
 * This method is called while the robot is doing the second corridor, where there are two obstacles. Since we don't know the shape of
 * these items, and knowing that if there is a circle shape the IR will retrieve wrong values, we don't want to check the proximity based on the
 * next obstacle anymore. So what we do in this method is just to correct the angle in case it is not in the fixed range and check
 * wherever the robot has finished to walk based on the walked variable.
 *
 * The parameters are;
 * 		- proximity: int value, it represent the distance between the IR sensor and the first obstacle in real time
 * 		- walked: float value, it represent the distance it actually walk
 * 		- walked_thresh: float value, it represent the distance we would like it to walk
 * 		- state_within_thresh: current status
 * 		- state_after_tresh: next status
 */

int state_forward_no_prox_check(int proximity, float walked, float walked_thresh, int state_within_thresh, int state_after_tresh)
{
	if (gyro_angle - expected_angle < -ANGLE_THRESHOLD || gyro_angle - expected_angle > ANGLE_THRESHOLD)
	{
		next_state = state_within_thresh;
		return STATE_ANGLE_CORRECTION;
	}

	if (walked >= walked_thresh)
	{
		return state_after_tresh;
	}

	if (proximity <= DISTANCE_OBSTACLE)
	{
		next_state = state_within_thresh;
		return STATE_PROXIMITY_OBSTACLE;
	}

	return state_within_thresh;
}
