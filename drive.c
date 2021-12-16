/*
	 ____ __     ____   ___    ____ __         (((((()
	| |_  \ \  /   ) ) | |  ) | |_  \ \  /  \(@)- /
	|_|__  \_\/  __)_) |_|_/  |_|__  \_\/   /(@)- \
											   ((())))
 */
/**
 *  \file  drive.c
 *  \brief  ev3dev-c example of using coroutines to control the motors.
 *  \author  Vitaly Kravtsov (in4lio@gmail.com)
 *  \copyright  See the LICENSE file.
 */

#include <stdio.h>
#include "coroutine.h"
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"
// WIN32 /////////////////////////////////////////
#ifdef __WIN32__

#include <windows.h>

// UNIX //////////////////////////////////////////
#else

#include <unistd.h>
#define Sleep(msec) usleep((msec)*1000)

//////////////////////////////////////////////////
#endif

#define L_MOTOR_PORT OUTPUT_C
#define L_MOTOR_EXT_PORT EXT_PORT__NONE_
#define R_MOTOR_PORT OUTPUT_B
#define R_MOTOR_EXT_PORT EXT_PORT__NONE_
#define IR_CHANNEL 0

#define SPEED_LINEAR 75	  /* Motor speed for linear motion, in percents */
#define SPEED_CIRCULAR 50 /* ... for circular motion */
#define HISTORY_LENGTH 200

#define WHEEL_DIAMETER 5.6
#define PI 3.141592
#define WHEEL_CONSTANT ((PI * WHEEL_DIAMETER) / 360)
#define DISTANCE_CONSTANT (1 / WHEEL_CONSTANT)

#define DISTANCE_MAX 200 /* 10 cm */
#define DISTANCE_MIN 170 /* 5 cm */
#define BACKWARDS_STEP 1 /* 5 cm */
#define DISTANCE_MIN_ANGLE_CORR 200

#define LS1 20
#define SS1 20
#define LS2 20

#define ANGLE_THRESHOLD 5

int max_speed; /* Motor maximal speed */

#define DEGREE_TO_COUNT(d) ((d)*260 / 90)

int app_alive;

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
	STATE_FORWARD4,
	STATE_FORWARD5,
	STATE_FORWARD6,
	STATE_PROXIMITY_CORRECTION,
	STATE_ANGLE_CORRECTION,
	STATE_REG_LAP,
	STATE_PROXIMITY_CORRECTION_BEFORE_ANGLE
};

int moving;	 /* Current moving */
int command; /* Command for the 'drive' coroutine */
int angle;	 /* Angle of rotation */
int gyro_angle, gyro_zero_angle;
int proximity;
float axle_distance;
int axle_zero_angle;
int rel_walk;
int state, next_state;
int expected_angle;
int lap;
int touch_pressed;

uint8_t ir, gyro, touch; /* Sequence numbers of sensors */
enum
{
	L,
	R
};
uint8_t motor[3] = {DESC_LIMIT, DESC_LIMIT, DESC_LIMIT}; /* Sequence numbers of motors */

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

	lap = 0;
	state = STATE_START;
	printf("Init State: %d\n", state);
	return (1);
}

int state_forward(int proximity, float walked, float walked_thresh, int state_within_thresh, int state_after_tresh);
int state_forward_no_prox_check(int proximity, float walked, float walked_thresh, int state_within_thresh, int state_after_tresh);
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
		Sleep(50);
		proximity = get_average_sensor(ir, 10) % 2550;
		CORO_YIELD();
	}
	CORO_END();
}

CORO_DEFINE(get_distance)
{
	CORO_LOCAL int pos;
	CORO_BEGIN();
	while (1)
	{

		_get_tacho_position(&pos);
		axle_distance = (float)(pos - axle_zero_angle) * WHEEL_CONSTANT;
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
		get_sensor_value(0, gyro, &temp_angle);
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
		printf("SEEE: %d\n", touch_pressed);
		CORO_YIELD();
	}
	CORO_END();
}

CORO_DEFINE(DFA)
{

	CORO_LOCAL int prev_angle, save_walk, save_walk2;

	CORO_BEGIN();

	while (1)
	{
		switch (state)
		{

		case STATE_START:
			printf("STATE START\n");
			expected_angle = 0 + lap;
			state = state_forward(proximity, axle_distance, 50, STATE_START, STATE_FORWARD1);
			if (state == STATE_FORWARD1)
			{
				_get_tacho_position(&axle_zero_angle);
			}
			if (state != STATE_START)
				break;
			rel_walk = (int)(50 * DISTANCE_CONSTANT);
			command = MOVE_FORWARD;
			break;

		case STATE_FORWARD1:
			printf("STATE_FORWARD1\n");
			expected_angle = 90 + lap;
			state = state_forward(proximity, axle_distance, 80, STATE_FORWARD1, STATE_FORWARD2);
			/* should enter here */
			if (state == STATE_FORWARD2)
				_get_tacho_position(&axle_zero_angle);
			rel_walk = (int)(80 * DISTANCE_CONSTANT);
			command = MOVE_FORWARD;
			break;

		case STATE_FORWARD2:
			printf("STATE_FORWARD2\n");
			expected_angle = 180 + lap;
			state = state_forward_no_prox_check(proximity, axle_distance, 80, STATE_FORWARD2, STATE_FORWARD3);
			if (state == STATE_FORWARD3)
			{
				_get_tacho_position(&axle_zero_angle);
			}
			rel_walk = (int)(80 * DISTANCE_CONSTANT);
			command = MOVE_FORWARD;
			break;

		case STATE_FORWARD3:
			printf("STATE_FORWARD3\n");
			expected_angle = 270 + lap;
			state = state_forward(proximity, axle_distance, 5, STATE_FORWARD3, STATE_FORWARD4);
			if (state == STATE_FORWARD4)
				_get_tacho_position(&axle_zero_angle);
			rel_walk = (int)(5 * DISTANCE_CONSTANT);
			command = MOVE_FORWARD;
			break;

		case STATE_FORWARD4:
			printf("STATE_FORWARD4\n");
			expected_angle = 180 + lap;
			state = state_forward(proximity, axle_distance, 60, STATE_FORWARD4, STATE_FORWARD5);
			if (state == STATE_FORWARD5)
			{
				_get_tacho_position(&axle_zero_angle);
			}
			rel_walk = (int)(60 * DISTANCE_CONSTANT);
			command = MOVE_FORWARD;
			break;


		case STATE_FORWARD5:
			printf("STATE_FORWARD5\n");
			expected_angle = 270 + lap;
			state = state_forward(proximity, axle_distance, 30, STATE_FORWARD5, STATE_FORWARD6);
			if (state == STATE_FORWARD6)
				_get_tacho_position(&axle_zero_angle);
			rel_walk = (int)(30 * DISTANCE_CONSTANT);
			command = MOVE_FORWARD;
			break;

		case STATE_FORWARD6:
			printf("STATE_FORWARD6\n");
			expected_angle = 360 + lap;
			state = state_forward(proximity, axle_distance, 90, STATE_FORWARD6, STATE_REG_LAP);
			rel_walk = (int)(90 * DISTANCE_CONSTANT);
			command = MOVE_FORWARD;
			break;

		case STATE_REG_LAP:
			printf("FINISHED LAP %d\n", lap / 360);
			lap += 355;
			state = STATE_START;
			_get_tacho_position(&axle_zero_angle);
			break;

		case STATE_PROXIMITY_CORRECTION:
			printf("STATE: PROXIMITY CORRECTION\nActual proximity: %d\n", proximity);
			while (proximity >= DISTANCE_MAX || proximity <= DISTANCE_MIN)
			{
				while (proximity >= DISTANCE_MAX)
				{
					// TODO: Sto assumendo l'inerzia, aggiungere un epsilon in più?
					printf("Getting closer of %d", (proximity - DISTANCE_MAX) / 10);
					rel_walk = (int)((proximity - DISTANCE_MAX) / 10 * DISTANCE_CONSTANT);
					if (rel_walk == 0)
						rel_walk = 1;
					command = MOVE_FORWARD_CORRECTION;
					CORO_WAIT(command == MOVE_NONE);
				}
				// proximity is updated here
				while (proximity <= DISTANCE_MIN)
				{
					printf("Getting further of %d", BACKWARDS_STEP);
					rel_walk = (int)((-BACKWARDS_STEP) * DISTANCE_CONSTANT);
					command = MOVE_FORWARD_CORRECTION;
					CORO_WAIT(command == MOVE_NONE);
				}
			}

			printf("Exiting proximity correction\n");
			_get_tacho_position(&axle_zero_angle);
			state = next_state;

			break;

		case STATE_PROXIMITY_CORRECTION_BEFORE_ANGLE:
			printf("STATE: PROXIMITY CORRECTION BEFORE ANGLE\nActual proximity: %d\n", proximity);
			while (proximity <= DISTANCE_MIN_ANGLE_CORR)
			{
				// proximity is updated here
				while (proximity <= DISTANCE_MIN_ANGLE_CORR)
				{
					printf("Getting further of %d", BACKWARDS_STEP);
					rel_walk = (int)((-BACKWARDS_STEP) * DISTANCE_CONSTANT);
					command = MOVE_FORWARD_CORRECTION;
					CORO_WAIT(command == MOVE_NONE);
				}
			}

			printf("Exiting proximity correction\n");
			state = next_state;
			break;

		case STATE_ANGLE_CORRECTION:
			printf("STATE: ANGLE CORRECTION\nActual angle: %d, Expected angle: %d\n", gyro_angle, expected_angle);

			_get_tacho_position(&save_walk);
			printf("AXLE DISTANCE: %d\n", axle_zero_angle);

			while (gyro_angle - expected_angle < -ANGLE_THRESHOLD || gyro_angle - expected_angle > ANGLE_THRESHOLD)
			{
				prev_angle = gyro_angle;
				printf("Turning this amount : %d = %d/2", (expected_angle - gyro_angle), (expected_angle - gyro_angle) / 2);
				angle = (expected_angle - gyro_angle) / 2;
				command = TURN_ANGLE;
				CORO_WAIT(command == MOVE_NONE);
				// TODO if angle has not changed, correct proximity
				/*	if(prev_angle == (
					.-1) || prev_angle == (gyro_angle+1) || prev_angle == (gyro_angle)){

					}*/
			}
			printf("Exiting angle correction\n");
			_get_tacho_position(&save_walk2);
			axle_zero_angle = axle_zero_angle + (save_walk2 - save_walk);
			printf("AXLE DISTANCE2 : AXLE DISTANCE %d %g\n", axle_zero_angle, axle_distance * DISTANCE_CONSTANT);

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
		printf("Moving %d, Command %d\n", moving, command);
		/* Waiting new command */
		CORO_WAIT(moving != command);

		_wait_stopped = 0;
		switch (command)
		{

		case MOVE_NONE:
			_stop();
			printf("Stop\n");
			_wait_stopped = 1;
			break;

		case MOVE_FORWARD_CORRECTION:
			printf("State while correcting prox %d\n", state);
			_run_to_rel_pos(speed_linear, rel_walk, speed_linear, rel_walk);
			_wait_stopped = 1;
			break;

		case MOVE_FORWARD:
			_run_to_rel_pos(speed_linear, rel_walk, speed_linear, rel_walk);

			break;

		case MOVE_BACKWARD:
			_run_forever(-speed_linear, -speed_linear);
			break;

		case TURN_LEFT:
			_run_forever(speed_circular, -speed_circular);
			break;

		case TURN_RIGHT:
			_run_forever(-speed_circular, speed_circular);
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
	/*
		int i;
	  uint8_t sn;
	  FLAGS_T state;
	  uint8_t sn_touch;
	  uint8_t sn_color;
	  uint8_t sn_compass;
	  uint8_t sn_sonar;
	  uint8_t sn_mag;
	  char s[ 256 ];
	  int val;
	  float value;
	  uint32_t n, ii;


		 printf( "Found sensors:\n" );
	  for ( i = 0; i < DESC_LIMIT; i++ ) {
		if ( ev3_sensor[ i ].type_inx != SENSOR_TYPE__NONE_ ) {
		  printf( "  type = %s\n", ev3_sensor_type( ev3_sensor[ i ].type_inx ));
		  printf( "  port = %s\n", ev3_sensor_port_name( i, s ));
		  if ( get_sensor_mode( i, s, sizeof( s ))) {
			printf( "  mode = %s\n", s );
		  }
		  if ( get_sensor_num_values( i, &n )) {
			for ( ii = 0; ii < n; ii++ ) {
			  if ( get_sensor_value( ii, i, &val )) {
				printf( "  value%d = %d\n", ii, val );
			  }
			}
		  }
		}
	  }

	*/
	setbuf(stdout, 0);
	app_alive = app_init();
	_get_tacho_position(&axle_zero_angle);		 /* Reset the travelled distance to 0 */
	get_sensor_value(0, gyro, &gyro_zero_angle); /* Reset the starting angle to 0 */
	while (app_alive)
	{
		CORO_CALL(get_proximity);
		CORO_CALL(get_distance);
		CORO_CALL(get_angle);
		CORO_CALL(DFA);
		CORO_CALL(drive);

		Sleep(50);
	}
	ev3_uninit();
	printf("*** ( EV3 ) Bye! ***\n");

	return (0);
}

int state_forward(int proximity, float walked, float walked_thresh, int state_within_thresh, int state_after_tresh)
{

	/*	if (walked >= walked_thresh)
		{
			command = MOVE_NONE;
			if (proximity >= DISTANCE_MAX || proximity <= DISTANCE_MIN)
			{
				next_state = state_after_tresh;
				return STATE_PROXIMITY_CORRECTION;
			}
			return state_after_tresh;
		}*/

	if (gyro_angle - expected_angle < -ANGLE_THRESHOLD || gyro_angle - expected_angle > ANGLE_THRESHOLD)
	{

		command = MOVE_NONE;
		printf("Proximity before angle correction %d\n", proximity);
		if (proximity <= DISTANCE_MIN_ANGLE_CORR)
		{
			next_state = state_within_thresh;
			return STATE_PROXIMITY_CORRECTION_BEFORE_ANGLE;
		}

		next_state = state_within_thresh;
		return STATE_ANGLE_CORRECTION;
	}

	if (walked >= walked_thresh)
	{
		command = MOVE_NONE;
		if (proximity >= DISTANCE_MAX || proximity <= DISTANCE_MIN)
		{
			next_state = state_after_tresh;
			return STATE_PROXIMITY_CORRECTION;
		}
		return state_after_tresh;
	}

	return state_within_thresh;
}

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

	return state_within_thresh;
}