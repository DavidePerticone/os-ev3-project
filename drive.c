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
	STATE_TURN_L1,
	STATE_FL1,
	STATE_STILL,

};

int moving;	 /* Current moving */
int command; /* Command for the 'drive' coroutine */
int angle;	 /* Angle of rotation */
int gyro_angle, gyro_zero_angle;
int proximity;
float axle_distance, axle_distance_zero;
int rel_walk;
int state;

uint8_t ir, gyro; /* Sequence numbers of sensors */
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

	state = STATE_START;
	printf("Init State: %d\n", state);
	return (1);
}

int state_start(int proximity, float walked, float walked_thresh);

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
CORO_CONTEXT(DFA);

CORO_DEFINE(get_proximity)
{
	CORO_BEGIN();
	while (1)
	{
		proximity = get_average_sensor(ir, 1);
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
		axle_distance = (float)(pos - axle_distance_zero) * WHEEL_CONSTANT;
		CORO_YIELD();
	}
	CORO_END();
}

CORO_DEFINE(get_angle) {
	CORO_LOCAL int temp_angle;
	CORO_BEGIN();
	for (;;) {
		get_sensor_value(0, gyro, &temp_angle);
		gyro_angle = -(temp_angle - gyro_zero_angle);
		printf("Gyro angle: %d\n", gyro_angle);
		CORO_YIELD();
	}
	CORO_END();
}

CORO_DEFINE(DFA)
{

	CORO_BEGIN();

	while (1)
	{
		switch (state)
		{

		case STATE_START:

			state = state_start(proximity, axle_distance, 25);
			rel_walk = (int)(25 * DISTANCE_CONSTANT);
			command = MOVE_FORWARD;

			break;

		case STATE_TURN_L1:
			angle = 60;
			command = TURN_ANGLE;
			CORO_WAIT(command == MOVE_NONE);
			state = STATE_FL1;
			break;

		case STATE_FL1:

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

		_wait_stopped = 0;
		switch (command)
		{

		case MOVE_NONE:
			_stop();
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
			_run_to_rel_pos(speed_circular, DEGREE_TO_COUNT(-angle), speed_circular, DEGREE_TO_COUNT(angle));
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

	app_alive = app_init();
	_get_tacho_position(&axle_distance_zero); /* Reset the travelled distance to 0 */
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

int state_start(int proximity, float walked, float walked_thresh)
{

	if (walked >= walked_thresh)
	{
		return STATE_TURN_L1;
	}

	return STATE_START;
}
