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

int moving;	 /* Current moving */
int command; /* Command for the 'drive' coroutine */
int angle;	 /* Angle of rotation */

uint8_t ir, touch, compass; /* Sequence numbers of sensors */
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

	if (ev3_search_sensor(HT_NXT_COMPASS, &compass, 0))
	{
		printf(" use the COMPASS sensor.\n");
		set_sensor_command(compass, "BEGIN-CAL");
		sleep(5);
		set_sensor_command(compass, "END-CAL");
	}

	/*
	if ( ev3_search_sensor( LEGO_EV3_TOUCH, &touch, 0 )) {
		printf( " use the TOUCH sensor.\n" );
	} else {
		touch = DESC_LIMIT;
		printf( " press UP on the EV3 brick.\n" );
	}
	printf( "Press BACK on the EV3 brick for EXIT...\n" );*/
	return (1);
}
int get_average_sensor(uint8_t sensor, int samples){

		int value=0, average=0, i;

		get_sensor_value(0, sensor, &value);
			average += value;

		for (i = 0; i < samples; i++)
		{
			get_sensor_value(0, sensor, &value);
			average = (average + value)/2;
		}
		return average;
	
}
CORO_CONTEXT(handle_ir_proximity);
CORO_CONTEXT(drive);
CORO_CONTEXT(handle_compass);

/* Coroutine of IR proximity handling (self-driving),
   based on Franz Detro drive_test.cpp */
CORO_DEFINE(handle_ir_proximity)
{
	CORO_LOCAL int front, prox;

	CORO_BEGIN();
	for (;;)
	{
		/* Waiting self-driving mode */

		CORO_WAIT(mode == MODE_AUTO);
		prox = 0;
		prox=get_average_sensor(ir, 200);
		
		printf("Proximity %d\n", prox);
		if (prox == 0)
		{
			/* Oops! Stop the vehicle */
			command = MOVE_NONE;
		}
		else if (prox < 120 || prox == 2550)
		{ /* Need for detour... */
			front = prox;
			/* Look to the left */
			angle = -30;
			do
			{
				command = TURN_ANGLE;
				CORO_WAIT(command == MOVE_NONE || command == TURN_ANGLE_COMPASS);

				prox = 0;
				prox=get_average_sensor(ir, 200);
				if (prox < front)
				{
					if (angle < 0)
					{
						/* Still looking to the left - look to the right */
						angle = 60;
					}
					else
					{
						/* Step back */
						command = STEP_BACKWARD;
						CORO_WAIT(command == MOVE_NONE || TURN_ANGLE_COMPASS);
					}
				}
			} while ((prox > 0) && (prox < 120) && (mode == MODE_AUTO));
		}
		else
		{
			/* Track is clear - Go! */
			 command = MOVE_FORWARD;
		}
		CORO_YIELD();
	}
	CORO_END();
}



CORO_DEFINE(handle_compass)
{
	CORO_LOCAL int direction;

	CORO_BEGIN();
	for (;;)
	{
		/* Waiting self-driving mode */

		CORO_WAIT(mode == MODE_AUTO);
		direction = 0;

		direction=get_average_sensor(compass, 200);
		while (direction < 340)
		{
			
			printf("%d\n", direction);
			direction=get_average_sensor(compass, 200);
			
				angle=-5;
			
			command = TURN_ANGLE_COMPASS;
			CORO_WAIT(command == MOVE_NONE);
		}

		command = MOVE_FORWARD;

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
			_run_forever(-speed_linear, -speed_linear);
			break;

		case MOVE_BACKWARD:
			_run_forever(speed_linear, speed_linear);
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
	while (app_alive)
	{
	//	CORO_CALL(handle_compass);
		CORO_CALL(handle_ir_proximity);
		CORO_CALL(drive);

		Sleep(10);
	}
	ev3_uninit();
	printf("*** ( EV3 ) Bye! ***\n");

	return (0);
}
