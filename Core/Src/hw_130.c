#include "hw_130.h"

#include <stdlib.h>


#define SR_IDX_DATA 0
#define SR_IDX_CLOCK 1
#define SR_IDX_LATCH 2



/*############################
 * Types and enum:
 #############################
 */

typedef struct motor_state{
	float speed;
	motor_rotation rotation;
}motor_state;

typedef struct motor_pwm{
	TIM_HandleTypeDef* htim;
	uint32_t channel;
}motor_pwm;

// Created as typedef struct for code clarity.
typedef struct motors{
	int initialized[4];
	bool invert[4];
	motor_state state[4];
	motor_pwm pwm_timer[4];
	int sr_mask_positive[4];
	int sr_mask_negative[4];
	int sr_pin[3];
	GPIO_TypeDef *sr_port[3];
}motors;

typedef motors *hw_130_driver;

/*############################
 * Static functions declarations:
 #############################
 */

static void shift_out(hw_130_driver driver, uint8_t data);
static void set_duty_cycle(motor_pwm pwm_interface, float duty_cycle);





/*############################
 * Public functions Definitions:
 #############################
 */



void motor_set(	hw_130_driver motor_driver,
				int n_motor,
				motor_rotation rotation,
				float speed)
{
	if(n_motor > 4 || n_motor < 0){
			return;
	}

	motor_driver->state[n_motor].rotation = rotation;
	motor_driver->state[n_motor].speed = speed;
}


/**
 * 139.25 uS max @ 16 MHz -> ~2228 clock cycle. duty
 */
void update_motors(hw_130_driver motor_driver){

	uint8_t control_byte = 0x00;

	//	Preparing the control byte and setting correct duty cycle in the state
	for(int i = 0; i<4; i++){
		float duty_cycle = motor_driver->state[i].speed;
		switch(motor_driver->state[i].rotation){
		case stop:
			//Set control bits at 0 - not really needed as it's initialised at 0!
			duty_cycle = 0;		// for free wheeling
			break;
		case hard_stop:
			//Set control bits at 0 - not really needed as it's initialised at 0!
			duty_cycle = 100;	// for Hard braking.
			break;
		case forwards:
			if(motor_driver->invert[i]){
				control_byte = control_byte | (1 << motor_driver->sr_mask_negative[i]);
			}else{
				control_byte = control_byte | (1 << motor_driver->sr_mask_positive[i]);
			}
			// leave duty cycle unchanged.
			break;
		case backwards:
			if(motor_driver->invert[i]){
				control_byte = control_byte | (1 << motor_driver->sr_mask_positive[i]);
			}else{
				control_byte = control_byte | (1 << motor_driver->sr_mask_negative[i]);

			}
			// leave duty cycle unchanged.
			break;
		default:
			// Should I do anything here? probably not.
			break;
		}
		motor_driver->state[i].speed = duty_cycle;
	}

	//	Pushing out PWM setting and control byte to shift register.
	shift_out( motor_driver, control_byte);
	for(int i = 0; i<4; i++){
		set_duty_cycle((motor_driver->pwm_timer[i]), motor_driver->state[i].speed);
	}
}



int motor_initialize(	hw_130_driver motor_driver,
						int n_motor,
						TIM_HandleTypeDef *timer,
						int channel,
						int pin_positive,
						int pin_negative,
						bool invert_default_direction)
{
	// Checking validity of the number of motor proposed
	if(n_motor > 4 || n_motor < 0){
		return -1;
	}

	// Storing info in the motor state variable:
	motor_driver->pwm_timer[n_motor].htim = timer;
	motor_driver->pwm_timer[n_motor].channel = channel;
	motor_driver->invert[n_motor] = invert_default_direction;
	motor_driver->state[n_motor].rotation = hard_stop;
	motor_driver->state[n_motor].speed = 0;
	motor_driver->sr_mask_positive[n_motor] = pin_positive;
	motor_driver->sr_mask_negative[n_motor] = pin_negative;


	// Flagging the motor as initialized.
	motor_driver->initialized[n_motor] = 1;
	return 0;
}


hw_130_driver create_hw_130(	int sr_data_pin, GPIO_TypeDef* data_gpio_port,
								int	sr_clock_pin, GPIO_TypeDef* clock_gpio_port,
								int sr_latch_pin, GPIO_TypeDef* latch_gpio_port)
{
	motors *motor_unit;


	// Allocating and verifying memory:
	motor_unit = calloc(1, sizeof(motors));
	if(motor_unit == NULL){
		return NULL;
	}


	// If memory pointer is valid, store Shift Register parameters:
	motor_unit->sr_pin[SR_IDX_DATA] = sr_data_pin;
	motor_unit->sr_pin[SR_IDX_CLOCK] = sr_clock_pin;
	motor_unit->sr_pin[SR_IDX_LATCH] = sr_latch_pin;

	motor_unit->sr_port[SR_IDX_DATA] = data_gpio_port;
	motor_unit->sr_port[SR_IDX_CLOCK] = clock_gpio_port;
	motor_unit->sr_port[SR_IDX_LATCH] = latch_gpio_port;


	return motor_unit;
}



int start_hw_130(hw_130_driver motor_driver)
{
	//	Checking for initialization:
	for( int i = 0 ; i< 4; i++){
		if(motor_driver->initialized[i] == 0) {
			// Motor not yet initialized, return error code:
			return -1;
		}
	}

	// All motors initialized
	// shifting out default value to SR:
	shift_out(motor_driver, 0x00);

	// Starting timer's pwm:
	for(int i = 0; i < 4; i++){
		HAL_TIM_PWM_Start(	motor_driver->pwm_timer[i].htim,
							motor_driver->pwm_timer[i].channel);
		set_duty_cycle( motor_driver->pwm_timer[i],0);
	}

	return 0;
}




/*############################
 * Static functions Definitions:
 #############################
 */


static void shift_out(hw_130_driver driver, uint8_t data) {

	HAL_GPIO_WritePin(driver->sr_port[SR_IDX_LATCH], driver->sr_pin[SR_IDX_LATCH], GPIO_PIN_RESET);
	__NOP();
    for (int i = 7; i >= 0; i--) {
        HAL_GPIO_WritePin(driver->sr_port[SR_IDX_DATA], driver->sr_pin[SR_IDX_DATA], (data & (1 << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        __NOP();
        HAL_GPIO_WritePin(driver->sr_port[SR_IDX_CLOCK], driver->sr_pin[SR_IDX_CLOCK], GPIO_PIN_SET);
        __NOP();
        __NOP();
        HAL_GPIO_WritePin(driver->sr_port[SR_IDX_CLOCK], driver->sr_pin[SR_IDX_CLOCK], GPIO_PIN_RESET);
        __NOP();
    }
    HAL_GPIO_WritePin(driver->sr_port[SR_IDX_DATA], driver->sr_pin[SR_IDX_DATA], GPIO_PIN_RESET);
    __NOP();
    __NOP();
    // Latch the data
    HAL_GPIO_WritePin(driver->sr_port[SR_IDX_LATCH], driver->sr_pin[SR_IDX_LATCH], GPIO_PIN_SET);
    //HAL_GPIO_WritePin(SHIFT_GPIO, SR_LATCH_PIN, GPIO_PIN_RESET);
}


static void set_duty_cycle(motor_pwm pwm_interface, float duty_cycle) {

	TIM_HandleTypeDef* const htim = pwm_interface.htim;
	uint32_t channel = pwm_interface.channel;
	if (duty_cycle > 100) duty_cycle = 100;
	if (duty_cycle < 0) duty_cycle = 0;

	float pw_resolution = (((float)(*htim).Init.Period + 1.0f) / 100.0f);

	uint16_t pw_desired = pw_resolution * duty_cycle;
	__HAL_TIM_SET_COMPARE(htim, channel, pw_desired);
}
