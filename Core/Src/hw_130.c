#include "hw_130.h"

#include <stdlib.h>

#define SR_IDX_DATA 0
#define SR_IDX_CLOCK 1
#define SR_IDX_LATCH 2


#define DIRECT_GPIO_SET(port, pin)  	( (port)->BSRR = (pin) )
#define DIRECT_GPIO_RESET(port, pin)  	( (port)->BSRR = (uint32_t)(pin) << 16U)



#define IS_VALID_ROTATION(x) ((x) > rotation_enum_min && (x) < rotation_enum_max)


#define IS_PIN_OUTPUT(GPIOx, pin)  (((GPIOx->MODER >> (2 * (__builtin_ctz(pin)))) & 0x3) == MODE_OUTPUT)




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
typedef struct hw_130_driver{
	int initialized[4];
	bool invert[4];
	motor_state state[4];
	motor_pwm pwm_timer[4];
	int sr_mask_positive[4];
	int sr_mask_negative[4];
	int sr_pin[3];
	GPIO_TypeDef *sr_port[3];
}hw_130_driver;


/*############################
 * Variables and Definitions:
 #############################
 */

//	This hosts pointer to all the Motor Controller that gets registered.
static volatile hw_130_driver *driver_registered[MAX_NUMBER_OF_HW_130_DRIVERS];

//	Counts the number of Motion Controller that got registered.
static volatile int driver_registered_number = 0;




/*############################
 * Static functions declarations:
 #############################
 */

__attribute__((always_inline)) static inline void shift_out( volatile hw_130_driver *motor_driver, uint8_t data);
__attribute__((always_inline)) static inline void set_duty_cycle(motor_pwm pwm_interface, float duty_cycle);

__attribute__((always_inline)) static inline ErrorStatus check_driver_validity(volatile hw_130_driver *motor_driver);





/*############################
 * Public functions Definitions:
 #############################
 */


/**
 * @brief 	Set new speed and rotation direction for a specified motor.
 *
 * @param 	motor_driver -> pointer to driver 'object' in memory, obtained from create_hw_130
 * @param 	n_motor		 -> motor logical index (0-3)
 * @param 	rotation 	 ->	direction of rotatio or stop state
 * @param 	speed 		 -> Electrical PWM fed to the motor.
 *
 * @return		-> ERROR if could not set motor (wrong driver object, wront direction or wrong motor number)
 * 				-> SUCCESS if motor correctly set.
 */
ErrorStatus motor_set( 	volatile hw_130_driver *motor_driver,
						int n_motor,
						motor_rotation rotation,
						float e_pwm)
{
	//		Checks over the passed parameters.

	if (check_driver_validity(motor_driver) == ERROR){
		return ERROR;
	}
	if(n_motor > 4 || n_motor < 0){
		return ERROR;
	}
	if( !IS_VALID_ROTATION(rotation)){
		return ERROR;
	}


	//	Actual update of the morot obect's state:

	motor_driver->state[n_motor].rotation = rotation;
	motor_driver->state[n_motor].speed = e_pwm;

	return SUCCESS;
}




/**
 * @brief 	Will update the driver state with the current requested state.
 *
 * @param 	motor_driver -> pointer to driver 'object' in memory, obtained from create_hw_130
 *
 * @return		-> ERROR if update successful
 * 				-> SUCCESS if motor_driver not recognised.
 */
ErrorStatus motor_update( volatile hw_130_driver *motor_driver){

	//	Checking if the motor_driver is a valid driver:
	if(check_driver_validity(motor_driver) == ERROR){
		return ERROR;
	}

	uint8_t control_byte = 0x00;

	//	Preparing the control byte and setting correct duty cycle in the state
	for(int i = 0; i<4; i++){

		switch(motor_driver->state[i].rotation){
		case stop:
			//Set control bits at 0 - not really needed as it's initialised at 0!
			motor_driver->state[i].speed = 0;  	// for free wheeling
			motor_driver->state[i].speed = 0;  	// for free wheeling
			break;
		case hard_stop:
			//Set control bits at 0 - not really needed as it's initialised at 0!
			motor_driver->state[i].speed = 100;		// for Hard braking.
			motor_driver->state[i].speed = 100;		// for Hard braking.
			break;
		case forwards:
			if(motor_driver->invert[i]){
				control_byte = control_byte | (1 << motor_driver->sr_mask_negative[i]);
			}else{
				control_byte = control_byte | (1 << motor_driver->sr_mask_positive[i]);
			}
			break;
		case backwards:
			if(motor_driver->invert[i]){
				control_byte = control_byte | (1 << motor_driver->sr_mask_positive[i]);
			}else{
				control_byte = control_byte | (1 << motor_driver->sr_mask_negative[i]);

			}
			break;
		default:
			// Should I do anything here? probably not.
			break;
		}
	}

	//	Pushing out PWM setting and control byte to shift register.
	shift_out( motor_driver, control_byte);

	//	using direct calling instead of for cycle spares 98 clock cycles,
	// 	or 7.3 % over the motor_update fn...
	set_duty_cycle((motor_driver->pwm_timer[0]), motor_driver->state[0].speed);
	set_duty_cycle((motor_driver->pwm_timer[1]), motor_driver->state[1].speed);
	set_duty_cycle((motor_driver->pwm_timer[2]), motor_driver->state[2].speed);
	set_duty_cycle((motor_driver->pwm_timer[3]), motor_driver->state[3].speed);

	return SUCCESS;
}


/**
 * @brief 	Setup in memory one of the motor.
 * 			Only checks if the motor number is in range of the define max number.
 *
 * @param motor_driver	-> pointer to driver 'object' in memory, obtained from create_hw_130
 * @param n_motor 		-> motor index in the data array.
 * @param *timer 		-> handler of the timer associated to the given motor
 * @param channel 		-> number of the timer's pwm channel.
 * @pin_positive 		-> index in the Shift Register of the driver's positive pin
 * @pin_negative 		-> index in the Shift Register of the driver's negative pin
 *
 * @returns -> ERROR or SUCCESS.
 */
ErrorStatus motor_initialize(	volatile hw_130_driver *motor_driver,
								int n_motor,
								TIM_HandleTypeDef *timer,
								int channel,
								int pin_positive,
								int pin_negative,
								bool invert_default_direction)
{
	// Checking validity of the number of motor proposed
	if(n_motor > 4 || n_motor < 0){
		return ERROR;
	}

	//	Checking if the motor_driver is a valid driver:
	if(check_driver_validity(motor_driver) == ERROR){
		return ERROR;
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

	return SUCCESS;
}



/**
 * @brief	Setupt the initial memory for hw 130 driver,
 * 			setup of Shift Register, initialize PWM outputs to zero.
 *
 * @param driver_pt			-> pointer to the pointer to containg the newly created driver.
 * @param sr_data_pin 		-> pin number of the shift register's data pin.
 * @param sr_clock_pin 		-> pin number of the shift register's clock pin.
 * @param sr_latch_pin 		-> pin number of the shift register's latch pin.
 * @param data_gpio_port 	-> PORT (A, B, ...) of the shift register's data pin.
 * @param clock_gpio_port 	-> PORT (A, B, ...) of the shift register's clock pin.
 * @param latch_gpio_port 	-> PORT (A, B, ...) of the shift register's latch pin.
 *
 * @returns -> 	ERROR or SUCCESS.
 */
ErrorStatus create_hw_130(	volatile hw_130_driver **driver_pt,
							int sr_data_pin, GPIO_TypeDef* data_gpio_port,
							int	sr_clock_pin, GPIO_TypeDef* clock_gpio_port,
							int sr_latch_pin, GPIO_TypeDef* latch_gpio_port)
{
	//	Verifying we've not exceeded the allowed number of drivers:
	if( driver_registered_number > MAX_NUMBER_OF_HW_130_DRIVERS){
		return ERROR;
	}

	//	checking the GPIO given are actual output port:
	if( 	!(	IS_PIN_OUTPUT(data_gpio_port, sr_data_pin)	 &&
				IS_PIN_OUTPUT(clock_gpio_port, sr_clock_pin) &&
				IS_PIN_OUTPUT(latch_gpio_port, sr_latch_pin))
			){
		//	One of the SR pin has not been setup as output!
		return ERROR;
	}


	// Allocating and verifying memory:
	hw_130_driver *new_hw_130 = calloc(1, sizeof(hw_130_driver));
	if(new_hw_130 == NULL){
		return ERROR;
	}


	// If memory pointer is valid, store Shift Register parameters:
	new_hw_130->sr_pin[SR_IDX_DATA] = sr_data_pin;
	new_hw_130->sr_pin[SR_IDX_CLOCK] = sr_clock_pin;
	new_hw_130->sr_pin[SR_IDX_LATCH] = sr_latch_pin;

	new_hw_130->sr_port[SR_IDX_DATA] = data_gpio_port;
	new_hw_130->sr_port[SR_IDX_CLOCK] = clock_gpio_port;
	new_hw_130->sr_port[SR_IDX_LATCH] = latch_gpio_port;


	//	Registering statically the driver, for future validity checks.
	driver_registered[driver_registered_number] = new_hw_130;
	driver_registered_number ++;

	// Setting the newly created driver:
	*driver_pt = new_hw_130;


	return SUCCESS;

}


/**
 * @brief	Setupt the initial state of Shift Register and
 * 			fires up the PWM outputs.
 *
 * @param 	motor_driver -> pointer to driver 'object' in memory, obtained from create_hw_130
 *
 * @returns -> ERROR or SUCCESS
 */
ErrorStatus start_hw_130(volatile hw_130_driver *motor_driver)
{
	//	Checking for initialization:
	for( int i = 0 ; i< 4; i++){
		if(motor_driver->initialized[i] == 0) {
			// Motor not yet initialized, return error code:
			return ERROR;
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

	return SUCCESS;
}




/*############################
 * Static functions Definitions:
 #############################
 */

__attribute__((always_inline)) static inline void shift_out( volatile hw_130_driver *motor_driver, uint8_t data) {

	//	Copying GPIO data to local variable for increased speed
	GPIO_TypeDef *d_port = motor_driver->sr_port[SR_IDX_DATA];
	GPIO_TypeDef *c_port = motor_driver->sr_port[SR_IDX_CLOCK];
	GPIO_TypeDef *l_port = motor_driver->sr_port[SR_IDX_LATCH];

	int d_pin = motor_driver->sr_pin[SR_IDX_DATA];
	int c_pin = motor_driver->sr_pin[SR_IDX_CLOCK];
	int l_pin = motor_driver->sr_pin[SR_IDX_LATCH];


	//	Disabling SR latch:
	DIRECT_GPIO_RESET(l_port, l_pin);


    for (int i = 7; i >= 0; i--) {

    	// clock low:
    	DIRECT_GPIO_RESET(c_port, c_pin);

    	//	Data bit out:
        if(data & (1 << i)){
        	DIRECT_GPIO_SET(d_port, d_pin);
        }else{
        	DIRECT_GPIO_RESET(d_port, d_pin);
        }

        //	80 MHz, looking for > 30 nS of extra delay between
        // 	data settled and clock out: > 2 cycle:
        __NOP(); __NOP(); __NOP();

        // Clock High
    	DIRECT_GPIO_SET(c_port, c_pin);
    }
    // Reset clock and data to low:
	DIRECT_GPIO_RESET(c_port, c_pin);
	DIRECT_GPIO_RESET(d_port, d_pin);

    // Latch the data
	DIRECT_GPIO_SET(l_port, l_pin);
}



__attribute__((always_inline)) static inline void set_duty_cycle(motor_pwm pwm_interface, float duty_cycle) {

	// for readability we save in a nicer form the pwm data:
	TIM_HandleTypeDef* const htim = pwm_interface.htim;
	uint32_t channel = pwm_interface.channel;

	if (duty_cycle > 100) duty_cycle = 100;
	if (duty_cycle < 0) duty_cycle = 0;

	float pw_resolution = (((float)(*htim).Init.Period + 1.0f) / 100.0f);

	uint16_t pw_desired = pw_resolution * duty_cycle;
	__HAL_TIM_SET_COMPARE(htim, channel, pw_desired);

}



__attribute__((always_inline)) static inline ErrorStatus check_driver_validity(volatile hw_130_driver *motor_driver){

	for( int i = 0; i< driver_registered_number; i++){
		if(motor_driver == driver_registered[i]){
			return SUCCESS;
		}
	}

	return ERROR;

}
