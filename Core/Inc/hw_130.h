#ifndef HW_130

#define HW_130

#include <stdbool.h>
#include "stm32f4xx_hal.h"

/*############################
 * Types and enum:
 #############################
 */

typedef enum motor_rotation{
	stop,
	hard_stop,
	forwards,
	backwards
}motor_rotation;


// forward declaration for main state struct:
typedef struct motors motors;

// type to be used in the calling t.u.:
typedef motors *hw_130_driver;



/*############################
 * Public functions declaration:
 #############################
 */


/**
 * @brief	Setupt the initial memory for hw 130 driver,
 * 			setup of Shift Register, initialize PWM outputs to zero.
 *
 * @param sr_data_pin 		-> pin number of the shift register's data pin.
 * @param sr_clock_pin 		-> pin number of the shift register's clock pin.
 * @param sr_latch_pin 		-> pin number of the shift register's latch pin.
 * @param data_gpio_port 	-> PORT (A, B, ...) of the shift register's data pin.
 * @param clock_gpio_port 	-> PORT (A, B, ...) of the shift register's clock pin.
 * @param latch_gpio_port 	-> PORT (A, B, ...) of the shift register's latch pin.
 *
 * @returns -> pointer to the motor struct.
 * 				NULL if an error occurred.
 */
hw_130_driver create_hw_130(	int sr_data_pin, GPIO_TypeDef* data_gpio_port,
								int	sr_clock_pin, GPIO_TypeDef* clock_gpio_port,
								int sr_latch_pin, GPIO_TypeDef* latch_gpio_port);


/**
 * @brief 	Setup in memory one of the motor.
 * 			Only checks if the motor number is in range of the define max number.
 *
 * @param motor_driver	-> driver 'object' in memory, obtained from create_hw_130
 * @param n_motor 		-> motor index in the data array.
 * @param *timer 		-> handler of the timer associated to the given motor
 * @param channel 		-> number of the timer's pwm channel.
 * @pin_positive 		-> index in the Shift Register of the driver's positive pin
 * @pin_negative 		-> index in the Shift Register of the driver's negative pin
 *
 * @returns -> 0 if the motor index was ok, -1 if it was out of range.
 */
int motor_initialize(	hw_130_driver motor_driver,
						int n_motor,
						TIM_HandleTypeDef *timer,
						int channel,
						int pin_positive,
						int pin_negative,
						bool invert_default_direction);


/**
 * @brief	Setupt the initial state of Shift Register and
 * 			fires up the PWM outputs.
 *
 * @returns -> 0 if everything's ok. -1 if missing info on motors and start aborted.
 */
int start_hw_130(hw_130_driver motor_driver);


/**
 * @brief 	Set new speed and rotation direction for a specified motor.
 */
void motor_set(	hw_130_driver motor_driver,
				int n_motor,
				motor_rotation rotation,
				float speed);


/**
 * @brief 	Will update the driver state with the current requested state.
 *
 * @ param 	motor_driver -> driver 'object' in memory, obtained from create_hw_130
 */
void motor_update(hw_130_driver motor_driver);




#endif
