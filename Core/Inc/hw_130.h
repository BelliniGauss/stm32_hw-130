#ifndef HW_130

#define HW_130

#include <stdbool.h>
#include "stm32f4xx_hal.h"



#define MAX_NUMBER_OF_HW_130_DRIVERS 5



/*############################
 * Types and enum:
 #############################
 */

typedef enum motor_rotation{
	rotation_enum_min = -1,
	stop = 0,
	hard_stop,
	forwards,
	backwards,
	rotation_enum_max
}motor_rotation;


// forward declaration for main state struct:
typedef struct hw_130_driver hw_130_driver;



/**
 * Example of correct order for driver creation and initialization:
 *
 * create_hw_130(...)
 * motor_initialize(...)
 * motor_initialize(...)
 * motor_initialize(...)
 * motor_initialize(...)
 *
 *
 * After that, before actually using it, it should be started with:
 *
 * start_hw_130(...)
 *
 */



/*############################
 * Public functions declaration:
 #############################
 */


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
							int sr_latch_pin, GPIO_TypeDef* latch_gpio_port);


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
								bool invert_default_direction);


/**
 * @brief	Setupt the initial state of Shift Register and
 * 			fires up the PWM outputs.
 *
 * @param 	motor_driver -> pointer to driver 'object' in memory, obtained from create_hw_130
 *
 * @returns -> ERROR or SUCCESS
 */
ErrorStatus start_hw_130(volatile hw_130_driver *motor_driver);


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
ErrorStatus motor_set(	volatile hw_130_driver *motor_driver,
						int n_motor,
						motor_rotation rotation,
						float e_pwm);


/**
 * @brief 	Will update the driver state with the current requested state.
 *
 * @param 	motor_driver -> pointer to driver 'object' in memory, obtained from create_hw_130
 *
 * @return		-> ERROR if update successful
 * 				-> SUCCESS if motor_driver not recognised.
 */
ErrorStatus motor_update( volatile hw_130_driver *motor_driver);




#endif
