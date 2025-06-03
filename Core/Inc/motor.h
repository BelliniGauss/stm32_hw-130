#ifndef MOTOR_MANAGER
#define MOTOR_MANAGER

#include "hw_130.h"


/*	These two constants defint the electrical PWM duty cycle
 * 	that gets mapped to 0% mechanical and 100% mechanical.
 *
 *	The Min is useful if the engine cannot rotate at low pwm values
 *	Allows to offset it such that the 0% corresponds to electrical stall.
 */

#define PWM_ELECTRIC_MIN	 30

/*	Max is used to reduce the V max in case the power supply is higher
 * 	than the motor's max rating.
 */
#define PWM_ELECTRIC_MAX 	100




/*############################
 * Types and enum:
 #############################
 */


/*	Base type of the Motor Controller for the function's usage.
 * 	The library's fucntions operate on objects of this type.
 *
 * 	Detail:
 * 	Forward declaration of state-like struct for Motor Controller.
 * 	This contains private data not to be made directly available
 * 	nor modified from the user of this library, but only by the
 * 	library's functions.
*/
typedef struct motorManager_struct motorManager_struct;




/*############################
 * Public functions declaration:
 #############################
 */

/**
 * @brief 	This function initializa a Motion Controller. To be called before any other fn.
 *
 * @param 	motorManager_pt -> pointer to the pointer that will host the state-like struct.
 * @param 	hw_130 			-> actual driver for the motors, to be used by the Motor Controller.
 * @param 	frequency 		-> call frequency of the low level control. Suggested max (@80 MHz) 2500 Hz
 * @param 	*htim 			-> handler for the timer managing the low level control. Cannot be shared with other.
 *
 * @return  ERROR or SUCCESS
 */
ErrorStatus start_motion_control(	volatile motorManager_struct **motorManager_pt,
									volatile hw_130_driver *hw_130,
									int frequency,
									TIM_HandleTypeDef *htim);



/**
 * @brief 	This function sets a new speed request for the Motor Controller. Acts on all 4 motors.
 *
 * @param   *motion_controller 	-> pointer to the state-like struct of the Motor Controller, as returned by the start function.
 * @param	m_1 ... m_4 		-> spee setpoint for the 4 motors, in the interval (-100;+100).
 * @param 	acc 				-> Max acceleration for the ramp. Expressed in % points per seconds. Ex: 50 -> max 50%/sec.
 *
 * @return 		-> ERROR (1) if error in poarameter or execution.
 * 				-> SUCCESS (0) 	if the update was successful.
 */
ErrorStatus set_target_speed_all(	volatile motorManager_struct *motion_controller,
									float m_1,
									float m_2,
									float m_3,
									float m_4,
									float acc);

// Todo

//	void set_target_speed(motorManager_struct *motion_controller,int motor_idx, float speed, float acc);
//	void get_speed(motorManager_struct *motion_controller, float *speed_destination_buf[]);

//	maybe some pause fn?
//	Destructor ?




#endif
