#include "motor.h"
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <timer_bus_mapping.h>

#ifndef MAX_NUMBER_OF_DRIVERS
	#define MAX_NUMBER_OF_DRIVERS	5
#endif


/*############################
 * Types and Enums:
 #############################
 */

/**
 *	state of a single motor managed by the controller.
 */
typedef struct motionState_t{
	float current_speed;			/**< latest speed set by the manager / drivre */
	float desired_speed;			/**< target sepeed set by the user of the Motor Controller. */
	float speed_increments;			/**< increase per driver update cycle, as calculated at the desired speed setup.*/
}motionState_t;

/**
 * Object-lilke struct, opaque to the end user.
 * Host all the information about one Motor Controller.
 */
typedef struct motorController_struct{
	volatile hw_130_driver *driver;					/**< HW 130 driver oject, to be called for executing motor output. */
	motionState_t state[4];					/**< Vector containing the 4 motor's motion state structs.*/
	TIM_HandleTypeDef *interrupt_timer;		/**< pointer to the timer calling the update of the Motor Controller.*/
	float time_increments;					/**< Precalculated time interval between two MC updates.*/
}motorController_struct;






/*############################
 * Variables and Definitions:
 #############################
 */


#define PRESCALER_MAX 	0xFFFF			//	2^16 -1
#define DIVIDER_MAX 	0xFFFF			//	2^16 -1
#define VALUE_MAX 		0xFFFFFFFF		//	2^32 -1

#define FREQUENCY_MAX	20000
#define FREQUENCY_MIN	0.1


#define NULL_IDX -1


//	This hosts pointer to all the Motor Controller that gets registered.
static volatile motorController_struct *motion_controller_registered[MAX_NUMBER_OF_DRIVERS];

//	Counts the number of Motion Controller that got registered.
static volatile int motion_controller_registered_number = 0;





/*############################
 * Static functions declarations:
 #############################
 */

/**
 * @brief 	given a timer and a desired reset frequency will set the prescaler
 * 			and TOP value to archieve such frequency
 *
 * @param desired_frequency -> Desired time rfrequency in Hz
 * @param htim -> memory address of the timer to modify
 *
 * @return -> -1 if error occurred, 0 if ok
 */
static ErrorStatus set_timer_frequency(float desired_frequency, TIM_HandleTypeDef *htim);

/**
 * @brief 	Will return the current base frequency for a given timer
 *
 * @param 	htim -> memory address of the timer of interest
 *
 * @return 	timer's base frequency in Hz.
 */
static ErrorStatus get_tim_clock(TIM_HandleTypeDef *htim, int *destination_buffer);


__attribute__((always_inline)) inline static void motion_update(TIM_HandleTypeDef *htim);
__attribute__((always_inline)) inline float pwm_mechanical2electrical(float pwm_mech_requested);





/*############################
 * Callbacks:
 #############################
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	motion_update(htim);
}





/*############################
 * Public functions definitions:
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
ErrorStatus start_motion_control(	volatile motorController_struct **motorController_pt,
									volatile hw_130_driver *hw_130,
									int frequency,
									TIM_HandleTypeDef *htim){

	//	If we already registered the max numebr of driver we return NULL pointer.
	if(motion_controller_registered_number >= MAX_NUMBER_OF_DRIVERS){
		return ERROR;
	}

	// create in memory (and initialize to 0) the motorManager object
	volatile motorController_struct *new_motor_controller= calloc(1, sizeof(motorController_struct));
	// If the initialization was not succesful, return NULL
	if(new_motor_controller == NULL){
		return ERROR;
	}

	//	Registering the HW 130 driver component to it:
	new_motor_controller->driver = hw_130;



	// Initialize the actual timer:
	if(set_timer_frequency(frequency, htim) != 0){
		// Could not initialize correctly the timer:
		// Release the memory:
		free(new_motor_controller);
		return ERROR;
	}
	if(	HAL_TIM_Base_Start_IT(htim)!= HAL_OK){
		// Could not initialize correctly the timer:
		// Release the memory:
		free(new_motor_controller);
		return ERROR;
	}


	//	Registering the interrupt generating timer to the motor manager.
	new_motor_controller->interrupt_timer = htim;
	new_motor_controller->time_increments = 1.0/(float)frequency;

	// Registering the motionManager to the motor manager array.
	motion_controller_registered[motion_controller_registered_number] = new_motor_controller;
	motion_controller_registered_number ++;

	*motorController_pt = new_motor_controller;
	return SUCCESS;
}





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
ErrorStatus set_target_speed_all( 	volatile motorController_struct *motion_controller,
									float m_1,
									float m_2,
									float m_3,
									float m_4,
									float acc){
	// Todo aggiungere logica di treshold.

	/*############################
	 * Checking validity of data received:
	 #############################
	 */

	//	Acceleration:
	if(acc <= 0){
		return ERROR;
	}


	//	Target speeds:
	float targets[4] = {m_1, m_2, m_3, m_4};
	for( int i = 0; i< 4; i++){
		if(targets[i] < -100 || targets[i] > 100){
			return ERROR;
		}
	}


	//	Pointer to a valid Motion Controller:
	bool pointer_valid = false;
	for(int i = 0; i < motion_controller_registered_number; i++){
		if(motion_controller_registered[i] == motion_controller){
			pointer_valid = true;
			break;
		}
	}
	if( !pointer_valid){
		return ERROR;
	}


	/*############################
	 * Update of the data:
	 #############################
	 */

	float delta [4];
	float delta_max = 0.0f;


	//	Disable interrupts such that we can update data consumed
	//	by the Motion Controller ISR update.
	__disable_irq();

	for (int i = 0; i < 4; i++) {
		delta[i] = fabsf(motion_controller->state[i].current_speed - targets[i]);
		if (delta[i] > delta_max) {
			delta_max = delta[i];
		}
	}


	float time_acc = delta_max / acc;
	float time_increment = motion_controller->time_increments;

	for (int i = 0; i < 4; i++) {
		motion_controller->state[i].desired_speed = targets[i];
		motion_controller->state[i].speed_increments = (time_acc > 0) ? (delta[i] * time_increment / time_acc) : 0.0f;
	}


	__enable_irq();



	return SUCCESS;
}





/*############################
 * Static functions definitions:
 #############################
 */


__attribute__((always_inline)) inline static void motion_update(TIM_HandleTypeDef *htim)
{


	//	Identify interrupt generator
	int controller_idx = NULL_IDX;
	for(int i = 0 ; i < motion_controller_registered_number; i++ ){
		if(motion_controller_registered[i]->interrupt_timer == htim){
			controller_idx = i;
			break;
		}
	}
	if(controller_idx == NULL_IDX){
		//	Could not find a controller associate to the timer generating interrupt.
		return;
	}

	//	Retrieve motor's driver object.
	volatile hw_130_driver *hw_130 = motion_controller_registered[controller_idx]->driver;


	//	Cicle trough the 4 motors and update them.
	for(int i = 0; i< 4; i++){


		//	Retrieveing the motor's state and some of the data:
		motionState_t state = motion_controller_registered[controller_idx]->state[i];

		float delta = state.desired_speed - state.current_speed;
		float speed_variation = state.speed_increments;
		float speed = state.current_speed;

		//	Checking for the direction of adjustment and for the magnitude of the update.
		if(delta > 0){
			// If close to target, cap the correction to the remaining difference:
			if(speed_variation < delta){
				speed_variation = delta;
			}
			// speed too low. increase.
			speed += speed_variation;
		}else{
			// If close to target, cap the correction to the remaining difference:
			if(-speed_variation > delta){
				speed_variation = -delta;	// needs to make it positive for correct sign in the next step.
			}
			// speed too high. Lower it.
			speed -= speed_variation;
		}

		//	Calculating speed and rotation as requested by hw130 driver:
		//	Also, applying a treshold such that low speed requests gets collapsed to 'stop' state.
		float speed_abs = abs(speed);
		motor_rotation direction = stop;
		if(speed_abs > 2.5){
			direction = (speed > 0) ? forwards : backwards;
		}
		float pwm_motor = pwm_mechanical2electrical(speed_abs);

		//	Updating motor's driver and the Motion Controller's object/state:
		motor_set(hw_130, i, direction, pwm_motor);
		state.current_speed = speed;
		motion_controller_registered[controller_idx]->state[i] = state;

	}

	//	Actually update the motor driver:
	motor_update(hw_130);

}


//	This fn. maps the mechanical pwm requested to the correct electrical pwm for the motor's driver.
__attribute__((always_inline)) inline float pwm_mechanical2electrical(float pwm_mech_requested){

	float pwm_electrical = pwm_mech_requested * ((float)(PWM_ELECTRIC_MAX - PWM_ELECTRIC_MIN) / 100);
	return pwm_electrical + PWM_ELECTRIC_MIN;
}



/**
 * @brief 	given a timer and a desired reset frequency will set the prescaler
 * 			and TOP value to archieve such frequency
 *
 * @param 	desired_frequency -> Desired time rfrequency in Hz
 * @param 	htim -> memory address of the timer to modify
 *
 * @return 	-> ERROR if error occurred
 * 			-> OK if setup successful.
 */
ErrorStatus set_timer_frequency(float desired_frequency, TIM_HandleTypeDef *htim){

	if(desired_frequency > FREQUENCY_MAX || desired_frequency < FREQUENCY_MIN){
		return ERROR;
	}

	int timer_clock = 0;
	ErrorStatus result =  get_tim_clock(htim, &timer_clock);
	if(result == ERROR){
		//get_tim_clock could not get the frequency of the desired timer.
		return ERROR;
	}

	double desired_period = 1 / (double) desired_frequency;
	if((timer_clock * desired_period) > (VALUE_MAX)){
		return ERROR;
	}


	double scale = timer_clock / desired_frequency;
	double smallest_top = scale /  PRESCALER_MAX;
	uint32_t top =  smallest_top + 1;
	uint32_t prescaler = roundf( scale / (double)top);


	//	Actually setting the newfound values in the timer:
	// Stop the timer
	HAL_TIM_Base_Stop(htim);

	// Set new prescaler and top counting values:
	__HAL_TIM_SET_PRESCALER(htim, prescaler);
	__HAL_TIM_SET_AUTORELOAD(htim, top);

	// Reset and Restart counter
	__HAL_TIM_SET_COUNTER(htim, 0);
	//HAL_TIM_Base_Start(htim);

	return SUCCESS;
}




/**
 * @brief 	Will return the current base frequency for a given timer
 *
 * @param htim -> memory address of the timer of interest
 *
 * @return 	-> ERROR if problem in getting timer's info.
 * 			-> Timer's base frequency in Hz.
 */
ErrorStatus get_tim_clock(TIM_HandleTypeDef *htim, int *destination_buffer) {

    RCC_ClkInitTypeDef clk_config;
    uint32_t latency;
    HAL_RCC_GetClockConfig(&clk_config, &latency);

    BusType timer_bus = getTimerBus(htim);

    switch(timer_bus){
    case BUS_APB1:
    	uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    	if (clk_config.APB1CLKDivider != RCC_HCLK_DIV1) {
    		*destination_buffer = pclk1 * 2;
			return SUCCESS;
		} else {
			*destination_buffer = pclk1;
			return SUCCESS;
		}
    	break;
    case BUS_APB2:

    	uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
		if (clk_config.APB2CLKDivider != RCC_HCLK_DIV1) {
			*destination_buffer = pclk2 * 2;
			return SUCCESS;
		} else {
			*destination_buffer = pclk2;
			return SUCCESS;
		}
		break;

    case BUS_UNKNOWN:
    	//	Error:
    	return ERROR;
    	break;

    default:
    	//	Error:
    	return ERROR;
    	break;
    }

    //	This point should be unreacheable.
    //	If execution reaches this something was very wrong.
    return ERROR;

}
