#include "motor.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>

#define MAX_NUMBER_OF_DRIVERS	5



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
typedef struct motorManager_struct{
	hw_130_driver driver;					/**< HW 130 driver oject, to be called for executing motor output. */
	motionState_t state[4];					/**< Vector containing the 4 motor's motion state structs.*/
	TIM_HandleTypeDef *interrupt_timer;		/**< pointer to the timer calling the update of the Motor Controller.*/
	float time_increments;					/**< Precalculated time interval between two MC updates.*/
}motorManager_struct;






/*############################
 * Variables and Definitions:
 #############################
 */


#define PRESCALER_MAX 	0xFFFF			//	2^16 -1
#define DIVIDER_MAX 	0xFFFF			//	2^16 -1
#define VALUE_MAX 		0xFFFFFFFF		//	2^32 -1

#define NULL_IDX -1

//	This hosts pointer to all the Motor Controller that gets registered.
static volatile motorManager_struct *motion_controller_registered[MAX_NUMBER_OF_DRIVERS];

//	Counts the number of Motion Controller that got registered.
static volatile int motion_controller_registered_number = 0;





/*############################
 * Static functions declarations:
 #############################
 */

static int set_timer_frequency(uint32_t timer_desired_frequency, TIM_HandleTypeDef *htim);
static uint32_t get_tim_clock(TIM_HandleTypeDef *htim);


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


motorManager_struct* start_motion_control( hw_130_driver hw_130,  int frequency, TIM_HandleTypeDef *htim){

	//	If we already registered the max numebr of driver we return
	if(motion_controller_registered_number >= MAX_NUMBER_OF_DRIVERS){
		return NULL;
	}

	// create in memory (and initialize to 0) the motorManager object
	motorManager_struct *new_motorManager= calloc(1, sizeof(motorManager_struct));
	// If the initialization was not succesful, return error!
	if(new_motorManager == NULL){
		return NULL;
	}

	//	Registering the HW 130 driver component to it:
	new_motorManager->driver = hw_130;



	// Initialize the actual timer:
	if(set_timer_frequency(frequency, htim) != 0){
		// Could not initialize correctly the timer:
		// Release the memory:
		free(new_motorManager);
		return NULL;
	}
	if(	HAL_TIM_Base_Start_IT(htim)!= HAL_OK){
		// Could not initialize correctly the timer:
		// Release the memory:
		free(new_motorManager);
		return NULL;
	}


	//	Registering the interrupt generating timer to the motor manager.
	new_motorManager->interrupt_timer = htim;
	new_motorManager->time_increments = 1.0/(float)frequency;

	// Registering the motionManager to the motor manager array.
	motion_controller_registered[motion_controller_registered_number] = new_motorManager;
	motion_controller_registered_number ++;

	return new_motorManager;
}






void set_target_speed_all(motorManager_struct *motion_controller, float m_1, float m_2, float m_3, float m_4, float acc){



	__disable_irq();

	GPIOC->ODR = 1<<14;    		// DEBUG on

	// Todo aggiungere logica di treshold.

	float delta [4];

	delta[0] = abs(motion_controller->state[0].current_speed - m_1);
	delta[1] = abs(motion_controller->state[1].current_speed - m_2);
	delta[2] = abs(motion_controller->state[2].current_speed - m_3);
	delta[3] = abs(motion_controller->state[3].current_speed - m_4);

	float delta_max = delta[0];
	for (int i = 1; i<4 ; i++){
		 delta_max = (delta[i] >= delta_max) ? delta[i] : delta_max;
	}

	float time_acc = delta_max / acc;
	float time_increment = motion_controller->time_increments;



	motion_controller->state[0].desired_speed = m_1;
	motion_controller->state[0].speed_increments = delta[0] * time_increment / time_acc;
	motion_controller->state[1].desired_speed = m_2;
	motion_controller->state[1].speed_increments = delta[1] * time_increment / time_acc;
	motion_controller->state[2].desired_speed = m_3;
	motion_controller->state[2].speed_increments = delta[2] * time_increment / time_acc;
	motion_controller->state[3].desired_speed = m_4;
	motion_controller->state[3].speed_increments = delta[3] * time_increment / time_acc;

	GPIOC->ODR &= ~(1<<14);		//	DEBUG off

	__enable_irq();


}





/*############################
 * Static functions definitions:
 #############################
 */


__attribute__((always_inline)) inline static void motion_update(TIM_HandleTypeDef *htim)
{

	//__disable_irq(); 	// todo verify it's not actually needed...

	//	Identify interrupt generator
	int controller_idx = NULL_IDX;
	for(int i = 0 ; i < motion_controller_registered_number; i++ ){
		if(motion_controller_registered[i]->interrupt_timer == htim){
			controller_idx = i;
			break;
		}
	}
	if(controller_idx == NULL_IDX){
		//	Interrupt not reelated to our motion controller.
		//__enable_irq();
		return;
	}

	//	Retrieve motor's driver object.
	hw_130_driver motors = motion_controller_registered[controller_idx]->driver;


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
		float speed_abs = abs(speed);
		motor_rotation direction = stop;
		if(speed_abs > 2.5){
			direction = (speed > 0) ? forwards : backwards;
		}
		float pwm_motor = pwm_mechanical2electrical(speed_abs);

		//	Updating motor's driver and the Motion Controller's object/state:
		motor_set(motors, i, direction, pwm_motor);
		state.current_speed = speed;
		motion_controller_registered[controller_idx]->state[i] = state;

	}
	//__enable_irq();

	//	Set those values to the hw130 driver
	motor_update(motors);
}


//	This fn. maps the mechanical pwm requested to the correct electrical pwm for the motor's driver.
__attribute__((always_inline)) inline float pwm_mechanical2electrical(float pwm_mech_requested){

	float pwm_electrical = pwm_mech_requested * ((float)(PWM_ELECTRIC_MAX - PWM_ELECTRIC_MIN) / 100);
	return pwm_electrical + PWM_ELECTRIC_MIN;
}



/**
 * @brief 	given a timer and a desired frequency will set the prescaler and TOP value
 * 			to archieve such frequency
 *
 * @param timer_desired_frequency -> Desired time rfrequency in Hz
 * @param htim -> memory address of the timer to modify
 *
 * return -> -1 if error occurred, 0 if ok
 */
int set_timer_frequency(uint32_t timer_desired_frequency, TIM_HandleTypeDef *htim){

	double timer_clock = (double)get_tim_clock(htim);
	double timer_desired_period = 1 / (double) timer_desired_frequency;

	if((timer_clock*timer_desired_period) > (VALUE_MAX)){
		return -1;
	}

	//	TODO more error checking, not only on frequency value request.

	double scale = timer_clock / timer_desired_frequency;
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

	return 0;
}




/**
 * @brief 	Will return the current base frequency for a given timer
 *
 * @param htim -> memory address of the timer of interest
 *
 * @return 	timer's base frequency in Hz.
 */
uint32_t get_tim_clock(TIM_HandleTypeDef *htim) {
    RCC_ClkInitTypeDef clk_config;
    uint32_t latency;
    HAL_RCC_GetClockConfig(&clk_config, &latency);

    // Check if timer is on APB2
    if (htim->Instance == TIM1 ||
        htim->Instance == TIM9 || htim->Instance == TIM10 || htim->Instance == TIM11) {

        uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
        if (clk_config.APB2CLKDivider != RCC_HCLK_DIV1) {
            return pclk2 * 2;  // Timer clock is doubled
        } else {
            return pclk2;
        }
    } else {
        uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
        if (clk_config.APB1CLKDivider != RCC_HCLK_DIV1) {
            return pclk1 * 2;
        } else {
            return pclk1;
        }
    }
}
