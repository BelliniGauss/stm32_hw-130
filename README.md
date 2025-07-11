# stm32_hw-130 - motor driver for speed ramp
hw-130 driver for stm32 
companion speed-ramp driver. 


## Usage
an example of usage is present in Core -> Src -> main.c

### Setup

The following is only the code-side setup. 
Other necessary steps, omitted here, includes: 
- finding the correct GPIO mapping
- finding timer and their channel associated to the pwm pins
- setting up those GPIO and timer. 


Setup is composed by 3 steps: 

#### Creation of the memory structure:

The object-like memory structure containing all the driver's parameter 
has to be created by allocating the necessary memory.
This is done in the declaration and initialization that follows:
    
    ErrorStatus create_hw_130(	volatile hw_130_driver **driver_pt,
                                    int sr_data_pin, GPIO_TypeDef* data_gpio_port,
                                    int	sr_clock_pin, GPIO_TypeDef* clock_gpio_port,
                                    int sr_latch_pin, GPIO_TypeDef* latch_gpio_port);

Example:

    hw_130_driver volatile *motor_driver = NULL;
    ErrorStyatus result = create_hw_130(&motor_driver, 	SR_DATA_Pin, SR_DATA_GPIO_Port,
                                                        SR_CLOCK_Pin, SR_CLOCK_GPIO_Port,
                                                        SR_LATCH_Pin, SR_LATCH_GPIO_Port);
    
 - hw_130_driver is the type for the hw-130 driver used in the context of this library,
 - motor_driver is a pointer to the driver struct containing the state. 
 - create_hw_130 function that takes in a pointer to the driver's pointer (this will get filled in with the address of the newly created driver) and the pin number and pin port for the driver's Shift Register. 
 (See the .h file for exact parameters). Returns ErrorStyatus SUCCESS or ERROR.


#### Assignment of motor's parameters: 

For each of the four motors it's necessary to initialize the relevan properties.
As of now it's necessary to initialize ALL the four motors.

    ErrorStatus motor_initialize(	volatile hw_130_driver *motor_driver,
                                    int n_motor,
                                    TIM_HandleTypeDef *timer,
                                    int channel,
                                    int pin_positive,
                                    int pin_negative,
                                    bool invert_default_direction);

Example:

    motor_initialize(motor_driver,	0, &htim2, TIM_CHANNEL_1, MOTOR_SR_1_P, MOTOR_SR_1_N, MOTOR_1_INVERT);

the initialization function takes in, in order:
- the hw-130 object defined at the beginning.
- logical index of the motor,
- memory address of the relevant timer, who's PWM is connected to the ENABLE pin of L293d driver for this specific motor, 
- timer channel on which we find the PWM output connected to 'Enable' pin on L293D driver for this specific Motor (is a channel of the just mentioned timer.)
- index of positive side L293D input in the Shift Register pinout
- index of negative side L293D input in the Shift Register pinout
- bool flag signaling the default rotation direction should be inverted. 


#### Initialization of the driver: 

This will reset the Shift Register to default zero, making the motor stationary at startup.
Will also initialize the pwm output on the selected timers and channels. 
    
    ErrorStatus start_hw_130(volatile hw_130_driver *motor_driver);

Example: 

    start_hw_130(motor_driver);

It simply takes in the the hw-130 object that we want to actually start up. 




### Motor Controller:
This component allows the user to request smooth speed ramps to the motors, 
without warrying about the actual control of the motors. 

#### Creation and Inizializaton of the Controller: 

Similarly to what seen for HW 130 driver, we use object-like approach. 
We can create more than one (up to 5 by default, can be changed by defining MAX_NUMBER_OF_DRIVERS )
that can manage more than one hw130 driver. 

    ErrorStatus start_motion_control(	volatile motorController_struct **motorController_pt,
                                            volatile hw_130_driver *hw_130,
                                            int frequency,
                                            TIM_HandleTypeDef *htim);

Example:

    volatile motorController_struct * control_driver = NULL;
    start_motion_control( &control_driver, motor_driver, 200, &htim10);

Only note is that here as well the initializating function takes a pointer to the struct pointer, 
to allocate the controller. 
It then takes an hw_130 driver as initialized before. 
A frequency: this is the update frequency of the ramp control loop.
A timer handler to the timer used for generating the update at the required frequency. It cannot be shared. 



### Operation: 

We'll focus only on the usage trough the ramp function: 


    ErrorStatus set_target_speed_all(	volatile motorController_struct *motion_controller,
                                            float m_1,
                                            float m_2,
                                            float m_3,
                                            float m_4,
                                            float acc);

Example: 

    set_target_speed_all(control_driver, pwm, pwm, pwm, pwm, 200);        

The usage is intuitive, pass a controller, the 4 desired pwm [-100;+100] 
and the max acceeleration admissible.

This will setup the library and start the requested ramp. 

