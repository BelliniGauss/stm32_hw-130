# stm32_hw-130
hw-130 driver for stm32 


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
    
    hw_130_driver motor_driver =  create_hw_130( SR_DATA_Pin, SR_DATA_GPIO_Port, ... )
    
 - hw_130_driver is the type for the hw-130 driver used in the context of this library,
 - motor_driver is our actual variable that holds that object (it's a pointer to the driver struct), 
 - create_hw_130 function that takes in the pin number and pin port for the driver's Shift Register. 
 (See the .h file for exact parameters)


#### Assignment of motor's parameters: 

For each of the four motors it's necessary to initialize the relevan properties.
As of now it's necessary to initialize ALL the four motors.

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
    
    start_hw_130(motor_driver);

It simply takes in the the hw-130 object that we want to actually start up. 


### Operation: 

To Operate one or more motors of the hw-130 driver we firstly submit the new parameters 
for all the motors we need to modify. 
We do so by calling:

    motor_set(motor_driver, 3, direction, pwm);

Where, in order, we find: 
- the hw-130 object where the motor of interest resides
- the logical index of the motor of interest
- the new desired direction (see type motor_rotation in hw_130.h )
- the electrical pwm required, expressed in 0 - 100 interval


After this step no modification has yet been pushed to the actual driver. 
We apply all the modifications at one by calling

    update_motors(motor_driver);

where, once again, the parameter passed is the hw-130 object defined at the beginning.