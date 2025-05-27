# stm32_hw-130
hw-130 driver for stm32 


## Usage
an example of usage is present in Core -> Src -> main.c

### Setup

Setup is composed by 3 steps: 

#### Creation of the memory structure:

The object-like memory structure containing all the driver's parameter 
has to be created by allocating the necessary memory.
This is done in the 
'''c
    hw_130_driver motor_driver =  create_hw_130
'''