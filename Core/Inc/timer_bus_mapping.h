
/**
 * 	Macro function to check what clock bus a timer is on.
 *
 */

typedef enum {
	BUS_APB1 = 10,
	BUS_APB2 = 11,
	BUS_UNKNOWN= 12
} BusType;

/**#######################################
 * 			STM32 F4XX family:
 * #######################################
 */

#ifdef __STM32F4xx_H
	#include "stm32f4xx.h"

	__attribute__((always_inline)) inline BusType getTimerBus(TIM_HandleTypeDef *htim) {
		#if defined(STM32F401xC) || defined(STM32F401xE)
		TIM_TypeDef *timer = htim->Instance;
			if (timer == TIM1 || timer == TIM9 || timer == TIM10 || timer == TIM11)
				return BUS_APB2;
			else
				return BUS_APB1;
		#else
			#error "STM32 model not supported by the Motor Controller! Add it to timer_bus_mapping.h"
			return BUS_UNKNOWN;
		#endif
	}
#endif


/**#######################################
 * 			STM32 L4XX family:
 * #######################################
 */

#ifdef __STM32L4xx_H
	#include "stm32l4xx.h"

	__attribute__((always_inline)) inline BusType getTimerBus(TIM_HandleTypeDef *htim) {
		#if defined(STM32L476xx)
		TIM_TypeDef *timer = htim->Instance;
			if (timer == TIM1 || timer == TIM8 || timer == TIM15 || timer == TIM16 || timer == TIM17)
				return BUS_APB2;
			else
				return BUS_APB1;
		#else
			#error "STM32 model not supported by the Motor Controller! Add it to timer_bus_mapping.h"
			return BUS_UNKNOWN;
		#endif
	}

#endif
