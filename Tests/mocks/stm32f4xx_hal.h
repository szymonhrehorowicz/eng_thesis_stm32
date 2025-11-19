#if !defined(HAL_MOCK)
#define HAL_MOCK

typedef struct
{
} TIM_HandleTypeDef;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim5;

#define GPIOA 0
#define GPIOB 1
#define GPIOC 2
#define GPIOD 3

#define GPIO_PIN_1 0
#define GPIO_PIN_2 1
#define GPIO_PIN_3 2
#define GPIO_PIN_4 3
#define GPIO_PIN_5 4
#define GPIO_PIN_6 5
#define GPIO_PIN_7 6
#define GPIO_PIN_8 7
#define GPIO_PIN_9 8
#define GPIO_PIN_10 9
#define GPIO_PIN_11 10
#define GPIO_PIN_12 11
#define GPIO_PIN_13 12
#define GPIO_PIN_14 13
#define GPIO_PIN_15 14
#define GPIO_PIN_16 15

#define GPIO_PIN_SET 1
#define GPIO_PIN_RESET 0

#define TIM_CHANNEL_1 0
#define TIM_CHANNEL_2 1
#define TIM_CHANNEL_3 2
#define TIM_CHANNEL_4 3

int HAL_GPIO_WritePin(int, int, int);

#endif