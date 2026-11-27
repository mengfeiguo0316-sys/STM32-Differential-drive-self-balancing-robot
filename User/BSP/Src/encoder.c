#include "encoder.h"
#include "stm32f1xx_hal.h"


/* 定义单个编码器的硬件特性结构体 */
typedef struct {
    GPIO_TypeDef    *port;
    uint16_t        pin1;
    uint16_t        pin2;
} encoder_hw_config_t;

/* 硬件配置表：迁移时，只需修改这张表里的端口、引脚或电平极性 */
static const encoder_hw_config_t encoder_configs[ENCODER_NUM] = {
    [ENCODER_L] = {GPIOB, GPIO_PIN_6,GPIO_PIN_7},
    [ENCODER_R] = {GPIOC, GPIO_PIN_6, GPIO_PIN_7},
};



