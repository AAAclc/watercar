#include "bsp_key.h"

#define KEY_DEBOUNCE_MS 30   // 消抖时间

typedef enum {
    KS_IDLE = 0,
    KS_PRESS_DEBOUNCE,
    KS_HOLD,
    KS_RELEASE_DEBOUNCE
} KeyState;

static KeyState key_state[2] = {KS_IDLE, KS_IDLE};
static uint32_t key_tick[2];

void Key_Init(void)
{
    key_state[0] = KS_IDLE;
    key_state[1] = KS_IDLE;
}

uint8_t Key_Scan(void)
{
    uint8_t result = KEY_NONE;
    uint16_t pins[2]    = {GPIO_PIN_0, GPIO_PIN_1};
    uint8_t  codes[2]   = {KEY_START, KEY_RESET};

    for (int i = 0; i < 2; i++)
    {
        uint8_t pressed = (HAL_GPIO_ReadPin(GPIOC, pins[i]) == GPIO_PIN_RESET);

        switch (key_state[i])
        {
        case KS_IDLE:
            if (pressed) {
                key_state[i] = KS_PRESS_DEBOUNCE;
                key_tick[i] = HAL_GetTick();
            }
            break;

        case KS_PRESS_DEBOUNCE:
            if (pressed && HAL_GetTick() - key_tick[i] >= KEY_DEBOUNCE_MS) {
                key_state[i] = KS_HOLD;
                result = codes[i];
            } else if (!pressed) {
                key_state[i] = KS_IDLE;
            }
            break;

        case KS_HOLD:
            if (!pressed) {
                key_state[i] = KS_RELEASE_DEBOUNCE;
                key_tick[i] = HAL_GetTick();
            }
            break;

        case KS_RELEASE_DEBOUNCE:
            if (!pressed && HAL_GetTick() - key_tick[i] >= KEY_DEBOUNCE_MS) {
                key_state[i] = KS_IDLE;
            } else if (pressed) {
                key_state[i] = KS_HOLD;
            }
            break;
        }
    }
    return result;
}