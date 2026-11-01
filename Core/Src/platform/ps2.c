// PS2 transport. This module exposes only validated complete frames.
#include "ps2.h"
#include "ps2_frame.h"
#include "user_delay.h"
#include "user_usart.h"

#define DI HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)
#define DO_H HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)
#define DO_L HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET)
#define CS_H HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define CS_L HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define CLK_H HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET)
#define CLK_L HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET)

#define PS2_CLOCK_DELAY_US 10u
#define PS2_CONNECTED_TIMEOUT_MS 200u

static uint8_t latest_response[PS2_FRAME_LENGTH] = {0};
static uint16_t last_pressed_buttons = 0u;
static GamepadFrame latest_frame = {0};
static uint32_t last_valid_frame_ms = 0u;

static uint8_t PS2_Transfer(uint8_t command)
{
    uint8_t response = 0u;

    for (uint16_t bit = 0x01u; bit < 0x0100u; bit <<= 1)
    {
        if ((command & bit) != 0u)
        {
            DO_H;
        }
        else
        {
            DO_L;
        }

        CLK_H;
        HAL_Delay_us(PS2_CLOCK_DELAY_US);
        CLK_L;
        HAL_Delay_us(PS2_CLOCK_DELAY_US);
        CLK_H;
        if (DI == GPIO_PIN_SET)
        {
            response = (uint8_t)(response | bit);
        }
    }
    HAL_Delay_us(16u);
    return response;
}

static void PS2_ReadRawFrame(uint8_t response[PS2_FRAME_LENGTH])
{
    CS_L;
    HAL_Delay_us(16u);
    response[0] = PS2_Transfer(0x01u);
    response[1] = PS2_Transfer(0x42u);
    for (uint8_t index = 2u; index < PS2_FRAME_LENGTH; index++)
    {
        response[index] = PS2_Transfer(0x00u);
    }
    CS_H;
    HAL_Delay_us(16u);
}

uint8_t PS2_RedLight(void)
{
    uint8_t response_type;

    CS_L;
    HAL_Delay_us(16u);
    (void)PS2_Transfer(0x01u);
    response_type = PS2_Transfer(0x42u);
    CS_H;
    HAL_Delay_us(16u);
    return (response_type == 0x73u) ? 0u : 1u;
}

bool PS2_ReadFrame(GamepadFrame *frame)
{
    uint16_t current_pressed_buttons;

    if (frame == 0)
    {
        return false;
    }

    PS2_ReadRawFrame(latest_response);
    if (!PS2_DecodeGamepadFrame(latest_response,
                                sizeof(latest_response),
                                HAL_GetTick(),
                                last_pressed_buttons,
                                frame,
                                &current_pressed_buttons))
    {
        frame->valid = false;
        return false;
    }

    last_pressed_buttons = current_pressed_buttons;
    latest_frame = *frame;
    last_valid_frame_ms = frame->received_ms;
    return true;
}

bool PS2_IsConnected(void)
{
    return latest_frame.valid &&
           (uint32_t)(HAL_GetTick() - last_valid_frame_ms) <= PS2_CONNECTED_TIMEOUT_MS;
}

static void PS2_ShortPoll(void)
{
    CS_L;
    HAL_Delay_us(16u);
    (void)PS2_Transfer(0x01u);
    (void)PS2_Transfer(0x42u);
    (void)PS2_Transfer(0x00u);
    (void)PS2_Transfer(0x00u);
    (void)PS2_Transfer(0x00u);
    CS_H;
    HAL_Delay_us(16u);
}

static void PS2_EnterConfig(void)
{
    CS_L;
    HAL_Delay_us(16u);
    (void)PS2_Transfer(0x01u);
    (void)PS2_Transfer(0x43u);
    (void)PS2_Transfer(0x00u);
    (void)PS2_Transfer(0x01u);
    (void)PS2_Transfer(0x00u);
    (void)PS2_Transfer(0x00u);
    (void)PS2_Transfer(0x00u);
    (void)PS2_Transfer(0x00u);
    (void)PS2_Transfer(0x00u);
    CS_H;
    HAL_Delay_us(16u);
}

static void PS2_EnableAnalogMode(void)
{
    CS_L;
    (void)PS2_Transfer(0x01u);
    (void)PS2_Transfer(0x44u);
    (void)PS2_Transfer(0x00u);
    (void)PS2_Transfer(0x01u);
    (void)PS2_Transfer(0x03u);
    (void)PS2_Transfer(0x00u);
    (void)PS2_Transfer(0x00u);
    (void)PS2_Transfer(0x00u);
    (void)PS2_Transfer(0x00u);
    CS_H;
    HAL_Delay_us(16u);
}

static void PS2_EnableVibration(void)
{
    CS_L;
    HAL_Delay_us(16u);
    (void)PS2_Transfer(0x01u);
    (void)PS2_Transfer(0x4Du);
    (void)PS2_Transfer(0x00u);
    (void)PS2_Transfer(0x00u);
    (void)PS2_Transfer(0x01u);
    CS_H;
    HAL_Delay_us(16u);
}

static void PS2_ExitConfig(void)
{
    CS_L;
    HAL_Delay_us(16u);
    (void)PS2_Transfer(0x01u);
    (void)PS2_Transfer(0x43u);
    (void)PS2_Transfer(0x00u);
    (void)PS2_Transfer(0x00u);
    (void)PS2_Transfer(0x5Au);
    (void)PS2_Transfer(0x5Au);
    (void)PS2_Transfer(0x5Au);
    (void)PS2_Transfer(0x5Au);
    (void)PS2_Transfer(0x5Au);
    CS_H;
    HAL_Delay_us(16u);
}

void PS2_SetInit(void)
{
    PS2_ShortPoll();
    PS2_ShortPoll();
    PS2_ShortPoll();
    PS2_EnterConfig();
    PS2_EnableAnalogMode();
    PS2_EnableVibration();
    PS2_ExitConfig();
}

void PS2_Vibration(uint8_t motor1, uint8_t motor2)
{
    CS_L;
    HAL_Delay_us(16u);
    (void)PS2_Transfer(0x01u);
    (void)PS2_Transfer(0x42u);
    (void)PS2_Transfer(0x00u);
    (void)PS2_Transfer(motor1);
    (void)PS2_Transfer(motor2);
    (void)PS2_Transfer(0x00u);
    (void)PS2_Transfer(0x00u);
    (void)PS2_Transfer(0x00u);
    (void)PS2_Transfer(0x00u);
    CS_H;
    HAL_Delay_us(16u);
}

void PS2_print(void)
{
    if (!latest_frame.valid)
    {
        return;
    }

    usart_printf("\nPS2 lx:%u ly:%u rx:%u ry:%u buttons:%04x\n",
                 latest_frame.lx,
                 latest_frame.ly,
                 latest_frame.rx,
                 latest_frame.ry,
                 latest_frame.buttons);
}

void PS2_Vofa_print(void)
{
    if (!latest_frame.valid)
    {
        return;
    }

    usart_printf("%u,%u,%u,%u,%u,%u,%u\n",
                 latest_frame.lx,
                 latest_frame.ly,
                 latest_frame.rx,
                 latest_frame.ry,
                 latest_frame.buttons,
                 latest_frame.buttons_pressed,
                 latest_frame.buttons_released);
}
