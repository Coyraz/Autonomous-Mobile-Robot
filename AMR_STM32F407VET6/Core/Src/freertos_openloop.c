/* ============================================================
 * freertos.c - OPEN LOOP CHARACTERIZATION MODE
 * ============================================================
 * PURPOSE: This is a TEMPORARY firmware for Phase 1 open loop
 *          characterization only. It bypasses PID completely.
 *
 * COMMAND FORMAT ACCEPTED:
 *   "P:LEFT_PWM,RIGHT_PWM\r\n"
 *   where LEFT_PWM and RIGHT_PWM are integers from 0 to 999.
 *   Example: "P:300,300\r\n" spins both wheels at PWM=300.
 *   Example: "P:0,0\r\n" stops both wheels.
 *
 * TELEMETRY FORMAT SENT (20Hz):
 *   {"l":LEFT_DELTA,"r":RIGHT_DELTA,"lp":LEFT_PWM,"rp":RIGHT_PWM}\r\n
 *   where LEFT_DELTA and RIGHT_DELTA are accumulated tick counts
 *   since power-on (signed 32-bit, handles overflow correctly).
 *
 * SAFETY:
 *   If no "P:..." command is received for 2000ms, both motors
 *   are stopped immediately. This prevents runaway robot.
 *
 * HOW TO RESTORE:
 *   After characterization is done, replace this file with the
 *   original freertos.c that contains the PID logic.
 * ============================================================
 */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "usart.h"
#include "tim.h"
#include "motor_control.h"
/* USER CODE END Includes */

/* ============================================================
 * IMPORTANT: We need a new function Motor_SetRawPWM() that
 * writes directly to the timer compare registers WITHOUT any
 * mm/s to PWM scaling math.
 *
 * Motor_SetSpeed() in motor_control.c does this math:
 *   pwm = (abs(speed) * 999) / 1000
 * So if we passed PWM=300 into Motor_SetSpeed(), it would
 * treat 300 as mm/s and output:
 *   pwm = (300 * 999) / 1000 = 299  (roughly the same)
 * Actually in this case it would nearly work, but the units
 * are conceptually wrong and it would clamp at speed 1000.
 * For PWM values above 1000 it would fail completely.
 * More importantly, this creates confusion about what you are
 * actually commanding. Keep it clean: use raw PWM directly.
 *
 * We define Motor_SetRawPWM() right here in this file so we
 * do not need to modify motor_control.c or motor_control.h.
 * This keeps the change isolated to freertos.c only.
 * ============================================================ */

/* ============================================================
 * RAW PWM FUNCTION (defined locally, forward direction only)
 * ============================================================ */
static void Motor_SetRawPWM(uint16_t left_pwm, uint16_t right_pwm)
{
    /* Clamp to valid range */
    if (left_pwm  > 999) left_pwm  = 999;
    if (right_pwm > 999) right_pwm = 999;

    /* --- LEFT MOTOR ---
     * Forward direction: IN1=HIGH, IN2=LOW
     * We always run forward for characterization.
     * Negative speeds are not needed here. */
    if (left_pwm > 0) {
        HAL_GPIO_WritePin(MOTOR_L_IN1_GPIO_Port, MOTOR_L_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_L_IN2_GPIO_Port, MOTOR_L_IN2_Pin, GPIO_PIN_RESET);
    } else {
        /* PWM=0 means stop: coast, both pins LOW */
        HAL_GPIO_WritePin(MOTOR_L_IN1_GPIO_Port, MOTOR_L_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_L_IN2_GPIO_Port, MOTOR_L_IN2_Pin, GPIO_PIN_RESET);
    }

    /* --- RIGHT MOTOR ---
     * Forward direction: IN3=HIGH, IN4=LOW */
    if (right_pwm > 0) {
        HAL_GPIO_WritePin(MOTOR_R_IN3_GPIO_Port, MOTOR_R_IN3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_R_IN4_GPIO_Port, MOTOR_R_IN4_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(MOTOR_R_IN3_GPIO_Port, MOTOR_R_IN3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_R_IN4_GPIO_Port, MOTOR_R_IN4_Pin, GPIO_PIN_RESET);
    }

    /* Write PWM values directly to TIM8 compare registers */
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, left_pwm);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, right_pwm);
}

/* ============================================================
 * PRIVATE VARIABLES
 * ============================================================ */
#define CMD_BUFFER_SIZE  64
#define CMD_TIMEOUT_MS   2000   /* Stop motors if silent for 2 sec */

static uint8_t uart_rx_buffer[CMD_BUFFER_SIZE];
static uint8_t uart_rx_index = 0;

/* Current commanded PWM values (shared between UART ISR and main task) */
/* volatile tells the compiler: do not cache this in a register,
 * always read from RAM, because another piece of code (the ISR)
 * can change it at any time. */
static volatile uint16_t g_left_pwm  = 0;
static volatile uint16_t g_right_pwm = 0;
static volatile uint32_t g_last_cmd_time = 0;

/* Accumulated tick counters (handles 16-bit timer overflow) */
/* The encoder timers are 16-bit hardware counters (0 to 65535).
 * When the motor spins fast and the counter wraps around from
 * 65535 back to 0, we need to detect that and add/subtract 65536
 * to keep a correct running total. That is what the delta logic
 * below does. */
static int32_t g_total_left_ticks  = 0;
static int32_t g_total_right_ticks = 0;

/* ============================================================
 * FREERTOS TASK SETUP
 * ============================================================ */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name       = "defaultTask",
    .stack_size = 3000 * 4,
    .priority   = (osPriority_t) osPriorityNormal,
};

void StartDefaultTask(void *argument);
void MX_FREERTOS_Init(void);

void MX_FREERTOS_Init(void)
{
    defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
                                    &defaultTask_attributes);
}

/* ============================================================
 * MAIN TASK
 * ============================================================ */
void StartDefaultTask(void *argument)
{
    /* 1. Start both encoder timers */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);  /* Right wheel */
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);  /* Left wheel  */

    /* 2. Initialize motor hardware (starts PWM timers, sets pins LOW) */
    Motor_Init();

    /* 3. Record the startup time for the safety timeout */
    g_last_cmd_time = HAL_GetTick();

    /* 4. Send a startup message so you can confirm firmware is running */
    char start_msg[] = "[CHAR_MODE:READY] Send P:LEFT,RIGHT to command PWM\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)start_msg, strlen(start_msg), 200);

    /* 5. Start UART receive interrupt.
     * This makes the UART hardware call HAL_UART_RxCpltCallback()
     * every time one byte arrives. We receive one byte at a time. */
    HAL_UART_Receive_IT(&huart1, &uart_rx_buffer[uart_rx_index], 1);

    /* --------------------------------------------------------
     * Variables for encoder delta calculation
     * We store the previous raw counter value so we can compute
     * how many ticks happened since the last loop cycle.
     * -------------------------------------------------------- */
    uint16_t prev_left_raw  = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    uint16_t prev_right_raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

    char tx_buffer[128];

    /* --------------------------------------------------------
     * Telemetry divider: task runs at 100Hz (every 10ms),
     * but we only send serial data every 5 cycles = 20Hz.
     * -------------------------------------------------------- */
    uint8_t telemetry_counter = 0;

    /* --------------------------------------------------------
     * MAIN LOOP - runs at 100Hz
     * -------------------------------------------------------- */
    for (;;)
    {
        /* --- A. READ ENCODER COUNTERS AND COMPUTE DELTA ---
         *
         * The timer counter register is 16-bit unsigned (0 to 65535).
         * We read it, subtract the previous value, then handle
         * the wrap-around case.
         *
         * Example of wrap-around:
         *   prev = 65500, current = 100
         *   naive delta = 100 - 65500 = -65400 (WRONG)
         *   correct delta = (65536 - 65500) + 100 = 136 (RIGHT)
         *
         * The cast to int16_t does this automatically because
         * int16_t can hold -32768 to +32767. If the wheel spins
         * forward and counter wraps, the subtraction naturally
         * gives the right small positive delta.
         * But if the robot moves fast and spins more than 32767
         * ticks in 10ms (impossible at these speeds), it breaks.
         * At maximum motor speed you get roughly:
         *   1 rev/s max => 4600 ticks/s => 46 ticks per 10ms
         * So int16_t delta is safe here. */

        uint16_t curr_left_raw  = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
        uint16_t curr_right_raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

        int16_t delta_left  = (int16_t)(curr_left_raw  - prev_left_raw);
        int16_t delta_right = (int16_t)(curr_right_raw - prev_right_raw);

        g_total_left_ticks  += delta_left;
        g_total_right_ticks += delta_right;

        prev_left_raw  = curr_left_raw;
        prev_right_raw = curr_right_raw;

        /* --- B. SAFETY TIMEOUT ---
         * If the Pi has not sent a "P:..." command in the last 2 seconds,
         * stop both motors immediately. This prevents the robot from
         * running away if the Pi crashes or the serial cable falls out. */
        uint32_t now = HAL_GetTick();
        if ((now - g_last_cmd_time) > CMD_TIMEOUT_MS) {
            g_left_pwm  = 0;
            g_right_pwm = 0;
        }

        /* --- C. APPLY CURRENT PWM COMMAND TO MOTORS ---
         * Read the volatile globals that the UART ISR may have updated. */
        Motor_SetRawPWM(g_left_pwm, g_right_pwm);

        /* --- D. SEND TELEMETRY AT 20Hz ---
         * We send: accumulated ticks and current PWM values.
         * The Python script on the Pi reads this to measure speed. */
        telemetry_counter++;
        if (telemetry_counter >= 5) {
            telemetry_counter = 0;

            int len = snprintf(tx_buffer, sizeof(tx_buffer),
                "{\"l\":%ld,\"r\":%ld,\"lp\":%u,\"rp\":%u}\r\n",
                g_total_left_ticks,
                g_total_right_ticks,
                (unsigned)g_left_pwm,
                (unsigned)g_right_pwm);

            if (len > 0 && len < (int)sizeof(tx_buffer)) {
                HAL_UART_Transmit(&huart1, (uint8_t*)tx_buffer, len, 10);
            }
        }

        /* --- E. SLEEP FOR 10ms (100Hz loop rate) --- */
        osDelay(10);
    }
}

/* ============================================================
 * UART RECEIVE INTERRUPT CALLBACK
 *
 * This function is called by HAL every time one byte arrives
 * over UART. We build up a string byte by byte until we see
 * a newline, then parse the complete command.
 *
 * Command format accepted: "P:LEFT_PWM,RIGHT_PWM\r\n"
 * Example: "P:300,450\r\n"
 *
 * Any other format is silently ignored.
 * ============================================================ */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    uint8_t received_byte = uart_rx_buffer[uart_rx_index];

    if (received_byte == '\n' || received_byte == '\r') {
        /* End of line received. If we have at least one character,
         * try to parse the command. */
        if (uart_rx_index > 0) {
            uart_rx_buffer[uart_rx_index] = '\0';

            uint16_t lp = 0;
            uint16_t rp = 0;

            /* sscanf parses the string. %hu means unsigned short (16-bit).
             * Returns the number of items successfully matched.
             * We need exactly 2 to have a valid command. */
            if (sscanf((char*)uart_rx_buffer, "P:%hu,%hu", &lp, &rp) == 2) {
                /* Clamp to safe range */
                if (lp > 999) lp = 999;
                if (rp > 999) rp = 999;

                g_left_pwm      = lp;
                g_right_pwm     = rp;
                g_last_cmd_time = HAL_GetTick();
            }
            /* If sscanf does not match 2 values, we ignore the line.
             * This means garbage data or wrong format will not
             * accidentally move the motors. */
        }

        uart_rx_index = 0;

    } else {
        /* Not end of line yet. Store the byte and advance the index. */
        uart_rx_index++;
        if (uart_rx_index >= (CMD_BUFFER_SIZE - 1)) {
            /* Buffer overflow protection: if we receive too many bytes
             * without a newline, reset. This prevents a bug where a
             * corrupted long string fills the buffer. */
            uart_rx_index = 0;
        }
    }

    /* Re-arm the UART interrupt to receive the next byte.
     * Without this line, only the first byte would ever trigger
     * the callback. The interrupt is one-shot and must be
     * re-enabled after each reception. */
    HAL_UART_Receive_IT(&huart1, &uart_rx_buffer[uart_rx_index], 1);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    /* Not used in characterization mode */
    (void)GPIO_Pin;
}
