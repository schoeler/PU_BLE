/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 * @defgroup tw_sensor_example main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_uart.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpiote.h"

/*Pins to connect shield. */
#define ARDUINO_I2C_SCL_PIN 7
#define ARDUINO_I2C_SDA_PIN 30

/*UART buffer size. */
#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 1

/*Common addresses definition for accelereomter. */
#define MMA7660_ADDR        (0x98U >> 1)

#define MMA7660_REG_XOUT    0x00U
#define MMA7660_REG_YOUT    0x01U
#define MMA7660_REG_ZOUT    0x02U
#define MMA7660_REG_TILT    0x03U
#define MMA7660_REG_SRST    0x04U
#define MMA7660_REG_SPCNT   0x05U
#define MMA7660_REG_INTSU   0x06U
#define MMA7660_REG_MODE    0x07U
#define MMA7660_REG_SR      0x08U
#define MMA7660_REG_PDET    0x09U
#define MMA7660_REG_PD      0x0AU

/* Mode for MMA7660. */
#define ACTIVE_MODE 1u

/*Failure flag for reading from accelerometer. */
#define MMA7660_FAILURE_FLAG (1u << 6)

/*Tilt specific bits*/
#define TILT_TAP_MASK (1U << 5)
#define TILT_SHAKE_MASK (1U << 7)

// [max 255, otherwise "int16_t" won't be sufficient to hold the sum
//  of accelerometer samples]
#define NUMBER_OF_SAMPLES 20

/* Define version of GCC. */
#define GCC_VERSION (__GNUC__ * 10000 \
                     + __GNUC_MINOR__ * 100 \
                     + __GNUC_PATCHLEVEL__)

/**
 * @brief Structure for holding sum of samples from accelerometer.
 */
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} sum_t;
static sum_t m_sum = {0};

/**
 * @brief Union to keep raw and converted data from accelerometer samples at one memory space.
 */
typedef union{
    uint8_t raw;
    int8_t  conv;
}elem_t;

/**
 * @brief Enum for selecting accelerometer orientation.
 */
typedef enum{
    LEFT = 1,
    RIGHT = 2,
    DOWN = 5,
    UP = 6
}accelerometer_orientation_t;

/**
 * @brief Structure for holding samples from accelerometer.
 */
typedef struct
{
    elem_t  x;
    elem_t  y;
    elem_t  z;
    uint8_t tilt;
} sample_t;

#ifdef __GNUC_PATCHLEVEL__
#if GCC_VERSION < 50505
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-braces"           // Hack to GCC 4.9.3 bug. Can be deleted after switch on using GCC 5.0.0
#endif
#endif
/* Buffer for samples. */
static sample_t m_sample_buffer[NUMBER_OF_SAMPLES] = {0};
#ifdef __GNUC_PATCHLEVEL__
#if GCC_VERSION < 50505
#pragma GCC diagnostic pop
#endif
#endif
/* Indicates if reading operation from accelerometer has ended. */
static volatile bool m_xfer_done = true;
/* Indicates if setting mode operation has ended. */
static volatile bool m_set_mode_done = false;
/* TWI instance. */
static const nrf_drv_twi_t m_twi_si_705x = NRF_DRV_TWI_INSTANCE(0);

/**
 * @brief Function for casting 6 bit uint to 6 bit int.
 *
 */
__STATIC_INLINE void int_to_uint(int8_t * put, uint8_t data)
{
    if (!(data & MMA7660_FAILURE_FLAG))     //6th bit is failure flag - we cannot read sample
    {
        *put = (int8_t)(data << 2) / 4;
    }
}

/**
 * @brief UART events handler.
 */
static void uart_events_handler(app_uart_evt_t * p_event)
{
    switch (p_event->evt_type)
    {
        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}


/**
 * @brief UART initialization.
 */
static void uart_config(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud38400
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_events_handler,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);

    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for setting active mode on MMA7660 accelerometer.
 */

typedef struct 
{
		uint8_t cmd_id[2];
		uint8_t cmd_len;
	  uint8_t param_len;
} Si705xCmdStruct;

#define SI705x_ADDR				(0x40U) // >> 1)
#define SI705x_SCL_PIN	7
#define SI705x_SDA_PIN	9
#define SI705x_FREQ		NRF_TWI_FREQ_400K
#define SI705x_CMD_MEAS_HOLD 		0
#define SI705x_CMD_MEAS_NO_HOLD	1
#define SI705x_CMD_RESET				2
#define SI705x_CMD_WRITE_UR1		3
#define SI705x_CMD_READ_UR1			4
#define SI705x_CMD_READ_EID_B1	5
#define SI705x_CMD_READ_EID_B2	6
#define SI705x_CMD_READ_VER_FW	7
#define SI705x_CMD_READ_VER_HW	8
#define SI705x_CMD_MAX					9

Si705xCmdStruct gSi705xCmdTable[SI705x_CMD_MAX] = 
{ 
	{ .cmd_id = { 0xE3 }, 			.cmd_len = 1,	.param_len = 3},
	{ .cmd_id = { 0xF3 }, 			.cmd_len = 1,	.param_len = 3},
	{ .cmd_id = { 0xFE }, 			.cmd_len = 1, .param_len = 0},
	{ .cmd_id = { 0xE6 }, 			.cmd_len = 1, .param_len = 1},
	{ .cmd_id = { 0xE7 }, 			.cmd_len = 1, .param_len = 1},
	{ .cmd_id = { 0xFA, 0x0F },	.cmd_len = 2, .param_len = 8 },
	{ .cmd_id = { 0xFC, 0xC9 },	.cmd_len = 2, .param_len = 6 },
	{ .cmd_id = { 0x84, 0xB8 }, .cmd_len = 2, .param_len = 1 },
	{ .cmd_id = { 0xFC, 0xC9 }, .cmd_len = 2, .param_len = 1 }	
};


/**
 * @brief Function for averaging samples from accelerometer.
 */
void read_data(sample_t * p_new_sample)
{
    /* Variable to count samples. */
    static uint8_t sample_idx;
    static uint8_t prev_tilt;
    
    sample_t * p_sample = &m_sample_buffer[sample_idx];
    
    /* Subtracting oldest sample. */
    m_sum.x    -= p_sample->x.conv;
    m_sum.y    -= p_sample->y.conv;
    m_sum.z    -= p_sample->z.conv;
    
    p_sample->tilt = p_new_sample->tilt;    
    
    int_to_uint(&p_sample->x.conv, p_new_sample->x.raw);
    int_to_uint(&p_sample->y.conv, p_new_sample->y.raw);
    int_to_uint(&p_sample->z.conv, p_new_sample->z.raw);
    
    /* Adding new sample. This way we always have defined number of samples. */
    m_sum.x    += p_sample->x.conv;
    m_sum.y    += p_sample->y.conv;
    m_sum.z    += p_sample->z.conv;

    ++sample_idx;
    if (sample_idx >= NUMBER_OF_SAMPLES)
    {
        sample_idx = 0;
    }

    if (sample_idx == 0 || (prev_tilt && (prev_tilt != p_sample->tilt)))
    {
        char const * orientation;
        switch ((p_sample->tilt >> 2) & 0x07)
        {
            case LEFT: 
                orientation = "LEFT";
                break;
            case RIGHT:
                orientation = "RIGHT"; 
                break;
            case DOWN:             
                orientation = "DOWN";  
                break;
            case UP:
                orientation = "UP";    
                break;
            default: 
                orientation = "?";     
                break;
        }
        printf("X: %3d, Y: %3d, Z: %3d | %s%s%s\r\n",
                m_sum.x / NUMBER_OF_SAMPLES,
                m_sum.y / NUMBER_OF_SAMPLES,
                m_sum.z / NUMBER_OF_SAMPLES,
                orientation,
                (p_sample->tilt & TILT_TAP_MASK) ? " TAP"   : "",
                (p_sample->tilt & TILT_SHAKE_MASK) ? " SHAKE" : "");
                prev_tilt = p_sample->tilt;
    }
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{   
    ret_code_t err_code;
    static sample_t m_sample;
    
    switch(p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if ((p_event->type == NRF_DRV_TWI_EVT_DONE) &&
                (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TX))
            {
                if(m_set_mode_done != true)
                {
                    m_set_mode_done  = true;
                    return;
                }
                m_xfer_done = false;
                /* Read 4 bytes from the specified address. */
                err_code = nrf_drv_twi_rx(&m_twi_si_705x, SI705x_ADDR, (uint8_t*)&m_sample, sizeof(m_sample));
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                read_data(&m_sample);
                m_xfer_done = true;
            }
            break;
        default:
            break;        
    }   
}


/**
 * @brief I2C initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_si_705x_config = {
       .scl                = SI705x_SCL_PIN,
       .sda                = SI705x_SDA_PIN,
       .frequency          = SI705x_FREQ,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };
    
    err_code = nrf_drv_twi_init(&m_twi_si_705x, &twi_si_705x_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_twi_enable(&m_twi_si_705x);
}


#define LEDS_CONFIGURE(leds_mask) do { uint32_t pin;                  \
                                  for (pin = 0; pin < 32; pin++) \
                                      if ( (leds_mask) & (1 << pin) )   \
                                          nrf_gpio_cfg_output(pin); } while (0)

#define LEDS_ON(leds_mask) do {  NRF_GPIO->OUTCLR = (leds_mask) & (LEDS_MASK & LEDS_INV_MASK); \
                           NRF_GPIO->OUTSET = (leds_mask) & (LEDS_MASK & ~LEDS_INV_MASK); } while (0)

ret_code_t si705x_read(int cmd_id, uint8_t *ret_val)
{
		/* Start transaction with a slave with the specified address. */
		ret_code_t err_code = nrf_drv_twi_tx(&m_twi_si_705x, SI705x_ADDR, 
															gSi705xCmdTable[cmd_id].cmd_id, 
															gSi705xCmdTable[cmd_id].cmd_len,
															true);

		if (err_code)
			return err_code;
		
		err_code = nrf_drv_twi_rx(&m_twi_si_705x, SI705x_ADDR, ret_val, gSi705xCmdTable[cmd_id].param_len);

		return err_code;
}



uint8_t FW_VER, HW_VER;
/**
 * @brief Function for main application entry.
 */
int main(void)
{
		// Unlock the NFC pins as GPIO
#if 0
		uint32_t nfcpins = (*(uint32_t *)0x1000120C);
		if (nfcpins & 1) {
				nrf_nvmc_write_word(0x1000120C, 0xFFFFFFFE);
				NVIC_SystemReset();
		}
#endif
		
	  // Configure LED-pin as output
    LEDS_CONFIGURE(1 << 26);

    // Toggle LED
		LEDS_INVERT(1 << 26);
		nrf_delay_ms(500);

		LEDS_INVERT(1 << 26);
		nrf_delay_ms(500);

    ret_code_t err_code;
#if 0	
		err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
	
	  nrf_drv_gpiote_out_config_t config = GPIOTE_CONFIG_OUT_TASK_TOGGLE(false);
    err_code = nrf_drv_gpiote_out_init(7, &config); 
		nrf_drv_gpiote_out_set(7);
	
	  err_code = nrf_drv_gpiote_out_init(9, &config); 
		nrf_drv_gpiote_out_set(9);
		#endif		
		// Initialize I2C Interface
    twi_init();
		
		// Read HW and FW versions
		err_code = si705x_read(SI705x_CMD_READ_VER_FW, &FW_VER);
		if (err_code)
			while(1);
		
		err_code = si705x_read(SI705x_CMD_READ_VER_HW, &HW_VER);
		if (err_code)
			while(1);
		
		uint8_t reading[3];
		err_code = si705x_read(SI705x_CMD_MEAS_HOLD, reading);
		
		uint8_t reading2[3];
		err_code = si705x_read(SI705x_CMD_MEAS_NO_HOLD, reading2);
		
		
#define SI_705x_CMD_R_UR1	0xE7
#define SI_705x_CMD_W_UR1	0xE6
	
    uint8_t reg[] = { SI_705x_CMD_R_UR1 };
		uint8_t reg_2[] = { 0x84, 0xB8 };

		uint8_t reg_temp[] = { 0xE3 };
		
		
    
		/* Start transaction with a slave with the specified address. */
		int size = sizeof(reg_2);
		err_code = nrf_drv_twi_tx(&m_twi_si_705x, SI705x_ADDR, reg_temp, sizeof(reg_temp), true);

	
		reg[0] = 0x00;
		uint8_t reg_temp_r[2] = { 0 };
		
		err_code = nrf_drv_twi_rx(&m_twi_si_705x, SI705x_ADDR, reg_temp_r, sizeof(reg_temp_r));

		m_xfer_done = false;
}

/** @} */
