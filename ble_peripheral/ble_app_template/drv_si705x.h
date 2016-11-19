#ifndef DRV_SI705X
#define DRV_SI705X

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define LBS_UUID_BASE        {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, \
                              0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}
#define LBS_UUID_SERVICE     0x1523
#define LBS_UUID_LED_CHAR    0x1525
#define LBS_UUID_BUTTON_CHAR 0x1527
#define SI705x_UUID_TEMP_CHAR 0x1600

// Forward declaration of the ble_lbs_t type. 
typedef struct ble_si705x_s ble_si705x_t;
															
typedef void (*ble_si705x_handler_t) (ble_si705x_t * p_si705x, uint8_t new_state);

typedef struct
{
    ble_si705x_handler_t si705x_handler; /**< Event handler to be called when the LED Characteristic is written. */
} ble_si705x_init_t;

/**@brief LED Button Service structure. This structure contains various status information for the service. */
struct ble_si705x_s
{
    uint16_t                    service_handle;      /**< Handle of LED Button Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    temp_char_handles; /**< Handles related to the Button Characteristic. */
    uint8_t                     uuid_type;           /**< UUID type for the LED Button Service. */
    uint16_t                    conn_handle;         /**< Handle of the current connection (as provided by the BLE stack). BLE_CONN_HANDLE_INVALID if not in a connection. */
    ble_si705x_handler_t        si705x_handler;      /**< Event handler to be called when the LED Characteristic is written. */
	  uint16_t 										cccd_handle;
};


#endif