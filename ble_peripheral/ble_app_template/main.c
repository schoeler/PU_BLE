/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
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
 *
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "boards.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "device_manager.h"
#include "pstorage.h"
#include "app_trace.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "nrf_drv_twi.h"
#include "drv_si705x.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define CENTRAL_LINK_COUNT               0                                          /**<number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT            1                                          /**<number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                      "Ben"                       						    /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                "NordicSemiconductor"                      /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                 300                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS       BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED      /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER              0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE          4                                          /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(201, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(201, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                    8                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                   1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                         /**< Maximum encryption key size. */

#define DEAD_BEEF                        0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static dm_application_instance_t        m_app_handle;                               /**< Application identifier allocated by device manager */

static uint16_t                          m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */
static ble_si705x_t                      m_si705x;                                      /**< LED Button Service instance. */

// YOUR_JOB: Declare all services structure your application is using
//static ble_xx_service_t                     m_xxs;
//static ble_yy_service_t                     m_yys;


// YOUR_JOB: Use UUIDs for service(s) used in your application.
static ble_uuid_t m_adv_uuids[] = { {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},
																		{BLE_UUID_BATTERY_SERVICE,            BLE_UUID_TYPE_BLE} }; /**< Universally unique service identifiers. */

                                   
/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
APP_TIMER_DEF(m_app_timer_id);

void timer_timeout_handler(void * p_context);


static void timers_init(void)
{

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    // Create timers.

    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
                 For every new timer needed, increase the value of the macro APP_TIMER_MAX_TIMERS by
                 one.*/
    uint32_t err_code;
	  //memset(&m_app_timer_id, 0, sizeof(m_app_timer_id));
    err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
    APP_ERROR_CHECK(err_code); 
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    // YOUR_JOB: Use an appearance value matching the application's use case.
    err_code = sd_ble_gap_appearance_set(0x1234);
    APP_ERROR_CHECK(err_code); 

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;
																					

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the YYY Service events. 
 * YOUR_JOB implement a service handler function depending on the event the service you are using can generate
 *
 * @details This function will be called for all YY Service events which are passed to
 *          the application.
 *
 * @param[in]   p_yy_service   YY Service structure.
 * @param[in]   p_evt          Event received from the YY Service.
 *
 *
static void on_yys_evt(ble_yy_service_t     * p_yy_service, 
                       ble_yy_service_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_YY_NAME_EVT_WRITE:
            APPL_LOG("[APPL]: charact written with value %s. \r\n", p_evt->params.char_xx.value.p_str);
            break;
        
        default:
            // No implementation needed.
            break;
    }
}*/

static uint32_t temp_char_add(ble_si705x_t * p_si705x, const ble_si705x_init_t * p_si705x_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_si705x->uuid_type;
    ble_uuid.uuid = SI705x_UUID_TEMP_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t)*3;
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint8_t)*3;
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_si705x->service_handle,
                                               &char_md,
                                               &attr_char_value,
                                               &p_si705x->temp_char_handles);
}

static void temp_event_handler(uint8_t pin_no, uint8_t button_action)
{
	
}
	
uint32_t ble_si705x_init(ble_si705x_t * p_si705x, const ble_si705x_init_t * p_si705x_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure.
    p_si705x->conn_handle       = BLE_CONN_HANDLE_INVALID;
    p_si705x->si705x_handler = p_si705x_init->si705x_handler;

    // Add service.
    ble_uuid128_t base_uuid = {LBS_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_si705x->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_si705x->uuid_type;
    ble_uuid.uuid = LBS_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_si705x->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add characteristics.
    err_code = temp_char_add(p_si705x, p_si705x_init);
    VERIFY_SUCCESS(err_code);

    //err_code = led_char_add(p_lbs, p_lbs_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}


/**@brief Function for handling write events to the LED characteristic.
 *
 * @param[in] p_lbs     Instance of LED Button Service to which the write applies.
 * @param[in] led_state Written/desired state of the LED.
 */
static void si705x_handler(ble_si705x_t * p_lbs, uint8_t led_state)
{
    if (led_state)
    {
        //LEDS_ON(LEDBUTTON_LED_PIN);
    }
    else
    {
        //LEDS_OFF(LEDBUTTON_LED_PIN);
    }
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_si705x_init_t init;

    init.si705x_handler = si705x_handler;

    err_code = ble_si705x_init(&m_si705x, &init);
    APP_ERROR_CHECK(err_code);
	
    /* YOUR_JOB: Add code to initialize the services used by the application.
    uint32_t                           err_code;
    ble_xxs_init_t                     xxs_init;
    ble_yys_init_t                     yys_init;

    // Initialize XXX Service.
    memset(&xxs_init, 0, sizeof(xxs_init));

    xxs_init.evt_handler                = NULL;
    xxs_init.is_xxx_notify_supported    = true;
    xxs_init.ble_xx_initial_value.level = 100; 
    
    err_code = ble_bas_init(&m_xxs, &xxs_init);
    APP_ERROR_CHECK(err_code);

    // Initialize YYY Service.
    memset(&yys_init, 0, sizeof(yys_init));
    yys_init.evt_handler                  = on_yys_evt;
    yys_init.ble_yy_initial_value.counter = 0;

    err_code = ble_yy_service_init(&yys_init, &yy_init);
    APP_ERROR_CHECK(err_code);
    */
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
*/
static void application_timers_start(void)
{
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.*/
    uint32_t err_code;
    err_code = app_timer_start(m_app_timer_id, 32000, NULL);
    APP_ERROR_CHECK(err_code);
   
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}

void ble_si705x_on_ble_evt(ble_si705x_t *p_si705x, ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
     {
        case BLE_GAP_EVT_CONNECTED:
            p_si705x->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
					  UNUSED_PARAMETER(p_ble_evt);
					  p_si705x->conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

				case BLE_GATTS_EVT_WRITE:
						p_si705x->cccd_handle = p_ble_evt->evt.gatts_evt.params.authorize_request.request.write.handle;
						break;
        default:
            // No implementation needed.
            break;
    }

}

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
            {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_si705x_on_ble_evt(&m_si705x, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    /*YOUR_JOB add calls to _on_ble_evt functions from each service your application is using
    ble_xxs_on_ble_evt(&m_xxs, p_ble_evt);
    ble_yys_on_ble_evt(&m_yys, p_ble_evt);
    */
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
	
	  // From experimental blinky
	  ble_gap_addr_t addr;
    err_code = sd_ble_gap_address_get(&addr);
    APP_ERROR_CHECK(err_code);
    err_code = sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE, &addr);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            err_code = ble_advertising_restart_without_whitelist();
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}


/**@brief Function for handling the Device Manager events.
 *
 * @param[in] p_evt  Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    APP_ERROR_CHECK(event_result);

#ifdef BLE_DFU_APP_SUPPORT
    if (p_event->event_id == DM_EVT_LINK_SECURED)
    {
        app_context_load(p_handle);
    }
#endif // BLE_DFU_APP_SUPPORT

    return NRF_SUCCESS;
}


/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), 
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

typedef struct 
{
		uint8_t cmd_id[2];
		uint8_t cmd_len;
	  uint8_t param_len;
} Si705xCmdStruct;

#define SI705x_ADDR				(0x40U) // >> 1)
#define SI705x_SCL_PIN	7
#define SI705x_SDA_PIN	9
#define SI705x_FREQ		NRF_TWI_FREQ_100K
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
	{ .cmd_id = { 0xE3 }, 			.cmd_len = 1,	.param_len = 3 },
	{ .cmd_id = { 0xF3 }, 			.cmd_len = 1,	.param_len = 3 },
	{ .cmd_id = { 0xFE }, 			.cmd_len = 1, .param_len = 0 },
	{ .cmd_id = { 0xE6 }, 			.cmd_len = 1, .param_len = 1 },
	{ .cmd_id = { 0xE7 }, 			.cmd_len = 1, .param_len = 1 },
	{ .cmd_id = { 0xFA, 0x0F },	.cmd_len = 2, .param_len = 8 },
	{ .cmd_id = { 0xFC, 0xC9 },	.cmd_len = 2, .param_len = 6 },
	{ .cmd_id = { 0x84, 0xB8 }, .cmd_len = 2, .param_len = 1 },
	{ .cmd_id = { 0xFC, 0xC9 }, .cmd_len = 2, .param_len = 1 }	
};


/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{   
    ret_code_t err_code;
    
    switch(p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TXRX)
            {
						   	   int x = 1;
								x ++;
							int y = x;

            }

            break;
        default:
            break;        
    }   
}

/**
 * @brief I2C initialization.
 */

static const nrf_drv_twi_t m_twi_si_705x = NRF_DRV_TWI_INSTANCE(0);

void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_si_705x_config = {
       .scl                = SI705x_SCL_PIN,
       .sda                = SI705x_SDA_PIN,
       .frequency          = SI705x_FREQ,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };
    
#if 0
    err_code = nrf_drv_twi_init(&m_twi_si_705x, &twi_si_705x_config, twi_handler, NULL);
#else
	  err_code = nrf_drv_twi_init(&m_twi_si_705x, &twi_si_705x_config, NULL, NULL);
#endif
    APP_ERROR_CHECK(err_code);
    
    nrf_drv_twi_enable(&m_twi_si_705x);
}


ret_code_t si705x_read(int cmd_id, uint8_t *ret_val)
{
		/* Start transaction with a slave with the specified address. */
		ret_code_t err_code = nrf_drv_twi_tx(&m_twi_si_705x, SI705x_ADDR, 
															gSi705xCmdTable[cmd_id].cmd_id, 
															gSi705xCmdTable[cmd_id].cmd_len,
															false);

		if (err_code)
			return err_code;
		
		err_code = nrf_drv_twi_rx(&m_twi_si_705x, SI705x_ADDR, ret_val, gSi705xCmdTable[cmd_id].param_len);

		return err_code;
}

uint8_t FW_VER, HW_VER;

const uint32_t UICR_ADDR_0x20C    __attribute__((at(0x1000120C))) __attribute__((used)) = 0xFFFFFFFE;
/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;

		uint32_t nfcpins = (*(uint32_t *)0x1000120C);
		if (nfcpins & 1)
		{
    //nrf_nvmc_write_word(0x1000120C, 0xFFFFFFFE);
    NVIC_SystemReset();
		}



	  
	  // Configure LED-pin as output
    LEDS_CONFIGURE(1 << 26);
	
	  // Toggle LED
		LEDS_INVERT(1 << 26);
		nrf_delay_ms(500);

		LEDS_INVERT(1 << 26);
	
		// Initialize I2C Interface
    twi_init();
		

#if 1
		// Read HW and FW versions
		err_code = si705x_read(SI705x_CMD_READ_VER_FW, &FW_VER);
		if (err_code)
			while(1);
		
		err_code = si705x_read(SI705x_CMD_READ_VER_HW, &HW_VER);
		if (err_code)
			while(1);
		
		uint8_t reading[3];
		err_code = si705x_read(SI705x_CMD_MEAS_HOLD, reading);
#endif		


#if 0
		uint8_t reading[3];
		memset(&reading, 0, sizeof(reading));

		
		nrf_drv_twi_xfer_desc_t xfer = NRF_DRV_TWI_XFER_DESC_TXRX(SI705x_ADDR, 
																						gSi705xCmdTable[SI705x_CMD_MEAS_HOLD].cmd_id,
																						gSi705xCmdTable[SI705x_CMD_MEAS_HOLD].cmd_len,
																						reading,
																						gSi705xCmdTable[SI705x_CMD_MEAS_HOLD].param_len);
		err_code = nrf_drv_twi_xfer(&m_twi_si_705x, &xfer, 0);
#endif
    timers_init();
	  sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);

	 // Initialize.

	  ble_stack_init();
		bool erase_bonds = true;
	  device_manager_init(erase_bonds);
    gap_params_init();
		services_init();
    advertising_init();
    conn_params_init();

    // Start execution.
    application_timers_start();
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
		
		err_code = sd_ble_gap_tx_power_set(4);
    APP_ERROR_CHECK(err_code);
		




    // Enter main loop.
    for (;;)
    {
        power_manage();	


    }
}

	static uint8_t test = 0x65;
void timer_timeout_handler(void * p_context)
{
int err_code = 0;

	//uint16_t len = sizeof(button_state);
	ble_si705x_t * p_si705x = &m_si705x;

	if (p_si705x->conn_handle != BLE_CONN_HANDLE_INVALID &&
		  p_si705x->cccd_handle == BLE_GATT_HVX_NOTIFICATION)
	{
#if 1
		uint8_t reading2[3];
		  err_code = si705x_read(SI705x_CMD_MEAS_HOLD, reading2);

		test = ((175.52  * (((uint16_t)reading2[0] << 8) | reading2[1]) / 65535) - 46.85) * 100;
		
	ble_gatts_hvx_params_t params;		
	memset(&params, 0, sizeof(params));
	params.type = BLE_GATT_HVX_NOTIFICATION;
	params.handle = p_si705x->temp_char_handles.value_handle;
	params.p_data = reading2;
	uint16_t len = 3;
	params.p_len = &len;
		
		//int err_code = 
		sd_ble_gatts_hvx(p_si705x->conn_handle, &params);
#else
		
  ble_gatts_value_t p_param;
		p_param.len = 1;
		p_param.offset = 0;
		p_param.p_value = &test;

	//	p_si705x->temp_char_handles
  err_code = 	sd_ble_gatts_value_set(p_si705x->conn_handle, p_si705x->temp_char_handles.value_handle, &p_param);
#endif
	APP_ERROR_CHECK(err_code);
	test++;
	}
}

/**
 * @}
 */
