#include "wiced.h"
#include "sparcommon.h"
#include "wiced_platform.h"
#include "wiced_rtos.h"
#include "wiced_hal_gpio.h"
#include "wiced_bt_trace.h"

APPLICATION_START()
{
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);

    WICED_BT_TRACE("Hello, World\n");
    while(1)
    {

        WICED_BT_TRACE("Setting 0\n");
        wiced_hal_gpio_set_pin_output(WICED_GPIO_PIN_LED_2,0);
        wiced_rtos_delay_milliseconds(500,KEEP_THREAD_ACTIVE );
        WICED_BT_TRACE("Setting 1\n");
        wiced_hal_gpio_set_pin_output(WICED_GPIO_PIN_LED_2,1);
        wiced_rtos_delay_milliseconds(500,KEEP_THREAD_ACTIVE );
    }
}
