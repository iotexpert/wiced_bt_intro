#include "wiced.h"
#include "sparcommon.h"
#include "wiced_platform.h"
#include "wiced_rtos.h"
#include "wiced_hal_gpio.h"
#include "wiced_bt_trace.h"

#include "wiced_hal_aclk.h"
#include "wiced_hal_pwm.h"

APPLICATION_START()
{
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);

    WICED_BT_TRACE("Hello, World\n");

    wiced_hal_aclk_enable(2000, ACLK1, ACLK_FREQ_1_MHZ );
    wiced_hal_pwm_configure_pin (WICED_GPIO_PIN_LED_1, PWM1 );
    wiced_hal_pwm_start(PWM1, PMU_CLK, 0xFFFF-500, 0xFFFF-999,0);

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
