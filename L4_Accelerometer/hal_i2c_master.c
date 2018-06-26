/*
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of Cypress Semiconductor 
 *  Corporation. All rights reserved. This software, including source code, documentation and  related 
 * materials ("Software"), is owned by Cypress Semiconductor  Corporation or one of its 
 *  subsidiaries ("Cypress") and is protected by and subject to worldwide patent protection  
 * (United States and foreign), United States copyright laws and international treaty provisions. 
 * Therefore, you may use this Software only as provided in the license agreement accompanying the 
 * software package from which you obtained this Software ("EULA"). If no EULA applies, Cypress 
 * hereby grants you a personal, nonexclusive, non-transferable license to  copy, modify, and 
 * compile the Software source code solely for use in connection with Cypress's  integrated circuit 
 * products. Any reproduction, modification, translation, compilation,  or representation of this 
 * Software except as specified above is prohibited without the express written permission of 
 * Cypress. Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO  WARRANTY OF ANY KIND, EXPRESS 
 * OR IMPLIED, INCLUDING,  BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED WARRANTIES OF MERCHANTABILITY 
 * AND FITNESS FOR A PARTICULAR PURPOSE. Cypress reserves the right to make changes to 
 * the Software without notice. Cypress does not assume any liability arising out of the application 
 * or use of the Software or any product or circuit  described in the Software. Cypress does 
 * not authorize its products for use in any products where a malfunction or failure of the 
 * Cypress product may reasonably be expected to result  in significant property damage, injury 
 * or death ("High Risk Product"). By including Cypress's product in a High Risk Product, the 
 *  manufacturer of such system or application assumes  all risk of such use and in doing so agrees 
 * to indemnify Cypress against all liability.
 */

 /** @file
 *
 * WICED sample application for I2C Master usage
 *
 * This application demonstrates how to use I2C driver interface
 * to send and receive bytes or a stream of bytes over the I2C hardware as a master.
 * The on-board LSM9DS1 motion sensor acts as the I2C slave
 *
 * Features demonstrated
 * - I2C WICED APIs
 *
 * This I2C Master sample application is developed for Arduino compatible development kits supported by Wiced.
 * The SCL and SDA pins are configured to Px pins during boot-up process and routed to the corresponding Arduino headers.
 * Please follow the Kit User Guide(Hardware User Manual) of the particular development kit for more pin details.
 * The development kits supported by Wiced does not require any fly wiring.
 *
 * Usage
 *
 * Program a kit with the i2c_master app. Power on the kit.
 * You can see the logs through PUART. The power supply on the kit should be 3.3V
 */

#include "sparcommon.h"
#include "wiced_bt_trace.h"
#include "wiced_platform.h"
#include "wiced_hal_puart.h"
#include "wiced_hal_i2c.h"
#include "wiced_timer.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/

#define POLL_TIMER 2 /*2s timer for reading from sensor*/
#define CLK_FREQ 24000 /*Clock frequency in KHz*/
#define LSM9DS1_ACC_GYRO_I2C_ADDRESS (0xD4 >> 1) /*Motion sensor slave address*/
#define LSM9DS1_ACC_GYRO_WHO_AM_I_REG 0X0F /*WHO_AM_I register address*/


/******************************************************************************
 *                                Structures
 ******************************************************************************/

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/
static UINT8 current_speed;
static wiced_timer_t seconds_timer;

 /******************************************************************************
 *                                Function Definitions
 ******************************************************************************/
 static void initialize_app( void );

 static void comboread_cb (uint32_t arg);

 void button_cb (void* user_data, uint8_t value);

/******************************************************************************/
/*
 *  Entry point to the application.
 *****************************************************************************/
APPLICATION_START( )
{

    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );

    WICED_BT_TRACE( "\n\r--------------------------------------------------------------- \r\n\n"
                         "                 I2C Master Sample Application \n\r\n\r"
                         "---------------------------------------------------------------\n\r"
                         "      This application reads a register from I2C slave \r\n"
                         "     every %d seconds and displays the read data. Button\r\n"
                         "press results in toggling of I2C speed between 100 and 400 KHz\n\r"
                         "---------------------------------------------------------------\n\n\r", POLL_TIMER );

    initialize_app( );
}

/******************************************************************************
 * This functions initializes the I2C, button and POLL_TIMER second timer
******************************************************************************/

void initialize_app( void )
{
    wiced_hal_i2c_init();

    // Turn on Accelerometer - Register 0x20... 2g accelerometer on @ 50hz
    uint8_t data[] = {0x20, 0x40};
    wiced_hal_i2c_write(data,sizeof(data),LSM9DS1_ACC_GYRO_I2C_ADDRESS);

    /* register callback for button available on the platform */
    wiced_platform_register_button_callback( WICED_PLATFORM_BUTTON_1, button_cb, NULL, WICED_PLATFORM_BUTTON_RISING_EDGE);

    current_speed = wiced_hal_i2c_get_speed();

    WICED_BT_TRACE("Default I2C speed: %d KHz\n", (CLK_FREQ/current_speed));

    /*Start a timer for POLL_TIMER seconds*/

    if ( WICED_SUCCESS == wiced_init_timer( &seconds_timer, &comboread_cb, 0, WICED_MILLI_SECONDS_PERIODIC_TIMER )) {
            if ( WICED_SUCCESS != wiced_start_timer( &seconds_timer, 500 )) {
                WICED_BT_TRACE( "Seconds Timer Error\n\r" );
            }
        }

}

/******************************************************************************
 * This function reads the value from I2C slave and prints it
 *****************************************************************************/

void comboread_cb (uint32_t arg)
{
    UINT8  status = 0xFF;
    UINT8 reg_add = 0x28; // Acceleromter register

    typedef struct {
        int16_t ax;
        int16_t ay;
        int16_t az;
    } __attribute__((packed)) accel_val_t;

    accel_val_t data;
    status = wiced_hal_i2c_combined_read((UINT8 *)&reg_add, sizeof(UINT8), (uint8_t *)&data, sizeof(data), LSM9DS1_ACC_GYRO_I2C_ADDRESS);

    if(I2CM_SUCCESS == status) {

        WICED_BT_TRACE("Ax=%d Ay=%d Az=%d\n",data.ax,data.ay,data.az);
    }else if(I2CM_OP_FAILED == status) {
        WICED_BT_TRACE("I2C comboread operation failed\r\n");
    }else if(I2CM_BUSY == status) {
        WICED_BT_TRACE("I2C busy\r\n");
    }else{
        WICED_BT_TRACE("Unknown status from I2C\r\n");
    }

}

/******************************************************************************
 * This function toggles the I2C speed between 100 KHz and 400 KHz
 *****************************************************************************/
void button_cb (void* user_data, uint8_t value )
{
    if(I2CM_SPEED_100KHZ == current_speed)
    {
        wiced_hal_i2c_set_speed(I2CM_SPEED_400KHZ);
        current_speed = wiced_hal_i2c_get_speed();
        WICED_BT_TRACE("Current I2C speed: %d KHz\n", CLK_FREQ/current_speed);
    }
    else
    {
        wiced_hal_i2c_set_speed(I2CM_SPEED_100KHZ);
        current_speed = wiced_hal_i2c_get_speed();
        WICED_BT_TRACE("Current I2C speed: %d KHz\n", CLK_FREQ/current_speed);
    }

}
