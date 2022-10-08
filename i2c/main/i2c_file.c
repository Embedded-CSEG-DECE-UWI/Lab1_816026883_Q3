/* I2C example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_err.h"

#include "driver/i2c.h"


static const char *TAG = "main";

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

#define I2C_EXAMPLE_MASTER_SCL_IO           2                /*!< gpio number for I2C master clock */
#define I2C_EXAMPLE_MASTER_SDA_IO           0                /*!< gpio number for I2C master data  */
#define I2C_EXAMPLE_MASTER_NUM              I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE   0                /*!< I2C master do not need buffer */
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE   0                /*!< I2C master do not need buffer */

#define ads1115_SENSOR_ADDR                 0x48             /*!< slave address for ads1115 sensor */
//#define ads1115_CMD_START                   0x41             /*!< Command to set measure mode */
//#define ads1115_WHO_AM_I                    0x75             /*!< Command to read WHO_AM_I reg */
#define WRITE_BIT                           I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                            I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                        0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                       0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                             0x0              /*!< I2C ack value */
#define NACK_VAL                            0x1              /*!< I2C nack value */
#define LAST_NACK_VAL                       0x2              /*!< I2C last_nack value */

static esp_err_t i2c_master_ads1115_read_string(i2c_port_t i2c_num, uint8_t reg_addr, uint16_t *data)
///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////INITIALISE////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////


static esp_err_t i2c_master_init()
{
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    conf.sda_pullup_en = 1;
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    conf.scl_pullup_en = 1;
    conf.clk_stretch_tick = 300; // 300 ticks, Clock stretch is about 210us, you can make changes according to the actual situation.
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));

    return ESP_OK;
}

static esp_err_t i2c_master_ads1115_init(i2c_port_t i2c_num)
{
    uint8_t cmd_data;
    vTaskDelay(100 / portTICK_RATE_MS);
    i2c_master_init();
    
    ESP_ERROR_CHECK( i2c_master_ads1115_write_string(i2c_num, 0x01 , 0b1000010010000011 ));
    //THe binary string represents the configuration bits given in the quickstart section of the ads1115 datasheet

    return ESP_OK;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////INITIALISE////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////READ/////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

static esp_err_t i2c_master_ads1115_read_bits(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ads1115_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) 
    {   
        printf("An error has occurred while reading in line 104");
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ads1115_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    for(int i=0;i < data_len; i++)
    {
        printf("The byte at position");
        printf(i);
        printf("is");
        printf(  data[i] );
    }

    return ret;
}

static esp_err_t i2c_master_ads1115_read_string(i2c_port_t i2c_num, uint8_t reg_addr, uint16_t *data)
{
    esp_err_t ret;
    uint16_t sensor_data = 0;
    uint8_t read_buff[2];

    ret = i2c_master_ads1115_read_bits(i2c_num, reg_addr, read_buff, 2);
    sensor_data = (read_buff[0] << 8) | read_buff[1];
    *data = sensor_data;
    return ret;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////READ/////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////WRITE/////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////


static esp_err_t i2c_master_ads1115_write_bits(i2c_port_t i2c_num, uint8_t reg_address, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ads1115_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_write(cmd, data, data_len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t i2c_master_ads1115_write_string(i2c_port_t i2c_num, uint8_t reg_addr, uint16_t data)
{
    esp_err_t ret;
    uint8_t write_buff[2];

    write_buff[0] = (data >> 8) & 0xFF;
    write_buff[1] = (data >> 0) & 0xFF;

    ret = i2c_master_ads1115_write_bits(i2c_num, reg_addr, write_buff, 2);

    return ret;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////WRITE ///////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////MAIN ///////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////


static void i2c_task(void *arg)
{
    uint16_t sensor_data; //data is in the form of two 8 bit registers
    //uint8_t who_am_i, i;
    double asignal;
    //static uint32_t error_count = 0;
    int ret;
    printf("Simple comment before starting any actual adc or i2c stuff ");

    i2c_master_ads1115_init(I2C_EXAMPLE_MASTER_NUM);

    while (1) {

        ret = i2c_master_ads1115_read_string(I2C_EXAMPLE_MASTER_NUM, 0x00 , &sensor_data);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Succesfully read value from from ADC \n");

            printf("The return from the ADC is ");
            printf(int(ret));
            printf("\n");


            ESP_LOGI(TAG, "Sensor Data: %d\n", (int)sensor_data);
            asignal = (double) sensor_data * 1.25e-4;
            ESP_LOGI(TAG,"Voltage: %d.%d V\n",(uint16_t)asignal, (uint16_t)(asignal * 100) % 100);
            
            vTaskDelay(1000 / portTICK_RATE_MS);
        } 
        else 
        {
            ESP_LOGE(TAG, "No acknowledge bit, sensor not connected...skip...\n");
        }

        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    //i2c_driver_delete(I2C_EXAMPLE_MASTER_NUM);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void app_main(void)
{
    //start i2c task
    xTaskCreate(i2c_task, "i2c_task", 2048, NULL, 10, NULL);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////MAIN ///////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
