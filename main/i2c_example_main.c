/* Measurue flux and convert it to 4..20mA with pwm
 *
 *
  https://github.com/espressif/esp-rainmaker/blob/master/examples/led_light
  https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/api-reference/peripherals/index.html


*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "math.h"
#include "esp_log.h"
#include <string.h>

#include "esp_system.h"
#include "esp_spi_flash.h"
#include "hal/gpio_types.h"
#include "driver/i2c.h"


////__________________________________________ I2C
// SDA groen
// SCL geel
#define LOG_I2C "I2C"
#define I2C_MASTER_SCL_IO 1               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 0               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 400000        /*!< I2C master clock frequency make 380kHz*/
//#define I2C_MASTER_FREQ_HZ 400000        /*!< I2C master clock frequency make 380kHz*/
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */


esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    uint32_t pinreg = READ_PERI_REG(GPIO_PIN1_REG);
    ESP_LOGI(LOG_I2C, "PIN 1 %x",pinreg);
    gpio_set_drive_capability(I2C_MASTER_SCL_IO,GPIO_DRIVE_CAP_3 );
//    SET_PERI_REG_BITS(GPIO_PIN_REG[gpio_num], FUN_DRV_V, strength, FUN_DRV_S);
//    SET_PERI_REG_BITS(GPIO_PIN1_REG, GPIO_PIN1_PAD_DRIVER, 0, GPIO_PIN1_PAD_DRIVER_S);
    WRITE_PERI_REG(GPIO_PIN1_REG, 0); // need output not pulldown
    pinreg = READ_PERI_REG(GPIO_PIN1_REG);
    ESP_LOGI(LOG_I2C, "PIN 1 %x",pinreg);
    return ESP_OK;
}




//__________________________________________PWM
#include "driver/ledc.h"
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (5) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz

void ledc_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
}

void ledc_pwm(float duty){
    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ledc_channel.duty=(uint32_t)(duty *((1<<13) - 1));
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}


int SI7210_init(void);
int SI7210_read(void);
void data_uart_task(void *arg);



//___________________________________________ MAIN
void data_uart_send_string(char *s);

//void MCP3421_init(int gain);
//float MCP3421_value(void);
//void MCP3426_init(int gain);
//float MCP3426_value(void);
void MCP3426_task(void *arg);

void app_main(void){

//	led_strip.rgb_led_type = RGB_LED_TYPE_WS2812;
    printf("\nCurrent Sense \n");
    printf("Minimum free heap size cool: %d bytes\n", esp_get_minimum_free_heap_size());
    ledc_init();
    i2c_master_init();
//    xTaskCreatePinnedToCore(task, "test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
    xTaskCreate(data_uart_task, "data uart", 8000, NULL, 10, NULL);
    xTaskCreate(MCP3426_task, "MCP3426", 8000, NULL, 10, NULL);
    vTaskDelay(98);
//    SI7210_init();
    float duty, phase =0;
//    MCP3421_init(0);
    while (true) {
    	phase += 0.1; duty=0.5+0.5*sin(phase);
    	ledc_pwm(duty);
        vTaskDelay(98);
//        gauss=SI7210_read();
        vTaskDelay(2);
//        MCP3421_value();
//        printf("gauss %d\n",gauss);
    }

}
