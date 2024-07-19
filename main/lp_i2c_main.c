#include <stdio.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "lp_core_main.h"
#include "ulp_lp_core.h"
#include "lp_core_i2c.h"

#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <unistd.h>

#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "esp_vfs_fat.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#define filename "/data/device1.csv"
#define measure_period 5000000 // time between two measurements in us (10s)

int year, month, day, hour, minute, second;

void i2c_recover(void);
void i2c_send_config(void);
int year, month, day, hour, minute, second;
extern const uint8_t lp_core_main_bin_start[] asm("_binary_lp_core_main_bin_start");
extern const uint8_t lp_core_main_bin_end[] asm("_binary_lp_core_main_bin_end");
uint64_t fail_count = 0;
i2c_master_bus_handle_t bus_handle;
i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = 0,
    .scl_io_num = 7,
    .sda_io_num = 6,
    .glitch_ignore_cnt = 2,
    .flags.enable_internal_pullup = true,
};
i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = 0x42,
    .scl_speed_hz = 20000,
    .scl_wait_us = 1UL << 32};

i2c_master_dev_handle_t dev_handle;

esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = 1,
    .max_files = 5,
    .allocation_unit_size = 16 * 1024};
sdmmc_card_t *card;

sdmmc_host_t host = SDSPI_HOST_DEFAULT();

spi_bus_config_t bus_cfg = {
    .mosi_io_num = GPIO_NUM_15,
    .miso_io_num = GPIO_NUM_8,
    .sclk_io_num = GPIO_NUM_21,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4000,
};
void lp_core_init(void)
{
    esp_err_t ret = ESP_OK;

    ulp_lp_core_cfg_t cfg = {
        .wakeup_source = ULP_LP_CORE_WAKEUP_SOURCE_LP_TIMER,
        .lp_timer_sleep_duration_us = (uint32_t)measure_period,
    };

    ret = ulp_lp_core_load_binary(lp_core_main_bin_start, (lp_core_main_bin_end - lp_core_main_bin_start));
    if (ret != ESP_OK)
    {
        printf("LP Core load failed\n");
        abort();
    }

    ret = ulp_lp_core_run(&cfg);
    if (ret != ESP_OK)
    {
        printf("LP Core run failed\n");
        abort();
    }

    printf("LP core loaded with firmware successfully\n");
}
int sd_create_file(void)
{
    if (access(filename, F_OK) == 0)
    {
        FILE *fp = fopen(filename, "r");
        if (fp == NULL)
        {
            printf("NULL file pointer\n");
            return -1;
        }
        fclose(fp);
        printf("FILE EXISTS\n");
        // file exists
        return 1;
    }
    else
    {
        FILE *fp = fopen(filename, "w");
        if (fp == NULL)
        {
            printf("COULD NOT CREATE FILE\n");
            return 0;
        }
        printf("file created\n");
        printf("%d\n", fprintf(fp, "DEVICE %d\n", 1));
        int ret = fwrite("YEAR,MONTH,DAY,HOUR,MINUTE,SECOND,LAT(10^-7),LON(10^-7)\n", 57U, 1, fp);
        fclose(fp);
        return ret;
    }
}
// unused method
void read_file(void)
{
    FILE *fp = fopen(filename, "w");
    if (fp == NULL)
    {
        printf("Could not open file for reading\n");
        return;
    }
    printf("\n%d\n", fwrite("YEAR,MONTH,DAY,HOUR,MINUTE,SECOND,LAT(10^-7),LON(10^-7)\n", 57U, 1, fp));
    fflush(fp);
    fseek(fp, 0, 0);
    char c = fgetc(fp);

    printf("%c\n", c);
    fclose(fp);
}
int store_readings(void)
{
    FILE *fp = fopen(filename, "a");
    if (fp == NULL)
    {
        printf("NULL file pointer\n");
        return -1;
    }
    int ret = fprintf(fp, "%d,%d,%d,%d,%d,%d,%ld,%ld\n", year, month, day, hour, minute, second, ulp_lat, ulp_lon);

    fclose(fp);
    return ret;
}
void sd_init(void)
{
    host.max_freq_khz = 1000;
    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK)
    {
        printf("FAILED TO INITIALIZE SPI\n");
    }
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = GPIO_NUM_23;
    slot_config.host_id = host.slot;
    ret = esp_vfs_fat_sdspi_mount("/data", &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            printf("Failed to mount filesystem.\n");
        }
        else
        {
            printf("Failed to initialize the card (). \n");
        }
    }
    else
    {
        printf("card initialized succesfully\n");
    }
}
static void lp_i2c_init(void)
{
    esp_err_t ret = ESP_OK;

    /* Initialize LP I2C with default configuration */
    lp_core_i2c_cfg_t i2c_cfg = LP_CORE_I2C_DEFAULT_CONFIG();
    i2c_cfg.i2c_timing_cfg.clk_speed_hz = 20000UL;

    ret = lp_core_i2c_master_init(LP_I2C_NUM_0, &i2c_cfg);
    if (ret != ESP_OK)
    {
        printf("LP I2C init failed\n");
        abort();
    }

    printf("LP I2C initialized successfully\n");
}
void app_main(void)
{
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_config_t ulp_pin_cfg;
    ulp_pin_cfg.mode = GPIO_MODE_OUTPUT;
    ulp_pin_cfg.pin_bit_mask = (1ULL << 3) | (1ULL << 4);
    ulp_pin_cfg.intr_type = 0;
    gpio_config(&ulp_pin_cfg);

    gpio_set_direction(GPIO_NUM_3, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);

    gpio_set_level(GPIO_NUM_3, 0);
    //  gpio_hold_en(GPIO_NUM_3);
    /* Setup wakeup triggers */
    // keep rtc_io on during sleep.
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    rtc_gpio_init(GPIO_NUM_2);
    rtc_gpio_set_direction(GPIO_NUM_2, GPIO_MODE_INPUT);
    rtc_gpio_pulldown_en(GPIO_NUM_2);
    ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup_io(1ULL << 2, ESP_EXT1_WAKEUP_ANY_HIGH));
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause != 3)
    {

        i2c_send_config();
        //   printf("Num bytes available = %ld\n", );
        printf("Not an LP core wakeup. Cause = %d\n", cause);
        printf("Initializing...\n");
        /* Initialize LP_I2C from the main processor */
        lp_i2c_init();

        /* Load LP Core binary and start the coprocessor */
        lp_core_init();
        sd_init();
        if (sd_create_file() != 1)
            printf("initial file check failed\n");
    }
    else if (cause == 3)
    {

        printf("LP core woke up the main CPU\n");
        if (ulp_numFails > 20)
        {
            printf("I2C lockout : attempting recovery TRANSACTION ");
            // vTaskDelay(5000 / portTICK_PERIOD_MS);
            i2c_recover();
            i2c_send_config();
            esp_restart();
        }
        else if (ulp_lat != 0)
        {
            // convert date and time to big endian (I had already written the bit operations considering it was big end but it was little endina)

            printf("FIX FOUND: lat: %ld lon: %ld date: %ld time: %ld\n", ulp_lat, ulp_lon, ulp_date, ulp_time);

            year = ulp_date & 0x0000FFFF;
            month = ulp_date >> 16 & 0x000000FF;
            day = ulp_date >> 24;
            hour = ulp_time >> 8 & 0x000000FF;
            minute = ulp_time >> 16 & 0x000000FF;
            second = ulp_time >> 24 & 0x000000FF;
            // remount sd card before storing readings
            sd_init();
            if (store_readings() > 0)
                printf("data written successfully\n");
            else
                printf("data NOT written successfully\n");
        }
    }

    /* Enter Deep Sleep */

    vTaskDelay(200 / portTICK_PERIOD_MS);
    printf("Entering deep sleep...\n");

    esp_deep_sleep_start();
}

void i2c_send_config()
{
    gpio_set_level(GPIO_NUM_4, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
    // turn off NMEA messages on i2c
    uint8_t NMEAOFFRAM[20] = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x01, 0x01, 0x00, 0x00, 0x02, 0x00, 0x72, 0x10, 0x00, 0x1F, 0xBA};
    uint8_t NMEAOFFBBR[20] = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x01, 0x02, 0x00, 0x00, 0x02, 0x00, 0x72, 0x10, 0x00, 0x20, 0xC2};
    // set dynmaioc sanity check model to pedestrian
    uint8_t DYNMODELRAM[] = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x01, 0x01, 0x00, 0x00, 0x21, 0x00, 0x11, 0x20, 0x03, 0xF0, 0x55};
    uint8_t DYNMODELBBR[] = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x01, 0x02, 0x00, 0x00, 0x21, 0x00, 0x11, 0x20, 0x03, 0xF1, 0x5D};

    // set output and time accuracy,0xfilter,0xto,0x3m
    uint8_t OUTMASKRAM[] = {0xB5, 0x62, 0x06, 0x8A, 0x0A, 0x00, 0x01, 0x01, 0x00, 0x00, 0xB3, 0x00, 0x11, 0x30, 0x1e, 0x00, 0xae, 0x23};
    uint8_t OUTMASKBBR[] = {0xB5, 0x62, 0x06, 0x8A, 0x0A, 0x00, 0x01, 0x02, 0x00, 0x00, 0xB3, 0x00, 0x11, 0x30, 0x1e, 0x00, 0xaf, 0x2c};
    uint8_t TIMEMASKRAM[] = {0xB5, 0x62, 0x06, 0x8A, 0x0A, 0x00, 0x01, 0x01, 0x00, 0x00, 0xB4, 0x00, 0x11, 0x30, 0x64, 0x00, 0xF5, 0xb5};
    uint8_t TIMEMASKBBR[] = {0xB5, 0x62, 0x06, 0x8A, 0x0A, 0x00, 0x01, 0x02, 0x00, 0x00, 0xB4, 0x00, 0x11, 0x30, 0x64, 0x00, 0xf6, 0xbe};
    // set lna mode to low gain
    uint8_t LNAMODERAM[] = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x01, 0x01, 0x00, 0x00, 0x57, 0x00, 0xA3, 0x20, 0x01, 0xB6, 0x17};
    uint8_t LNAMODEBBR[] = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x01, 0x02, 0x00, 0x00, 0x57, 0x00, 0xA3, 0x20, 0x01, 0xB7, 0x1F};
    uint8_t PDOP_RAM[] = {0xB5, 0x62, 0x06, 0x8A, 0x0A, 0x00, 0x01, 0x01, 0x00, 0x00, 0xB1, 0x00, 0x11, 0x30, 0x32, 0x00, 0xC0, 0x3F};
    uint8_t PDOP_BBR[] = {0xB5, 0x62, 0x06, 0x8A, 0x0A, 0x00, 0x01, 0x02, 0x00, 0x00, 0xB1, 0x00, 0x11, 0x30, 0x32, 0x00, 0xC1, 0x48};
    uint8_t MIN_ELEV_RAM[] = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x01, 0x02, 0x00, 0x00, 0xA4, 0x00, 0x11, 0x20, 0x14, 0x85, 0xFD};
    uint8_t MIN_ELEV_BBR[] = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x01, 0x01, 0x00, 0x00, 0xA4, 0x00, 0x11, 0x20, 0x14, 0x84, 0xF5};
    uint8_t SIGATT_COMP_RAM[] = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x01, 0x01, 0x00, 0x00, 0xD6, 0x00, 0x11, 0x20, 0x00, 0xA2, 0xDB};
    uint8_t SIGATT_COMP_BBR[] = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x01, 0x02, 0x00, 0x00, 0xD6, 0x00, 0x11, 0x20, 0x00, 0xA3, 0xE3};
    uint8_t SBAS_SCAN_RAM[] = {0xB5, 0x62, 0x06, 0x8A, 0x10, 0x00, 0x01, 0x01, 0x00, 0x00, 0x06, 0x00, 0x36, 0x50, 0x00, 0xA8, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDA, 0xD9};
    uint8_t SBAS_SCAN_BBR[] = {0xB5, 0x62, 0x06, 0x8A, 0x10, 0x00, 0x01, 0x02, 0x00, 0x00, 0x06, 0x00, 0x36, 0x50, 0x00, 0xA8, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDB, 0xE8};
    uint8_t UBX_OUT_DISABLE[] = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x72, 0x10, 0x00, 0x1E, 0xB5};
    uint8_t NMEA_OUT_DISABLE[] = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x01, 0x01, 0x00, 0x00, 0x02, 0x00, 0x72, 0x10, 0x00, 0x1F, 0xBA};
    uint8_t I2C_TIMEOUT_DISABLE_RAM[] = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x01, 0x01, 0x00, 0x00, 0x02, 0x00, 0x51, 0x10, 0x01, 0xFF, 0x5C};
    uint8_t I2C_TIMEOUT_DISABLE_BBR[] = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x01, 0x02, 0x00, 0x00, 0x02, 0x00, 0x51, 0x10, 0x01, 0x00, 0x64};
    // send config to GPS module (keep repeating each individual transaction until its successful)
    // uint32_t *timeout_reg = 0x3ff5300C;
    // uint32_t *scl_st_to = 0x3ff53098;
    // uint32_t *scl_main_st_to = 0x3ff5309C;
    //*timeout_reg &= 0xFE000000;
    //*scl_st_to |= 0x00FFFFFF;
    //*scl_main_st_to |= 0x00FFFFFF;
    while (i2c_master_transmit(dev_handle, NMEAOFFRAM, sizeof(NMEAOFFRAM), 1000) != ESP_OK)
    {
        fail_count++;
        if (fail_count > 20)
            i2c_recover();
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
    while (i2c_master_transmit(dev_handle, NMEAOFFBBR, sizeof(NMEAOFFBBR), 1000) != ESP_OK)
    {
        fail_count++;
        if (fail_count > 20)
            i2c_recover();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    while (i2c_master_transmit(dev_handle, DYNMODELRAM, sizeof(DYNMODELRAM), 1000) != ESP_OK)
    {
        fail_count++;
        if (fail_count > 20)
            i2c_recover();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    while (i2c_master_transmit(dev_handle, DYNMODELBBR, sizeof(DYNMODELBBR), 1000) != ESP_OK)
    {
        fail_count++;
        if (fail_count > 20)
            i2c_recover();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    while (i2c_master_transmit(dev_handle, OUTMASKBBR, sizeof(OUTMASKBBR), 1000) != ESP_OK)
    {
        fail_count++;
        if (fail_count > 20)
            i2c_recover();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    while (i2c_master_transmit(dev_handle, OUTMASKRAM, sizeof(OUTMASKRAM), 1000) != ESP_OK)
    {
        fail_count++;
        if (fail_count > 20)
            i2c_recover();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    while (i2c_master_transmit(dev_handle, TIMEMASKBBR, sizeof(TIMEMASKBBR), 1000) != ESP_OK)
    {
        fail_count++;
        if (fail_count > 20)
            i2c_recover();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    while (i2c_master_transmit(dev_handle, TIMEMASKRAM, sizeof(TIMEMASKRAM), 1000) != ESP_OK)
    {
        fail_count++;
        if (fail_count > 20)
            i2c_recover();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    while (i2c_master_transmit(dev_handle, LNAMODEBBR, sizeof(LNAMODEBBR), 1000) != ESP_OK)
    {
        fail_count++;
        if (fail_count > 20)
            i2c_recover();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    while (i2c_master_transmit(dev_handle, LNAMODERAM, sizeof(LNAMODERAM), 1000) != ESP_OK)
    {
        fail_count++;
        if (fail_count > 20)
            i2c_recover();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    while (i2c_master_transmit(dev_handle, PDOP_RAM, sizeof(PDOP_RAM), 1000) != ESP_OK)
    {
        fail_count++;
        if (fail_count > 20)
            i2c_recover();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    while (i2c_master_transmit(dev_handle, PDOP_BBR, sizeof(PDOP_BBR), 1000) != ESP_OK)
    {
        fail_count++;
        if (fail_count > 20)
            i2c_recover();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    while (i2c_master_transmit(dev_handle, MIN_ELEV_BBR, sizeof(MIN_ELEV_BBR), 1000) != ESP_OK)
    {
        fail_count++;
        if (fail_count > 20)
            i2c_recover();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    while (i2c_master_transmit(dev_handle, MIN_ELEV_RAM, sizeof(MIN_ELEV_RAM), 1000) != ESP_OK)
    {
        fail_count++;
        if (fail_count > 20)
            i2c_recover();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    while (i2c_master_transmit(dev_handle, SIGATT_COMP_BBR, sizeof(SIGATT_COMP_BBR), 1000) != ESP_OK)
    {
        fail_count++;
        if (fail_count > 20)
            i2c_recover();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    while (i2c_master_transmit(dev_handle, SIGATT_COMP_RAM, sizeof(SIGATT_COMP_RAM), 1000) != ESP_OK)
    {
        fail_count++;
        if (fail_count > 20)
            i2c_recover();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    while (i2c_master_transmit(dev_handle, SBAS_SCAN_RAM, sizeof(SBAS_SCAN_RAM), 1000) != ESP_OK)
    {
        fail_count++;
        if (fail_count > 20)
            i2c_recover();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    while (i2c_master_transmit(dev_handle, SBAS_SCAN_BBR, sizeof(SBAS_SCAN_BBR), 1000) != ESP_OK)
    {
        fail_count++;
        if (fail_count > 20)
            i2c_recover();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    while (i2c_master_transmit(dev_handle, UBX_OUT_DISABLE, sizeof(UBX_OUT_DISABLE), 1000) != ESP_OK)
    {
        fail_count++;
        if (fail_count > 20)
            i2c_recover();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    while (i2c_master_transmit(dev_handle, NMEA_OUT_DISABLE, sizeof(NMEA_OUT_DISABLE), 1000) != ESP_OK)
    {
        fail_count++;
        if (fail_count > 20)
            i2c_recover();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    while (i2c_master_transmit(dev_handle, I2C_TIMEOUT_DISABLE_RAM, sizeof(I2C_TIMEOUT_DISABLE_RAM), 1000) != ESP_OK)
    {
        fail_count++;
        if (fail_count > 20)
            i2c_recover();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    while (i2c_master_transmit(dev_handle, I2C_TIMEOUT_DISABLE_BBR, sizeof(I2C_TIMEOUT_DISABLE_BBR), 1000) != ESP_OK)
    {
        fail_count++;
        if (fail_count > 20)
            i2c_recover();
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    printf("config succesfully sent\n");

    // delete this i2c bus as all reads/writes will now be done by the ULP Co-processor using LP_I2C peripheral
    while (i2c_del_master_bus(bus_handle) != ESP_OK)
        ;
    printf("Main CPU i2c bus deleted succesfully\n");
    gpio_set_level(GPIO_NUM_4, 0);
}

void i2c_recover()
{

    // printf("trying recovery\n");
    // uint8_t main_state = (uint8_t)(*(uint32_t *)(0x3FF53008) & 0x0700) >> 24;
    // uint8_t scl_state = (uint8_t)(*(uint32_t *)(0x3FF53008) & 0x7000) >> 28;
    // uint8_t bus_state = (uint8_t)(*(uint32_t *)(0x3FF53008) & 0x0010) >> 4;
    // printf("MAIN state %d , SCL State %d, Bus State %d\n", main_state, scl_state, bus_state);
    //  vTaskDelay(5000 / portTICK_PERIOD_MS);
    gpio_config_t configg;
    configg.pin_bit_mask = 1ULL << 6 | 1ULL << 7;
    configg.mode = GPIO_MODE_OUTPUT; // gpio_config(&configg);
    for (int i = 0; i < 9; i++)
    {
        gpio_set_level(7, 0);
        if (i == 7)
            gpio_set_level(6, 0);
        if (i == 8)
            gpio_set_level(6, 1);
        vTaskDelay(0.01 / portTICK_PERIOD_MS);
        gpio_set_level(7, 1);
        vTaskDelay(0.01 / portTICK_PERIOD_MS);
    }
    fail_count = 0;

    configg.mode = GPIO_MODE_DISABLE;
    gpio_config(&configg);
    // ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    // ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    // ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    // ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
    // ESP_ERROR_CHECK(i2c_master_bus_reset(bus_handle));

    // uint32_t *timeout_reg = 0x3ff5300C;
    // uint32_t *scl_st_to = 0x3ff53098;
    // uint32_t *scl_main_st_to = 0x3ff5309C;
    //*timeout_reg &= 0xFE000000;
    //*scl_st_to |= 0x00FFFFFF;
    //*scl_main_st_to |= 0x00FFFFFF;
    //  printf("tried recovery\n");
}
