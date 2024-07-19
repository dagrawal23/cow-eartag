#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "lp_core_i2c.h"
#include "ulp_lp_core_i2c.h"
#include "ulp_lp_core_utils.h"
#include "ulp_lp_core_gpio.h"
uint32_t numFails;
uint32_t lat;
uint32_t lon;
uint32_t date; // 16 MSB for year 8 LSB for day, middle 8 bits for month (UTC) : YYYY_YYYY_YYYY_YYYY_MMMM_MMMM_DDDD_DDDD
uint32_t time;
//  uint32_t transactions; // 8 MSBs dont care, 8 bits each from right to left for Hour, Min, Sec: XXXX_XXXX_HHHH_HHHH_MMMM_MMMM_SSSS_SSSS
int main(void)
{

    // transactions = 0;
    lat = 0;
    lon = 0;
    uint32_t num_bytes_to_read;
    numFails = 0;
    ulp_lp_core_gpio_init(LP_IO_NUM_4);
    ulp_lp_core_gpio_output_enable(LP_IO_NUM_4);
    ulp_lp_core_gpio_set_output_mode(LP_IO_NUM_4, RTCIO_OUTPUT_NORMAL);
    ulp_lp_core_gpio_set_level(LP_IO_NUM_4, 1);

    // ulp_lp_core_gpio_init(LP_IO_NUM_3);
    // ulp_lp_core_gpio_output_enable(LP_IO_NUM_3);
    // ulp_lp_core_gpio_set_level(LP_IO_NUM_3, 0);
    // ulp_lp_core_gpio_set_output_mode(LP_IO_NUM_3, RTCIO_OUTPUT_NORMAL);
    ulp_lp_core_gpio_init(LP_IO_NUM_3);
    ulp_lp_core_gpio_output_enable(LP_IO_NUM_3);
    ulp_lp_core_gpio_set_level(LP_IO_NUM_3, 0);
    ulp_lp_core_gpio_set_output_mode(LP_IO_NUM_3, RTCIO_OUTPUT_NORMAL);
    num_bytes_to_read = 0;
    uint8_t UBX_NAV_PVT[] = {0xb5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x08, 0x19};
    uint8_t REG_NUM_BYTES_TO_READ = 0xFD;
    uint8_t num_bytes[2];
    uint8_t read[100];
    ulp_lp_core_delay_us(1500000);
    while (1)
    {
        num_bytes_to_read = 0;
        num_bytes[0] = 0;
        num_bytes[1] = 0;
        // poll UBX_NAV_PVT
        while (lp_core_i2c_master_write_to_device(1, (uint16_t)0x42, UBX_NAV_PVT, 8U, (int32_t)5000) != ESP_OK)
        {
            numFails++;
            if (numFails > 20)
            {

                ulp_lp_core_gpio_set_level(LP_IO_NUM_4, 0);
                ulp_lp_core_gpio_set_level(LP_IO_NUM_3, 1);
                ulp_lp_core_delay_us(1000);
                ulp_lp_core_gpio_set_level(LP_IO_NUM_3, 0);
                return 0;
            }
        }
        // transactions++;
        // numFails = 0;
        ulp_lp_core_delay_us(500000);
        // check number of bytes available to read
        while (lp_core_i2c_master_write_read_device(1, 0x42, &REG_NUM_BYTES_TO_READ, 1U, num_bytes, 2U, 5000) != ESP_OK)
        {
            numFails++;
            if (numFails > 20)
            {
                lat = 0;
                lon = 0;

                ulp_lp_core_gpio_set_level(LP_IO_NUM_4, 0);
                ulp_lp_core_gpio_set_level(LP_IO_NUM_3, 1);
                ulp_lp_core_delay_us(1000);
                ulp_lp_core_gpio_set_level(LP_IO_NUM_3, 0);
                return 0;
            }
        }
        // transactions++;
        //   transactions += 0;
        // numFails = 0;
        num_bytes_to_read = 0U | num_bytes[0] << 8 | num_bytes[1];

        // have available bytes to read and
        if (num_bytes_to_read > 0)
        {
            int bytes_to_read = num_bytes_to_read;
            while (bytes_to_read != 0)
            {
                // clear the read buffer by reading everything in iterations of 100bytes
                while (lp_core_i2c_master_read_from_device(1, 0x42, read, (bytes_to_read > 100) ? 100U : bytes_to_read, 10000) != ESP_OK)
                {
                    numFails++;
                    if (numFails > 20)
                    {

                        lat = 0;
                        lon = 0;
                        ulp_lp_core_gpio_set_level(LP_IO_NUM_4, 0);
                        ulp_lp_core_gpio_set_level(LP_IO_NUM_3, 1);
                        ulp_lp_core_delay_us(1000);
                        ulp_lp_core_gpio_set_level(LP_IO_NUM_3, 0);
                        return 0;
                    }
                }
                // transactions++;
                // transactions += 0;
                // numFails = 0;
                bytes_to_read = bytes_to_read > 100 ? (bytes_to_read - 100) : 0;
            }
            // if exactly 100 bytes were read then it might be a reply to our polling request. check for a fix
            if (num_bytes_to_read == 100)
            {
                uint8_t chksumA = 0;
                uint8_t chksumB = 0;
                for (int i = 2; i < 98; i++)
                {
                    chksumA = chksumA + read[i];
                    chksumB = chksumA + chksumB;
                }

                uint32_t *metadata = read;
                if (chksumA == read[98] && chksumB == read[99] && *metadata == 0x070162b5)
                {
                    if (read[26] == 3)
                    {
                        lat = *((uint32_t *)(&read[34]));
                        lon = *((uint32_t *)(&read[30]));
                        date = *((uint32_t *)(&read[10]));
                        time = *((uint32_t *)(&read[13]));
                        break;
                    }
                }
            }
        }
        // to avoid overworking the i2c bus
        ulp_lp_core_delay_us(1000000);
    }
    ulp_lp_core_gpio_set_level(LP_IO_NUM_4, 0);
    ulp_lp_core_gpio_set_level(LP_IO_NUM_3, 1);
    ulp_lp_core_delay_us(1000);
    ulp_lp_core_gpio_set_level(LP_IO_NUM_3, 0);
    return 0;
}
void lp_i2c_recover()
{
    ulp_lp_core_gpio_init(LP_IO_NUM_7);
    ulp_lp_core_gpio_output_enable(LP_IO_NUM_7);
    ulp_lp_core_gpio_set_output_mode(LP_IO_NUM_7, RTCIO_OUTPUT_OD);
    ulp_lp_core_gpio_set_level(LP_IO_NUM_7, 0);
    for (int i = 0; i < 9; i++)
    {
        ulp_lp_core_gpio_set_level(LP_IO_NUM_7, 0);
        ulp_lp_core_delay_us(3);
        ulp_lp_core_gpio_set_level(LP_IO_NUM_7, 1);
        ulp_lp_core_delay_us(3);
    }
}