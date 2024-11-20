/*****************************************************************//**
 * @file main_sampler_test.cpp
 *
 * @brief Basic test of nexys4 ddr mmio cores
 *
 * @author p chu
 * @version v1.0: initial release
 *********************************************************************/

// #define _DEBUG
#include "chu_init.h"
#include "gpio_cores.h"
#include "xadc_core.h"
#include "sseg_core.h"
#include "spi_core.h"
#include "i2c_core.h"
#include "ps2_core.h"
#include "ddfs_core.h"
#include "adsr_core.h"
#include <math.h>

//Linux Terminal Colors...
#define RESET "\033[0m"       //Reset color...
#define RED "\033[31m"        //Red color...
#define GREEN "\033[32m"      //Green color...
#define YELLOW "\033[33m"     //Yellow color...
#define CYAN "\033[36m"       // yan color...

GpoCore led(get_slot_addr(BRIDGE_BASE, S2_LED));
SsegCore sseg(get_slot_addr(BRIDGE_BASE, S8_SSEG));
SpiCore spi(get_slot_addr(BRIDGE_BASE, S9_SPI));

void double_to_sseg(SsegCore *sseg, double adc_voltage) {
  // Turn off unneeded SSeg displays (positions 0–3)
  for (int i = 3; i >= 0; --i)
    sseg->write_1ptn(0xff, i); // Active LOW

  // Set the decimal point: Place it between the 6th and 7th digits
  sseg->set_dp(
      0b01000000); // Binary representation for decimal point at position 6

  // Extract integer and fractional parts of adc_voltage
  int integer_part = (int)adc_voltage; // Whole number part
  int fractional_part =
      (int)((adc_voltage - integer_part) * 1000); // First three decimal places

  // Display integer part (7th and 6th positions)
  uint8_t upper_disp_val =
      sseg->h2s(integer_part / 10); // Tens place of integer part
  sseg->write_1ptn(upper_disp_val, 7);

  uint8_t lower_disp_val =
      sseg->h2s(integer_part % 10); // Ones place of integer part
  sseg->write_1ptn(lower_disp_val, 6);

  // Display fractional part (5th, 4th, and 3rd positions)
  uint8_t frac_first = sseg->h2s((fractional_part / 100) % 10); // Tenths place
  sseg->write_1ptn(frac_first, 5);

  uint8_t frac_second =
      sseg->h2s((fractional_part / 10) % 10); // Hundredths place
  sseg->write_1ptn(frac_second, 4);
}


double map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main() {
    //SPI setup and data acquisition/conversion variables...
    const uint8_t RD_CMD = 0x0b;
    const uint8_t DATA_REG = 0x08;
    const float raw_max = 127.0 / 2.0; //Ratio to convert raw data (binary register) to real accelerometer values...

    //Calculation variables... 
    float magnitude_old = 0;
    float delta;
    const float threshold = 0.05;

    while (1) {
        //Tapping logic, acquiring data from SPI accelerometer...
        int8_t xraw, yraw, zraw;
        float x, y, z, magnitude;
        spi.set_freq(400000);
        spi.set_mode(0, 0);

        //Read x/y/z raw data...
        spi.assert_ss(0);
        spi.transfer(RD_CMD);
        spi.transfer(DATA_REG);
        xraw = spi.transfer(0x00);
        yraw = spi.transfer(0x00);
        zraw = spi.transfer(0x00);
        spi.deassert_ss(0);

        //Application calculations...
        x = fabs((float)xraw / raw_max);
        y = fabs((float)yraw / raw_max);
        z = fabs((float)zraw / raw_max);
        magnitude = x + y + z;
        delta = magnitude - magnitude_old;

        //Update magnitude old within current cycle...
        if (magnitude != magnitude_old)
            magnitude_old = magnitude;

        if (delta > threshold) {
            //Map the delta to the LED range...
            int LED = (int)map(delta, 0.0, 0.5, 0.0, 15.0);

            //Set bounds for the LED range within our range of LEDs on the hardware...
            if (LED < 0) LED = 0;
            if (LED > 15) LED = 15;

            //Write LED values to LED core...
            for (int i = 0; i <= LED; i++) {
                led.write(1, i);
                double_to_sseg(&sseg, delta);
                sleep_ms(10); 
            }

            //Display output to UART...
            uart.disp(CYAN "Magnitude: " RESET);
            uart.disp(magnitude, 3);
            uart.disp( " " YELLOW "Delta: " RESET);
            uart.disp(delta, 3);
            uart.disp( " " GREEN "LED value: " RESET);
            uart.disp(LED, 3);
            uart.disp("\n\r");

            sleep_ms(15); 
            led.write(0);
        }
    } // while loop

    return 0;
}

