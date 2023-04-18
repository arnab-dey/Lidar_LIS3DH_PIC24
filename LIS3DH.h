/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H

#include <xc.h> // include processor files - each processor file is guarded.
#define I2C_WRITE 0
#define I2C_READ 1

#define LIS3DH_ADDRESS 0x18
#define LIS3DH_DATARATE_POWERDOWN (0b0000)
#define LIS3DH_DATARATE_1HZ (0b0001)
#define LIS3DH_DATARATE_10HZ (0b0010)
#define LIS3DH_DATARATE_25HZ (0b0011)
#define LIS3DH_DATARATE_50HZ (0b0100)
#define LIS3DH_DATARATE_100HZ (0b0101)
#define LIS3DH_DATARATE_200HZ (0b0110)
#define LIS3DH_DATARATE_400HZ (0b0111)
#define LIS3DH_DATARATE_LOWPOWER_1_6KHZ (0b1000)
#define LIS3DH_DATARATE_LOWPOWER_5KHZ (0b1001)
#define LIS3DH_RANGE_2G (0b00)
#define LIS3DH_RANGE_4G (0b01)
#define LIS3DH_RANGE_8G (0b10)
#define LIS3DH_RANGE_16G (0b11)

#define STANDARD_GRAVITY (9.806)
#define ADDRESS_AUTO_INCREMENT (0x80)
#define POWER_MODE_LOW 0
#define POWER_MODE_NORMAL 1
#define POWER_MODE_HIGH_RESOLUTION 2
// Register address
#define LIS3DH_REG_WHO_AM_I (0x0F)
#define LIS3DH_REG_CTRL5 (0x24)
#define LIS3DH_REG_CTRL1 (0x20)
#define LIS3DH_REG_CTRL4 (0x23)
#define LIS3DH_REG_TEMPCFG (0x1F)
#define LIS3DH_REG_OUT_X_L (0x28)
#define LIS3DH_REG_OUT_Y_L (0x2A)
#define LIS3DH_REG_OUT_Z_L (0x2C)

void write(const uint8_t *buffer, size_t len, bool stop);
void read(uint8_t *buffer, size_t len, bool stop);
void write_then_read(const uint8_t *write_buffer, size_t write_len,
        uint8_t *read_buffer, size_t read_len, bool stop);
void write8(uint8_t reg, uint32_t value);
uint8_t read8(uint8_t reg);
uint16_t read16(uint8_t reg);
bool sensor_init();
void delay_ms(uint16_t ms);
uint8_t get_datarate();
void set_datarate(uint8_t datarate);
uint8_t get_range();
void set_range(uint8_t range);
void get_acceleration(float *x, float *y, float *z);

// TODO Insert appropriate #include <>

// TODO Insert C++ class definitions if appropriate

// TODO Insert declarations

// Comment a function and leverage automatic documentation with slash star star
/**
    <p><b>Function prototype:</b></p>
  
    <p><b>Summary:</b></p>

    <p><b>Description:</b></p>

    <p><b>Precondition:</b></p>

    <p><b>Parameters:</b></p>

    <p><b>Returns:</b></p>

    <p><b>Example:</b></p>
    <code>
 
    </code>

    <p><b>Remarks:</b></p>
 */
// TODO Insert declarations or function prototypes (right here) to leverage 
// live documentation

#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

