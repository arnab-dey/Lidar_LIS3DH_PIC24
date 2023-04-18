/*
 * File:   LIS3DH.c
 * Author: Arnab Dey
 * Description: Lidar LIS3DH library
 * I2C1 is used to interface the sensor
 * Created on April 17, 2023, 8:00 PM
 */
#include "xc.h"
#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "LIS3DH.h"

static uint8_t is_rep_start_state = 0;
static uint8_t current_range = LIS3DH_RANGE_2G;
static uint8_t mode = POWER_MODE_NORMAL;

void write(const uint8_t *buffer, size_t len, bool stop){
    // Send start
    IFS1bits.MI2C1IF = 0;
    I2C1CONbits.SEN = 1;
    while(I2C1CONbits.SEN == 1);
    IFS1bits.MI2C1IF = 0;
    // Select slave
    I2C1TRN = (LIS3DH_ADDRESS << 1) | I2C_WRITE;
    while(IFS1bits.MI2C1IF == 0);
    IFS1bits.MI2C1IF = 0;
    // Write prefix data: TODO: Check len and send 
//    I2C1TRN = *prefix_buffer | TCS34725_COMMAND_BIT;
//    while(IFS1bits.MI2C1IF == 0);
//    IFS1bits.MI2C1IF = 0;
    size_t i;
    for(i = 0; i < len; ++i){
        // Write data itself
        I2C1TRN = buffer[i];
        while(IFS1bits.MI2C1IF == 0);
        IFS1bits.MI2C1IF = 0;
    }
    
    if (stop == true) {
        is_rep_start_state = 0;
        // End transmission
        I2C1CONbits.PEN = 1;
        while(I2C1CONbits.PEN == 1);
    } else {
        is_rep_start_state = 1;
    }
}

void read(uint8_t *buffer, size_t len, bool stop) {
    IFS1bits.MI2C1IF = 0;
    // Check if we are in repeated start state
    if (is_rep_start_state == 1) {
        // Send repeated start state
        I2C1CONbits.RSEN = 1;
        while(I2C1CONbits.RSEN == 1);
        IFS1bits.MI2C1IF = 0;
        is_rep_start_state = 0;
    } else {
        // Send start
        I2C1CONbits.SEN = 1;
        while(I2C1CONbits.SEN == 1);
        IFS1bits.MI2C1IF = 0;
    }
    // Select slave with read
    I2C1TRN = (LIS3DH_ADDRESS << 1) | I2C_READ;
    while(IFS1bits.MI2C1IF == 0);
    IFS1bits.MI2C1IF = 0;
    
    // Receive data
    size_t i;
    for(i = 0; i < len; ++i){
        I2C1CONbits.RCEN = 1;
        while(I2C1CONbits.RCEN == 1);
        buffer[i] = I2C1RCV;
        while(IFS1bits.MI2C1IF == 0);
        IFS1bits.MI2C1IF = 0;
        if (i == len-1) {
            I2C1CONbits.ACKDT = 1;
        } else {
            I2C1CONbits.ACKDT = 0;
        }
        I2C1CONbits.ACKEN = 1;
        while(IFS1bits.MI2C1IF == 0);
        IFS1bits.MI2C1IF = 0;
    }
    
    if (stop == true) {
        // Send stop
        I2C1CONbits.PEN = 1;
        while(I2C1CONbits.PEN == 1);
        is_rep_start_state = 0;
    } else {
        is_rep_start_state = 1;
    }
}

void write_then_read(const uint8_t *write_buffer, size_t write_len,
        uint8_t *read_buffer, size_t read_len, bool stop) {
    write(write_buffer, write_len, stop);
    read(read_buffer, read_len, true);
}

void write8(uint8_t reg, uint32_t value) {
    uint8_t buffer[2] = {reg, value & 0xFF};
    write(buffer, 2, true);
}

uint8_t read8(uint8_t reg) {
  uint8_t buffer[1] = {reg};
  write_then_read(buffer, 1, buffer, 1, false);
  return buffer[0];
}

uint16_t read16(uint8_t reg) {
  uint8_t buffer[2] = {reg, 0};
  write_then_read(buffer, 1, buffer, 2, false);
  return ((uint16_t)(buffer[1]) << 8) | ((uint16_t)(buffer[0]) & 0xFF);
}

bool sensor_init() {
    // First read the device ID
    uint8_t id = read8(LIS3DH_REG_WHO_AM_I);
    if (id != 0x33) {
        return false;
    }
    // Reboot
    write8(LIS3DH_REG_CTRL5, 0x80);
    delay_ms(10); // takes 5ms
    // Enable all axes, normal mode
    write8(LIS3DH_REG_CTRL1, 0x07);
    // High res enabled & BDU enabled
    mode = POWER_MODE_HIGH_RESOLUTION;
    write8(LIS3DH_REG_CTRL4, 0x88);
    // Disable ADCs
    write8(LIS3DH_REG_TEMPCFG, 0x00);
    // Disable interrupt
    write8(LIS3DH_REG_CTRL5, 0x00);
    // Set datarate
    set_datarate(LIS3DH_DATARATE_400HZ);
    // Set range
    set_range(LIS3DH_RANGE_2G);
    
    // For sanity check: Read CTRL1 register and see if we read
    // the value we wrote
//    id = read8(LIS3DH_REG_CTRL1);
//    if (id != 0x07) {
//        return false;
//    }
    return true;
}

uint8_t get_datarate(){
    uint8_t datarate = read8(LIS3DH_REG_CTRL1);
    return (datarate >> 4) & 0x0F;
}

void set_datarate(uint8_t datarate){
    // First read the existing value
    uint8_t val = read8(LIS3DH_REG_CTRL1);
    // Prepare the datarate to be set
    uint8_t datarate_to_be_set = ((datarate & 0x0F) << 4);
    val &= ~(0xF0); // Clear 4 MSBs
    val |= datarate_to_be_set; // Update 4 MSBs with the datarate
    // Write to the register
    write8(LIS3DH_REG_CTRL1, val);
}
/*
 * Returns the range of the accelerometer
 */
uint8_t get_range(){
    uint8_t range = read8(LIS3DH_REG_CTRL4);
    // FS bits give the range
    return (range >> 4) & 0x03;
}

void set_range(uint8_t range){
    // First read the existing value
    uint8_t val = read8(LIS3DH_REG_CTRL4);
    // Prepare the range to be set
    uint8_t range_to_be_set = ((range & 0x03) << 4);
    val &= ~(0x30); // Clear FS bits
    val |= range_to_be_set; // Update FS bits
    // Write to the register
    write8(LIS3DH_REG_CTRL4, val);
    current_range = range; // Update so that we don't need to read always
}

void get_acceleration(float *x, float *y, float *z){
    // 2 reads (high and low bytes) for each axis
    // So need to read total 6 registers to get acceleration
    // values for x,y,z axes
    // Get X-axis acceleration
    int16_t accl_x = (int16_t)read16(LIS3DH_REG_OUT_X_L | ADDRESS_AUTO_INCREMENT);
    // Get Y-axis acceleration
    int16_t accl_y = (int16_t)read16(LIS3DH_REG_OUT_Y_L | ADDRESS_AUTO_INCREMENT);
    // Get Z-axis acceleration
    int16_t accl_z = (int16_t)read16(LIS3DH_REG_OUT_Z_L | ADDRESS_AUTO_INCREMENT);
    // Now adjust the according to range and set divider for m/s^2 value
    float divider = 1;
    switch(mode){
        // Calculation equation
        // A: Hi-res: Output is 12 bit left-adjusted: First need to make it right
        // adjusted = divide by (1<<4)
        // In case of normal power mode, output is 10 bit left-adjusted: So
        // divide by (1<<6)
        // In case of low poer mode, output is 8 bit left-adjusted: So
        // divide by (1<<8)
        // B: Sensitivity unit is mg/digit: Convert to to g/digit
        // So divide by 1000 which is converted to 1024 for ease
        // of calculation as per the data sheet
        // C: Then multiply by sensitivity
        // D: Convert to m/s^2: multiply by STANDARD_GRAVITY (~9.806 m/s^2)
        // Total conversion factor = C*D/(A*B)
        // We use divider notation; so our divider will be (A*B)/(C*D)
        case POWER_MODE_HIGH_RESOLUTION: 
            if (LIS3DH_RANGE_16G == current_range){
                divider = ((float)(1 << 4)*(float)1024)/(float)(LIS3DH_SENSITIVITY_HIRES_16G*STANDARD_GRAVITY);
            } else if (LIS3DH_RANGE_8G == current_range) {
                divider = ((float)(1 << 4)*(float)1024)/(float)(LIS3DH_SENSITIVITY_HIRES_8G*STANDARD_GRAVITY);
            } else if (LIS3DH_RANGE_4G == current_range) {
                divider = ((float)(1 << 4)*(float)1024)/(float)(LIS3DH_SENSITIVITY_HIRES_4G*STANDARD_GRAVITY);
            } else if (LIS3DH_RANGE_2G == current_range) {
                divider = ((float)(1 << 4)*(float)1024)/(float)(LIS3DH_SENSITIVITY_HIRES_2G*STANDARD_GRAVITY);
            }
            break;
        case POWER_MODE_NORMAL:
            if (LIS3DH_RANGE_16G == current_range){
                divider = ((float)(1 << 6)*(float)1024)/(float)(LIS3DH_SENSITIVITY_NORMAL_16G*STANDARD_GRAVITY);
            } else if (LIS3DH_RANGE_8G == current_range) {
                divider = ((float)(1 << 6)*(float)1024)/(float)(LIS3DH_SENSITIVITY_NORMAL_8G*STANDARD_GRAVITY);
            } else if (LIS3DH_RANGE_4G == current_range) {
                divider = ((float)(1 << 6)*(float)1024)/(float)(LIS3DH_SENSITIVITY_NORMAL_4G*STANDARD_GRAVITY);
            } else if (LIS3DH_RANGE_2G == current_range) {
                divider = ((float)(1 << 6)*(float)1024)/(float)(LIS3DH_SENSITIVITY_NORMAL_2G*STANDARD_GRAVITY);
            }
            break;
        case POWER_MODE_LOW:
            if (LIS3DH_RANGE_16G == current_range){
                divider = ((float)(1 << 8)*(float)1024)/(float)(LIS3DH_SENSITIVITY_LOWPOWER_16G*STANDARD_GRAVITY);
            } else if (LIS3DH_RANGE_8G == current_range) {
                divider = ((float)(1 << 8)*(float)1024)/(float)(LIS3DH_SENSITIVITY_LOWPOWER_8G*STANDARD_GRAVITY);
            } else if (LIS3DH_RANGE_4G == current_range) {
                divider = ((float)(1 << 8)*(float)1024)/(float)(LIS3DH_SENSITIVITY_LOWPOWER_4G*STANDARD_GRAVITY);
            } else if (LIS3DH_RANGE_2G == current_range) {
                divider = ((float)(1 << 8)*(float)1024)/(float)(LIS3DH_SENSITIVITY_LOWPOWER_2G*STANDARD_GRAVITY);
            }
            break;
        default:
            break;
    }
    // Now calculate the acceleration (m/s^2)
    *x = (float)(accl_x)/divider;
    *y = (float)(accl_y)/divider;
    *z = (float)(accl_z)/divider;
}

/* Detect when the accelerometer is shaken
* @param threshold: Increase or decrease to change shake sensitivity.
*   This requires a minimum value of 10.
*   10 is the total acceleration if the board is not
*   moving, therefore anything less than
*   10 will erroneously report a constant shake detected.
*
* @param avg_count: The number of readings taken and used for the average
*  acceleration.
* @param total_delay_ms: The total time in milliseconds it takes to obtain avg_count
*  readings from acceleration.
*/
bool is_shaken(double threshold, uint16_t avg_count, uint16_t total_delay_ms){
    float acc_x = 0, acc_y = 0, acc_z = 0;
    double total_acc_x = 0, total_acc_y = 0, total_acc_z = 0;
    double avg_acc_x, avg_acc_y, avg_acc_z, total_avg_acc;
    uint8_t i;
    uint16_t delay_per_count_ms = total_delay_ms/avg_count;
    for (i=0;i<avg_count;i++){
        get_acceleration(&acc_x, &acc_y, &acc_z);
        total_acc_x += acc_x;
        total_acc_y += acc_y;
        total_acc_z += acc_z;
        delay_ms(delay_per_count_ms);
    }
    avg_acc_x = (double)total_acc_x/(double)avg_count;
    avg_acc_y = (double)total_acc_y/(double)avg_count;
    avg_acc_z = (double)total_acc_z/(double)avg_count;
    total_avg_acc = sqrt((avg_acc_x*avg_acc_x) + (avg_acc_y*avg_acc_y) + (avg_acc_z*avg_acc_z));
    return total_avg_acc > threshold;
}

void get_tilt_angle(double *pitch, double *roll){
    // Pitch: Angle in X-Z plane (0 degree along X-axis)
    // Roll: Angle in Y-Z plane (0 degree along Y-axis)
    // Yaw: Not possible to estimate from 3-axis accelerometer
    float acc_x, acc_y, acc_z;
    get_acceleration(&acc_x, &acc_y, &acc_z);
    *pitch = atan((acc_x)/(sqrt(acc_y*acc_y+ acc_z*acc_z)))*(180.0/PI);
    *roll = atan(acc_y/(sqrt(acc_x*acc_x+acc_z*acc_z)))*(180.0/PI);
    // Pitch quadrant determination
    if ((acc_z < 0.0) && (acc_x >= 0.0)){
        // Second quadrant
        *pitch = 180.0-*pitch;
    } else if ((acc_z < 0.0) && (acc_x < 0.0)){
        // Third quadrant
        *pitch = -180.0 - *pitch;
    }
    // Roll quadrant determination
    if ((acc_z < 0.0) && (acc_y >= 0.0)){
        // Second quadrant
        *roll = 180.0-*roll;
    } else if ((acc_z < 0.0) && (acc_y < 0.0)){
        // Third quadrant
        *roll = -180.0 - *roll;
    }
}

/*!
 *  @brief  milliseconds delay
 *  @param  ms
 *          Amount of delay in ms
 */
void delay_ms(uint16_t ms){
    while(ms > 0){
    asm("repeat #15993");
    asm("nop");
    ms--;
    }
}
