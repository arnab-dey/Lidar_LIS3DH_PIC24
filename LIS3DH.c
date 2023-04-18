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
    // High res & BDU enabled
    write8(LIS3DH_REG_CTRL4, 0x88);
    // Enable ADCs
    write8(LIS3DH_REG_TEMPCFG, 0x80);
    // Enable interrupt
    write8(LIS3DH_REG_CTRL5, 0x08);
    
    // For sanity check: Read CTRL1 register and see if we read
    // the value we wrote
    id = read8(LIS3DH_REG_CTRL1);
    if (id != 0x07) {
        return false;
    }
    return true;
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
