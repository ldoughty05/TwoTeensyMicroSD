/**************************************************************************//**
 *
 * @file imu.h
 *
 * @author Luke Doughty
 *
 * @brief Establishing I2C communication with the ICM-20948 IMU sensor 
 *        and printing readings.
 *
 * These functions provide a way to 
 *
 ******************************************************************************/
#include "ICM_20948.h" // for the ICM_20948_AGMT_t struct

#define USE_SPI // Comment out this line if using I2C
#define SERIAL_PORT Serial5 // UART to other Teensy


#ifndef IMU_H
#define IMU_H
#endif

/*
 * Verify whoami register to make sure the ICM-20948 is connected.
 * 
 * @param myICM The ICM-20948 device.
 */
// void findIMU(ICM_20948_Device_t* myICM);

/*
 * Set sampling modes and full scale ranges.
 * 
 * @param myICM The ICM-20948 device.
 */
// void configureSensors(ICM_20948_Device_t* myICM);

/*
 * Set up the digital low pass filter to get rid of noisey data.
 * 
 * @param myICM The ICM-20948 device.
 */
// void setupLowPassFilter(ICM_20948_Device_t* myICM);

/* 
 * Print the raw readings of all four sensors to the serial port.
 *
 * @param agmt The raw readings of the accelerometer, gyroscope, magnetometer, and temperature sensor.
 */
void printRawAGMT(ICM_20948_AGMT_t agmt);

/*
 * Initialized IMU, configures sensors, sets up low pass filter, and wakes up the sensor.
 * Works for both I2C and SPI. Uncoment #define USE_SPI if using SPI.
 *
 * @param myICM The ICM-20948 device.
 *
 */
void setupIMU(ICM_20948_Device_t* myICM);