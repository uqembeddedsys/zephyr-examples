/**
 ************************************************************************
 * @file hal_imu.h
 * @author Wilfred MK, Aaron Helmore
 * @date 04.03.2021
 * @brief Contains defines and helpers that assist the IMU thread
 **********************************************************************
 * */
#ifndef HAL_IMU_H
#define HAL_IMU_H

/* Debug Thread Stack size */
#define THREAD_IMU_RW_STACK 2048

/* Debug Thread Priority */
#define THREAD_PRIORITY_IMU -1

//Functions

void thread_imu_rw(void);

/* IMU FUNCTIONS */
float getGyroSens(void);

unsigned short getAccelSens(void);

int setSensors(unsigned char sensors);

unsigned short getAccelSens(void);

int updateAccel(void);

int updateGyro(void);

int updateCompass(void);

int setGyroFSR(unsigned short fsr);

int setAccelFSR(unsigned char fsr);

int setLPF(unsigned short lpf);

int setSampleRate(unsigned short rate);

int setCompassSampleRate(unsigned short rate);

bool dataReady(void);

int update(unsigned char sensors);

float calcAccel(int axis);

float calcGyro(int axis);

float calcMag(int axis);

int updateTemperature(void);

int begin(void);

void printIMUData(void);

int dmpLoad(void);

int dmpBegin(unsigned short features, unsigned short fifoRate);

int dmpEnableFeatures(unsigned short mask);

int dmpSetFifoRate(unsigned short rate);

int dmpSetPedometerSteps(unsigned long steps);

int dmpSetPedometerTime(unsigned long time);

unsigned long dmpGetPedometerTime(void);

unsigned long dmpGetPedometerSteps(void);

int dmpBegin(unsigned short features, unsigned short fifoRate);

#endif