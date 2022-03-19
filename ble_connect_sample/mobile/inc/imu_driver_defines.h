#ifndef IMU_DRIVER_DEFINES_H
#define IMU_DRIVER_DEFINES_H

#define INV_SUCCESS 0
#define INV_ERROR 0x20

int ax, ay, az;
int gx, gy, gz;
int mx, my, mz;
long qw, qx, qy, qz;
long temperature;
unsigned long time;
float pitch, roll, yaw;
float heading;

enum t_axisOrder
{
    X_AXIS, // 0
    Y_AXIS, // 1
    Z_AXIS  // 2
};

// Define's passed to update(), to request a specific sensor (or multiple):
#define UPDATE_ACCEL (1 << 1)
#define UPDATE_GYRO (1 << 2)
#define UPDATE_COMPASS (1 << 3)
#define UPDATE_TEMP (1 << 4)

#define INT_ACTIVE_HIGH 0
#define INT_ACTIVE_LOW 1
#define INT_LATCHED 1
#define INT_50US_PULSE 0

#define MAX_DMP_SAMPLE_RATE 200 // Maximum sample rate for the DMP FIFO (200Hz)
#define FIFO_BUFFER_SIZE 512    // Max FIFO buffer size

const signed char defaultOrientation[9] = {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1};
#define ORIENT_PORTRAIT 0
#define ORIENT_LANDSCAPE 1
#define ORIENT_REVERSE_PORTRAIT 2
#define ORIENT_REVERSE_LANDSCAPE 3

#endif