#ifndef INDEX_H
#define INDEX_H

#define KALMANQ 1e-4
#define KALMANR 1e-2
#define REFSTEP 1  // define the steps between currImage and refImage
#define IMAGESTORE_MAX_SIZE 40
#define SMOOTH_RADIUS 10

// filter method
#define FILTER_EKF 0
#define FILTER_EKF_EXTEND 1
#define FILTER_SMOOTH 2

#define WRITE_FILE true
#define WAIT_TIME 20

#endif
