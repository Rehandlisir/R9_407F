#ifndef __LEGK__
#define __LEGK__
#include "./SYSTEM/sys/sys.h"
#include "arm_math.h"
#include "./BSP/R9/getadcdata.h"

// #define L1  100.11
// #define L2  100.11
// #define L3  100.11

#define DevAngle 0.3979


typedef struct 
{
    float32_t L0;
    float32_t L1;
    float32_t L2;
    float32_t L3;
    float32_t L4;
    float32_t theta1;
    float32_t theta2;
    float32_t theta3;
    float32_t theta4;
    float32_t theta5;
    float32_t x_o;
    float32_t y_o;
    float32_t z_o;

}S_LEGKINEMATICS_PRA;
extern S_LEGKINEMATICS_PRA legkinematicspra;
void legKinematics(void);
#endif

