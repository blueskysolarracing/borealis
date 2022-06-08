#ifndef BATTERY_EKF_H
#define BATTERY_EKF_H

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

//choosen delta t are from E2_DYNData_35_P25 script 1
#define NUM_14P_UNITS (5)
#define DELTA_T (1.0f)  // sampling rate in seconds. constant

#define STATE_NUM (3)
#define INPUT_NUM (2)
#define OUTPUT_NUM (1)
//choosen parameter are for when temperature T= 25
#define Q_CAP (5.134436834270520f*3600)   // in Ah how precise do we want these values to be?
#define R_INT (0.0112f)     // internal resistance of the 14p battery unit
#define R_CT  (0.0025f)     //R*C=2.4107 R=0.0025 -> C=2.4107/0.0025=964.25
#define C_CT  (964.25f)
#define R_D   (0.0025f)     //assume R_D and C_D value same as R_CT and C_CT
#define C_D   (964.25f)

#define VAR_Z    (2e-4f)
#define VAR_I_D  (1e-6f)
#define VAR_I_CT (1e-6f)
#define VAR_SENS (2e-1f)    // Sensor uncertainty, terminal voltage measurement
#define VAR_INPT (2e-1f)    // Input uncertainty, input current measurement (sensor)

#define COULOMB_ETA (0.9929f)

typedef struct {
    float stateX[STATE_NUM];
    float covP[STATE_NUM*STATE_NUM];
} EKF_Model_14p;

typedef struct {
    EKF_Model_14p batteryPack[NUM_14P_UNITS];
} EKF_Battery;

void initBatteryAlgo(EKF_Battery* inBatteryPack);
void initEKFModel(EKF_Model_14p* inModel);
void init_A_Matrix();
void init_B_Matrix();

void addition_EKF(float* operand_1, float* operand_2, float* result, uint8_t* size, uint8_t subtract);
uint8_t multiply_EKF(float* operand_1, float* operand_2, float* result, uint8_t* opDim_1, uint8_t* opDim_2);
void transpose_EKF(float* in, float* _out,  uint8_t* dimIn);
void createIdentity_EKF(float* inBuffer, uint8_t size);
uint8_t inverse_EKF(float* inMatrix, float* outMatrix, uint8_t* dim);
float run_EKF(EKF_Model_14p* inputBatt, float battCurrent, float cellVoltage);

#endif // BATTERY_EKF_H
