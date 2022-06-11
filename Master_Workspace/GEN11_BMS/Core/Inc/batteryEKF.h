#ifndef BATTERY_EKF_H
#define BATTERY_EKF_H

#define _WINDOWS (1)
#define OFFLINE_TEST (0)
#define DEBUG_PRINTS (0)

#if OFFLINE_TEST
#include "blueskyOCVData.h"
#endif //OFFLINE_TEST

#if _WINDOWS

#define NOMINMAX
//#include <windows.h>
#include <stdio.h> //don't need for stm code
#include <math.h>
#include <stdint.h> //don't need for stm code
#include <time.h> //don't need for stm code
#include <stdlib.h> //don't need for stm code

#endif //_WINDOWS

#if OFFLINE_TEST

//choosen delta t are from E2_DYNData_35_P25 script 1
#define NUM_14P_UNITS (29)
#define DELTA_T (1.0f)  // sampling rate in seconds. constant

#define STATE_NUM (3)
#define INPUT_NUM (2)
#define OUTPUT_NUM (1)
//choosen parameter are for when temperature T= 25
#define Q_CAP (5.1344368f*3600)   // in Ah how precise do we want these values to be?
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

#else

#define NUM_14P_UNITS (29)
#define DELTA_T (1.0f)  // sampling rate in seconds. constant

#define STATE_NUM (3)
#define INPUT_NUM (2)
#define OUTPUT_NUM (1)
//choosen parameter are for when temperature T= 25
#define Q_CAP (5.1344368f*3600)   // in Ah how precise do we want these values to be?
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

#endif //OFFLINE_TEST

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
void compute_A_B_dt(float dt);

void addition_EKF(float* operand_1, float* operand_2, float* result, uint8_t* size, uint8_t subtract);
uint8_t multiply_EKF(float* operand_1, float* operand_2, float* result, uint8_t* opDim_1, uint8_t* opDim_2);
void transpose_EKF(float* in, float* _out,  uint8_t* dimIn);
void createIdentity_EKF(float* inBuffer, uint8_t size);
uint8_t inverse_EKF(float* inMatrix, float* outMatrix, uint8_t* dim);

void printMatrix(float* input, uint8_t* size);
float OCV(float soc);

#if OFFLINE_TEST

void run_EKF(EKF_Model_14p* inputBatt, uint32_t testDataID);

#else

void run_EKF(EKF_Model_14p* inputBatt, float dt);

#endif

// -------------- CONSTANTS AND EKF ALGO VARIABLES --------------
uint8_t dim1[2] = {3, 3};
uint8_t dim2[2] = {3, 1};
uint8_t dim3[2] = {1, 3};
uint8_t dim4[2] = {1, 1};
uint8_t dim5[2] = {3, 2};
uint8_t dim6[2] = {2, 1};
uint8_t dim7[2] = {1, 2};

// a priori state covariance matrix - k+1|k (prediction)
float P_k[STATE_NUM*STATE_NUM] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
// noise in state measurement (sampled) - expected value of the distribution
float Q[STATE_NUM*STATE_NUM] = {VAR_INPT, 0.0f, 0.0f, 0.0f, VAR_INPT, 0.0f, 0.0f, 0.0f, VAR_INPT};
// noise in output sensor (sampled) - expected value of the distribution
float R[OUTPUT_NUM] = {VAR_SENS};
// a priori measurement covariance
float S[OUTPUT_NUM] = {0.0f};
// Kalman Weight
float W[STATE_NUM*OUTPUT_NUM] = {0.0f, 0.0f, 0.0f};

float covList[STATE_NUM] = {VAR_Z, VAR_I_CT, VAR_I_D};
float A[STATE_NUM*STATE_NUM] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float B[STATE_NUM*INPUT_NUM] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float C[STATE_NUM*OUTPUT_NUM] = {0, -R_CT, -R_D};
float D[INPUT_NUM] = {-R_INT, 0.0f};

float U[INPUT_NUM] = {0.0f};
float V_Measured[1] = {0.0f};
float V_OCV[1] = {0.0f};

// ------------- HELPER VARIABLES TO COMPUTE INVERSES -------------

float A_T[STATE_NUM*STATE_NUM] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float A_P[STATE_NUM*STATE_NUM] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float A_P_AT[STATE_NUM*STATE_NUM] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float P_k1[STATE_NUM*STATE_NUM] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

float C_T[STATE_NUM*OUTPUT_NUM] = {0.0f, 0.0f, 0.0f};
float C_P[STATE_NUM*OUTPUT_NUM] = {0.0f, 0.0f, 0.0f};
float C_P_CT[OUTPUT_NUM*OUTPUT_NUM] = {0.0f};

float SInv[OUTPUT_NUM*OUTPUT_NUM] = {0.0f};

float P_CT[STATE_NUM*OUTPUT_NUM] = {0.0f, 0.0f, 0.0f};

float W_T[OUTPUT_NUM*STATE_NUM] = {0.0f, 0.0f, 0.0f};
float W_S[STATE_NUM*OUTPUT_NUM] = {0.0f, 0.0f, 0.0f};
float W_S_WT[STATE_NUM*STATE_NUM] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

float A_X[STATE_NUM] = {0.0f, 0.0f, 0.0f};
float B_U[STATE_NUM*INPUT_NUM] = {0.0f, 0.0f};

float X_k1[STATE_NUM] = {0.0f, 0.0f, 0.0f};

float Z_k1[OUTPUT_NUM] = {0.0f};
float CX_DU[OUTPUT_NUM] = {0.0f};
float C_X[OUTPUT_NUM] = {0.0f};
float D_U[OUTPUT_NUM] = {0.0f};

float Z_err[OUTPUT_NUM] = {0.0f};
float W_Zerr[STATE_NUM] = {0.0f};

#endif // BATTERY_EKF_H
