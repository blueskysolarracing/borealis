#ifndef BATTERY_EKF_H
#define BATTERY_EKF_H

#define _WINDOWS (0)
#define OFFLINE_TEST (0)
#define DEBUG_PRINTS (0)

#if OFFLINE_TEST

#include "blueskyOCVData.h"
#include "blueskyExperimentData.h"

#else

#include "blueskyOCVData.h"
#include <math.h>
#include <stdlib.h>

#endif //OFFLINE_TEST

//#if _WINDOWS

#define NOMINMAX
//#include <windows.h>
#include <stdio.h> //don't need for stm code
#include <math.h>
#include <stdint.h>
#include <time.h>
#include <stdlib.h>

//#endif //_WINDOWS

#if OFFLINE_TEST

#define NUM_14P_UNITS (5) // one bms is exception which has 4 units
#define DELTA_T (1.0f)  // in second

#define STATE_NUM (3)
#define INPUT_NUM (2)
#define OUTPUT_NUM (1)

#define Q_CAP (176400.0f)   // in Ampere Second  49 Ampere hour = 49*3600 = 176400 Ampere Second
#define R_INT (0.0074f)    // in Ohm
#define R_CT  (0.005f)     // in Ohm
#define C_CT  (4772.21f)   // in Farad
#define R_D   (0.005f)     // in Ohm
#define C_D   (4772.21f)   // in Farad

#define VAR_Z    (2e-4f)
#define VAR_I_D  (1e-6f)
#define VAR_I_CT (1e-6f)
#define VAR_SENS (2e-1f)    // Sensor uncertainty, terminal voltage measurement
#define VAR_INPT (2e-1f)    // Input uncertainty, input current measurement (sensor)

#define COULOMB_ETA (0.9929f)

#else

#define NUM_14P_UNITS (5) // one bms is exception which has 4 units
#define DELTA_T (1.0f)  // in second

#define STATE_NUM (3)
#define INPUT_NUM (2)
#define OUTPUT_NUM (1)

#define Q_CAP (176400.0f)   // in Ampere Second  49 Ampere hour = 49*3600 = 176400 Ampere Second
#define R_INT (0.0074f)    // in Ohm
#define R_CT  (0.005f)     // in Ohm
#define C_CT  (4772.21f)   // in Farad
#define R_D   (0.005f)     // in Ohm
#define C_D   (4772.21f)   // in Farad

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
