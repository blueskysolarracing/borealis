#ifndef BATTERY_EKF_H
#define BATTERY_EKF_H

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#define NUM_14P_UNITS (5) // one bms is exception which has 4 units
#define LUT_SIZE (201)
#define DELTA_T (1.0f)  // in second

//shared
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

typedef struct {
   // -------------- CONSTANTS AND EKF ALGO VARIABLES --------------
 uint8_t dim1[2];
 uint8_t dim2[2];
 uint8_t dim3[2];
 uint8_t dim4[2];
 uint8_t dim5[2];
 uint8_t dim6[2];
 uint8_t dim7[2];

// a priori state covariance matrix - k+1|k (prediction)
 float P_k[STATE_NUM*STATE_NUM];
// noise in state measurement (sampled) - expected value of the distribution
 float Q[STATE_NUM*STATE_NUM];
// noise in output sensor (sampled) - expected value of the distribution
 float R[OUTPUT_NUM];
// a priori measurement covariance
 float S[OUTPUT_NUM];
// Kalman Weight
 float W[STATE_NUM*OUTPUT_NUM];

 float covList[STATE_NUM];
 float A[STATE_NUM*STATE_NUM];
 float B[STATE_NUM*INPUT_NUM];
 float C[STATE_NUM*OUTPUT_NUM];
 float D[INPUT_NUM];

 float U[INPUT_NUM];
 float V_Measured[1];
 float V_OCV[1];

// ------------- HELPER VARIABLES TO COMPUTE INVERSES -------------

 float A_T[STATE_NUM*STATE_NUM];
 float A_P[STATE_NUM*STATE_NUM];
 float A_P_AT[STATE_NUM*STATE_NUM];
 float P_k1[STATE_NUM*STATE_NUM];

 float C_T[STATE_NUM*OUTPUT_NUM];
 float C_P[STATE_NUM*OUTPUT_NUM];
 float C_P_CT[OUTPUT_NUM*OUTPUT_NUM];

 float SInv[OUTPUT_NUM*OUTPUT_NUM];

 float P_CT[STATE_NUM*OUTPUT_NUM];

 float W_T[OUTPUT_NUM*STATE_NUM];
 float W_S[STATE_NUM*OUTPUT_NUM];
 float W_S_WT[STATE_NUM*STATE_NUM];

 float A_X[STATE_NUM];
 float B_U[STATE_NUM*INPUT_NUM];
 float CX_DU[OUTPUT_NUM];
 float X_k1[STATE_NUM];
 float Z_k1[OUTPUT_NUM];

 float C_X[OUTPUT_NUM];
 float D_U[OUTPUT_NUM];

 float Z_err[OUTPUT_NUM];
 float W_Zerr[STATE_NUM];

} EKF_Model_Matrix;

typedef struct {
    float stateX[STATE_NUM];
    float covP[STATE_NUM*STATE_NUM];
    EKF_Model_Matrix matrix;
} EKF_Model_14p;

typedef struct {
    EKF_Model_14p batteryPack[NUM_14P_UNITS];
} EKF_Battery;

void initBatteryAlgo(EKF_Model_14p* inBattery, float initial_v, float initial_deltaT);
void initEKFModel(EKF_Model_14p* inModel, float initial_v);
void init_A_Matrix();
void init_B_Matrix();
void compute_A_B_dt(EKF_Model_14p* inModel, float dt);

void addition_EKF(float* operand_1, float* operand_2, float* result, uint8_t* size, uint8_t subtract);
uint8_t multiply_EKF(float* operand_1, float* operand_2, float* result, uint8_t* opDim_1, uint8_t* opDim_2);
void transpose_EKF(float* in, float* _out,  uint8_t* dimIn);
void createIdentity_EKF(float* inBuffer, uint8_t size);
uint8_t inverse_EKF(float* inMatrix, float* outMatrix, uint8_t* dim);

float OCV(float soc);
void run_EKF(EKF_Model_14p* inputBatt, float dt, float currentIn, float measuredV);

// ------------------ EXTERNAL DEFN OF SoC v V_Out ------------------
extern float BSSR_OCV[LUT_SIZE];
extern float BSSR_SOC[LUT_SIZE];

#endif // BATTERY_EKF_H
