#ifndef DEFN_SHARED_EKF_H
#define DEFN_SHARED_EKF_H

#include <stdlib.h>
#include <stdint.h>

#define OFFLINE_TEST (0)

#if OFFLINE_TEST

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

// -------------- CONSTANTS AND EKF ALGO VARIABLES --------------
extern uint8_t dim1[2];
extern uint8_t dim2[2];
extern uint8_t dim3[2];
extern uint8_t dim4[2];
extern uint8_t dim5[2];
extern uint8_t dim6[2];
extern uint8_t dim7[2];

// a priori state covariance matrix - k+1|k (prediction)
extern float P_k[STATE_NUM*STATE_NUM];
// noise in state measurement (sampled) - expected value of the distribution
extern float Q[STATE_NUM*STATE_NUM];
// noise in output sensor (sampled) - expected value of the distribution
extern float R[OUTPUT_NUM];
// a priori measurement covariance
extern float S[OUTPUT_NUM];
// Kalman Weight
extern float W[STATE_NUM*OUTPUT_NUM];

extern float covList[STATE_NUM];
extern float A[STATE_NUM*STATE_NUM];
extern float B[STATE_NUM*INPUT_NUM];
extern float C[STATE_NUM*OUTPUT_NUM];
extern float D[INPUT_NUM];

extern float U[INPUT_NUM];
extern float V_Measured[1];
extern float V_OCV[1];

// ------------- HELPER VARIABLES TO COMPUTE INVERSES -------------

extern float A_T[STATE_NUM*STATE_NUM];
extern float A_P[STATE_NUM*STATE_NUM];
extern float A_P_AT[STATE_NUM*STATE_NUM];
extern float P_k1[STATE_NUM*STATE_NUM];

extern float C_T[STATE_NUM*OUTPUT_NUM];
extern float C_P[STATE_NUM*OUTPUT_NUM];
extern float C_P_CT[OUTPUT_NUM*OUTPUT_NUM];

extern float SInv[OUTPUT_NUM*OUTPUT_NUM];

extern float P_CT[STATE_NUM*OUTPUT_NUM];

extern float W_T[OUTPUT_NUM*STATE_NUM];
extern float W_S[STATE_NUM*OUTPUT_NUM];
extern float W_S_WT[STATE_NUM*STATE_NUM];

extern float A_X[STATE_NUM];
extern float B_U[STATE_NUM*INPUT_NUM];
extern float CX_DU[OUTPUT_NUM];
extern float X_k1[STATE_NUM];
extern float Z_k1[OUTPUT_NUM];

extern float C_X[OUTPUT_NUM];
extern float D_U[OUTPUT_NUM];

extern float Z_err[OUTPUT_NUM];
extern float W_Zerr[STATE_NUM];

#endif //DEFN_SHARED_EKF_H
