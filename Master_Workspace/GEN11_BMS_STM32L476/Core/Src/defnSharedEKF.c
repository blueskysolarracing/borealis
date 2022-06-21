#ifndef SHARED_DATA_EKF
#define SHARED_DATA_EKF

#include "defnSharedEKF.h"

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

#endif //SHARED_DATA_EKF
