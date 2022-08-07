#ifndef BATTERY_EKF_H
#define BATTERY_EKF_H

#include <math.h>
#include <string.h>
#include "defnSharedEKF.h"

#define NOMINMAX
#define _WINDOWS (0)
#define DEBUG_PRINTS (0)

#define NUM_14P_UNITS (5) // one bms is exception which has 4 units
#define LUT_SIZE (201)
#define DELTA_T (1.0f)  // in second

#if _WINDOWS

#include <windows.h>
#include <stdio.h>
#include <time.h>

#endif //_WINDOWS

typedef struct {
    float stateX[STATE_NUM];
    float covP[STATE_NUM*STATE_NUM];
} EKF_Model_14p;

typedef struct {
    EKF_Model_14p batteryPack[NUM_14P_UNITS];
} EKF_Battery;

void initBatteryAlgo(EKF_Battery* inBatteryPack, float* initial_v, float initial_deltaT);
void initEKFModel(EKF_Model_14p* inModel, float initial_v);
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

void run_EKF(EKF_Model_14p* inputBatt, float dt, float currentIn, float measuredV);

#endif

// ------------------ EXTERNAL DEFN OF SoC v V_Out ------------------
extern float BSSR_OCV[LUT_SIZE];
extern float BSSR_SOC[LUT_SIZE];

// ------------------ EXTERNAL DEFN OF Offline Test Data ------------------
#if OFFLINE_TEST

#define MAX_ITERATION_NUM (10)

extern float deltaT[MAX_ITERATION_NUM];
extern float current[MAX_ITERATION_NUM];
extern float voltage[MAX_ITERATION_NUM];

#endif //OFFLINE_TEST

#endif // BATTERY_EKF_H
