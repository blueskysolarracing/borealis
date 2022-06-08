#include "batteryEKF.h"

// -------------- CONSTANTS AND EKF ALGO VARIABLES --------------
uint8_t dim1[2] = {STATE_NUM, STATE_NUM};
uint8_t dim2[2] = {STATE_NUM, INPUT_NUM};
uint8_t dim3[2] = {STATE_NUM, 1};
uint8_t dim4[2] = {OUTPUT_NUM, STATE_NUM};
uint8_t dim5[2] = {1, 1};
uint8_t dim6[2] = {INPUT_NUM, 1};
uint8_t dim7[2] = {OUTPUT_NUM, INPUT_NUM};

// a priori state covariance matrix - k+1|k (prediction)
float P_k[STATE_NUM*STATE_NUM] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
// noise in state measurement (sampled) - expected value of the distribution
float Q[STATE_NUM*STATE_NUM] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
// noise in output sensor (sampled) - expected value of the distribution
float R[OUTPUT_NUM] = {0.0f};
// a priori measurement covariance
float S[OUTPUT_NUM] = {0.0f};
// Kalman Weight
float W[STATE_NUM*OUTPUT_NUM] = {0.0f, 0.0f, 0.0f};

float covList[STATE_NUM] = {VAR_Z, VAR_I_CT, VAR_I_D};
float A[STATE_NUM*STATE_NUM] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float B[STATE_NUM*INPUT_NUM] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float C[STATE_NUM*OUTPUT_NUM] = {0, -R_CT, -R_D};
float D[INPUT_NUM] = {-R_INT, 0.0f};

float U[INPUT_NUM] = {0.0f, 0.0f};
float V_Measured[1] = {0.0f};

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
float C_X[OUTPUT_NUM] = {0.0f};
float D_U[OUTPUT_NUM] = {0.0f};

float Z_err[OUTPUT_NUM] = {0.0f};
float W_Zerr[STATE_NUM] = {0.0f};

//--- FUNCTIONS ---//

void initBatteryAlgo(EKF_Battery* inBatteryPack){

    for(uint8_t unit = 0; unit < NUM_14P_UNITS; unit++){  
        initEKFModel(&inBatteryPack->batteryPack[unit]);
    }

    init_A_Matrix();
    init_B_Matrix();

    return;
}

void initEKFModel(EKF_Model_14p* inBattery){

    uint8_t i = 0;  
    uint8_t j = 0;

    // initialize state variables
    for (i = 0; i < STATE_NUM; i++){
        if(i == 0U){
            inBattery->stateX[i] = 1.0f;
        } else {
            inBattery->stateX[i] = 0.0f;
        }
    }

    // initialize the covariance matrix
    for(i = 0; i < STATE_NUM; i++){
        for(j = 0; j < STATE_NUM; j++){
            
            if (i == j){
                inBattery->covP[i*STATE_NUM + j] = covList[i];
            } else {
                inBattery->covP[i*STATE_NUM + j] = 0.0f;
            }

        }
    }

    return;
}

void init_A_Matrix(){

    A[0] = 1.0f;
    A[4] = exp(-DELTA_T/(R_CT*C_CT));
    A[8] = exp(-DELTA_T/(R_D*C_D));

    return;
}

void init_B_Matrix(){

    B[0] = -COULOMB_ETA*DELTA_T/Q_CAP; //in example, this equation is only multiplied by ETA when the current is negative
    B[2] = 1 - exp(-DELTA_T/(R_CT*C_CT));
    B[4] = 1 - exp(-DELTA_T/(R_D*C_D));

    return;
}

void addition_EKF(float* operand_1, float* operand_2, float* result, uint8_t* size,  uint8_t subtract){

    // check if we are doing addition or subtraction based on the 'subtract' boolean
    float coefficient = (0 == subtract) ? 1.0f : -1.0f;

    uint8_t rows = size[0];
    uint8_t cols = size[1];

    for(uint8_t i = 0; i < rows; i++){
        for(uint8_t j = 0; j < cols; j++){
            result[i*cols + j] = operand_1[i*cols + j] + coefficient*operand_2[i*cols + j];
        }
    }

    return;
}

uint8_t multiply_EKF(float* operand_1, float* operand_2, float* result, uint8_t* opDim_1, uint8_t* opDim_2){

    // opDim_1 and opDim_2 conatin the dimensions of operand_1 and operand_2 in an array of size 2 
    uint8_t r_1 = opDim_1[0];
    uint8_t c_1 = opDim_1[1];
    uint8_t r_2 = opDim_2[0];
    uint8_t c_2 = opDim_2[1];

    // temporary variables
    float entryVal = 0.0f;

    if (c_1 != r_2 ){
        return 0;
    }

    for(uint8_t i = 0; i < r_1; i++){
        for(uint8_t j = 0; j < c_2; j++){

            for(uint8_t k = 0; k < c_1; k++){
                entryVal += operand_1[i*c_1 + k]*operand_2[k*c_2 + j];
            }

            result[i*c_2 + j] = entryVal;
            entryVal = 0.0f;

        }
    }

    return 1;
}

void transpose_EKF(float* in, float* out, uint8_t* dimIn){

    uint8_t rows = dimIn[0];
    uint8_t cols = dimIn[1];

    for(uint8_t i = 0; i < rows; i++){
        for(uint8_t j = 0; j < cols; j++) {
            out[j*rows + i] = in[i*cols + j];
        }
    }

    return;
}

void createIdentity_EKF(float* inBuffer, uint8_t size){

    for(uint8_t i = 0; i < size; i++){
        for(uint8_t j = 0; j < size; j++){
            inBuffer[i*size + j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    return;
}

uint8_t inverse_EKF(float* in, float* out, uint8_t* dim){

    uint8_t rows = dim[0];
    uint8_t cols = dim[1];

    if(rows != cols){
        printf("Matrix is not square - cannot compute inverse with this method... \n");
        return 0;
    }

    if(1 == rows){

        out[0] = 1.0f/in[0];
        return 1;

    } else {

        // create a copy of the current input
        float* copyIn = (float*)calloc(rows*cols, sizeof(float));
        (void)memcpy(copyIn, in, rows*cols*sizeof(float));
        
        createIdentity_EKF(out, rows);
        float ratio = 0.0f;

        //Gaussian Elimination: create upper triangle matrix
        for (uint8_t j = 0; j < cols - 1; j++) {

            //iterate through the lower triangle to cancel out all elements    
            for (uint8_t i = j+1; i < rows; i++) {

                ratio = copyIn[i*cols+j]/ copyIn[j*cols+j];
               
                for (uint8_t k = 0; k < cols; k++) {
                    copyIn[i*cols+k] -= copyIn[j*cols+k] * ratio;
                    copyIn[i*cols+k] -= out[j*cols+k] * ratio;
                }

            }

        }

        //Jordan Elimination: create diagonal matrix
        for (uint8_t j=0; j < cols - 1; j++) {

            for (uint8_t i=j+1; i < rows; i++) {

                ratio = copyIn[(cols - 1 - i)*cols+(cols - 1 - j)]/copyIn[(cols - 1 - j)*cols+(cols - 1 - j)];
                copyIn[(cols - 1 - i)*cols+(cols - 1 - j)] -= copyIn[(cols - 1 - j)*cols+(cols - 1 - j)] * ratio;
                
                for (uint8_t k = 0; k < cols; k++) {
                    out[(cols - 1 - i)*cols+(cols - 1 - k)] -= out[(cols - 1 - j)*cols+(cols - 1 - k)] * ratio;
                }

            }

        }

       free(copyIn);
    }
    return 1;
}

float run_EKF(EKF_Model_14p* inputBatt, float battCurrent, float cellVoltage){
    // compute the a priori state covariance
    transpose_EKF(A, A_T, dim1);
    multiply_EKF(A, inputBatt->covP, A_P, dim1, dim1);
    multiply_EKF(A_P, A_T, A_P_AT, dim1, dim1);
    addition_EKF(A_P_AT, Q, P_k1, dim1, 0); //where does Q come from?

    // compute the a priori state covariance
    transpose_EKF(C, C_T, dim4);
    multiply_EKF(C, P_k1, C_P, dim4, dim1);
    multiply_EKF(C_P, C_T, C_P_CT, dim4, dim3);
    addition_EKF(C_P_CT, R, S, dim5, 0);

    // compute Kalman Gain
    inverse_EKF(S, SInv, dim5);
    multiply_EKF(P_k1, C_T, P_CT, dim1, dim3);
    multiply_EKF(P_CT, SInv, W, dim3, dim5);

    // update Posteriori covariance
    transpose_EKF(W, W_T, dim3);
    multiply_EKF(W, S, W_S, dim3, dim5);
    multiply_EKF(W_S, W_T, W_S_WT, dim3, dim4);
    addition_EKF(P_k1, W_S_WT, inputBatt->covP, dim1, 1);

    // a priori state estimate
    multiply_EKF(A, inputBatt->stateX, A_X, dim1, dim3);
    multiply_EKF(B, U, B_U, dim2, dim6);
    addition_EKF(A_X, B_U, X_k1, dim3, 0);

    // a priori measurement
    float I_Input = battCurrent;
    float I_InSign = (I_Input > 0.0f) ? 1.0f : -1.0f;

    U[0] = I_Input;
    U[1] = I_InSign;
    multiply_EKF(C, X_k1, C_X, dim4, dim3);
    multiply_EKF(D, U, D_U, dim7, dim6);
    addition_EKF(C_X, D_U, Z_k1, dim3, 0);

    // update of the posteriori state estimate
    V_Measured[0] = cellVoltage;   // voltage reading

    addition_EKF(V_Measured, Z_k1, Z_err, dim5, 1);
    multiply_EKF(W, Z_err, W_Zerr, dim3, dim5);
    addition_EKF(X_k1, W_Zerr, inputBatt->stateX, dim2, 0);    // SOC in inputBatt->stateX[0]

    return inputBatt->stateX[0];
}
