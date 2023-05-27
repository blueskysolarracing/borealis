#include "batteryEKF.h"
#include "blueskyOCVData.h"

#define Q_CAP (173880.0f)   // in Ampere Second  49 Ampere hour = 49*3600 = 176400 Ampere Second 
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

#define COULOMB_ETA (1.0f)


extern float BSSR_OCV[BSSR_OCV_DATA_SIZE];
extern float BSSR_SOC[BSSR_OCV_DATA_SIZE];

float SOC(float ocv){
    float dsoc = 0.0f;
    float docv = 0.0f;
    float soc = 0.0f;
    //using method of linear interpolation
    if(ocv <= BSSR_OCV[0]){
        soc = 0;
        return soc;
    }else if(ocv >= BSSR_OCV[LUT_SIZE-1]){
        soc = 1;
        return soc;
    }else if((ocv > BSSR_OCV[0])&&(ocv < BSSR_OCV[LUT_SIZE-1])){
    //binary search
    int lowerIndex = 0;
    int low = 0;
    int high = LUT_SIZE-1;
    int mid = 0;
    while(low <= high){
        mid = (low+high)/2;
        if(ocv>=BSSR_OCV[mid] && ocv<BSSR_OCV[mid+1]){
            lowerIndex = mid;
            break;
        } else if(ocv<BSSR_OCV[mid]){
            high = mid-1;
        } else {
            low = mid+1;
        }
    }
    dsoc = BSSR_SOC[lowerIndex+1]-BSSR_SOC[lowerIndex];
    docv = BSSR_OCV[lowerIndex+1]-BSSR_OCV[lowerIndex];
    soc = (ocv - BSSR_OCV[lowerIndex])*(dsoc/docv)+BSSR_SOC[lowerIndex];
    return soc;
    }else{
        return 0;
    }
}

void initBatteryAlgo(EKF_Model_14p* inBattery, float initial_v, float initial_deltaT){

        initEKFModel(inBattery, initial_v);
        compute_A_B_dt(inBattery, initial_deltaT);

    return;
}

void initEKFModel(EKF_Model_14p* inBattery, float initial_v){

	inBattery->run_EKF = run_EKF;

    uint8_t i = 0;  
    uint8_t j = 0;

    // initialize all helper matrix
    inBattery->matrix.dim1[0] = 3;
    inBattery->matrix.dim1[1] = 3;
    //dim2 {3, 1}
    inBattery->matrix.dim2[0] = 3;
    inBattery->matrix.dim2[1] = 1;
    //dim3 {1, 3}
    inBattery->matrix.dim3[0] = 1;
    inBattery->matrix.dim3[1] = 3;
    //dim4 {1, 1}
    inBattery->matrix.dim4[0] = 1;
    inBattery->matrix.dim4[1] = 1;
    //dim5 {3, 2}
    inBattery->matrix.dim5[0] = 3;
    inBattery->matrix.dim5[1] = 2;
    //dim6 {2, 1}
    inBattery->matrix.dim6[0] = 2;
    inBattery->matrix.dim6[1] = 1;
    //dim7 {1, 2}
    inBattery->matrix.dim7[0] = 1;
    inBattery->matrix.dim7[1] = 2;

    // a priori state covariance matrix - k+1|k (prediction)
    //float P_k[STATE_NUM*STATE_NUM] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    inBattery->matrix.P_k[0] = 0.0f;
    inBattery->matrix.P_k[1] = 0.0f;
    inBattery->matrix.P_k[2] = 0.0f;
    inBattery->matrix.P_k[3] = 0.0f;
    inBattery->matrix.P_k[4] = 0.0f;
    inBattery->matrix.P_k[5] = 0.0f;
    inBattery->matrix.P_k[6] = 0.0f;
    inBattery->matrix.P_k[7] = 0.0f;
    inBattery->matrix.P_k[8] = 0.0f;
    // noise in state measurement (sampled) - expected value of the distribution
    //float Q[STATE_NUM*STATE_NUM] = {VAR_INPT, 0.0f, 0.0f, 0.0f, VAR_INPT, 0.0f, 0.0f, 0.0f, VAR_INPT};
    inBattery->matrix.Q[0] = VAR_INPT;
    inBattery->matrix.Q[1] = 0.0f;
    inBattery->matrix.Q[2] = 0.0f;
    inBattery->matrix.Q[3] = 0.0f;
    inBattery->matrix.Q[4] = VAR_INPT;
    inBattery->matrix.Q[5] = 0.0f;
    inBattery->matrix.Q[6] = 0.0f;
    inBattery->matrix.Q[7] = 0.0f;
    inBattery->matrix.Q[8] = VAR_INPT;
    // noise in output sensor (sampled) - expected value of the distribution
    //float R[OUTPUT_NUM] = {VAR_SENS};
    inBattery->matrix.R[0] = VAR_SENS;
    // a priori measurement covariance
    //float S[OUTPUT_NUM] = {0.0f};
    inBattery->matrix.S[0] = 0.0f;
    // Kalman Weight
    //float W[STATE_NUM*OUTPUT_NUM] = {0.0f, 0.0f, 0.0f};
    inBattery->matrix.W[0] = 0.0f;
    inBattery->matrix.W[1] = 0.0f;
    inBattery->matrix.W[2] = 0.0f;

    //float covList[STATE_NUM] = {VAR_Z, VAR_I_CT, VAR_I_D};
    inBattery->matrix.covList[0] = VAR_Z;
    inBattery->matrix.covList[1] = VAR_I_CT;
    inBattery->matrix.covList[2] = VAR_I_D;
    //float A[STATE_NUM*STATE_NUM] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    inBattery->matrix.A[0] = 0.0f;
    inBattery->matrix.A[1] = 0.0f;
    inBattery->matrix.A[2] = 0.0f;
    inBattery->matrix.A[3] = 0.0f;
    inBattery->matrix.A[4] = 0.0f;
    inBattery->matrix.A[5] = 0.0f;
    inBattery->matrix.A[6] = 0.0f;
    inBattery->matrix.A[7] = 0.0f;
    inBattery->matrix.A[8] = 0.0f;
    //float B[STATE_NUM*INPUT_NUM] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    inBattery->matrix.B[0] = 0.0f;
    inBattery->matrix.B[1] = 0.0f;
    inBattery->matrix.B[2] = 0.0f;
    inBattery->matrix.B[3] = 0.0f;
    inBattery->matrix.B[4] = 0.0f;
    inBattery->matrix.B[5] = 0.0f;
    //float C[STATE_NUM*OUTPUT_NUM] = {0, -R_CT, -R_D};
    inBattery->matrix.C[0] = 0.0f;
    inBattery->matrix.C[1] = -R_CT;
    inBattery->matrix.C[2] = -R_D;
    //float D[INPUT_NUM] = {-R_INT, 0.0f};
    inBattery->matrix.D[0] = -R_INT;
    inBattery->matrix.D[1] = 0.0f;
    //float U[INPUT_NUM] = {0.0f, 0.0f};
    inBattery->matrix.U[0] = 0.0f;
    inBattery->matrix.U[1] = 0.0f;
    //float V_Measured[1] = {0.0f};
    inBattery->matrix.V_Measured[0] = 0.0f;
    //float V_OCV[1] = {0.0f};
    inBattery->matrix.V_OCV[0] = 0.0f;

// ------------- HELPER VARIABLES TO COMPUTE INVERSES -------------

    //float A_T[STATE_NUM*STATE_NUM] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    inBattery->matrix.A_T[0] = 0.0f;
    inBattery->matrix.A_T[1] = 0.0f;
    inBattery->matrix.A_T[2] = 0.0f;
    inBattery->matrix.A_T[3] = 0.0f;
    inBattery->matrix.A_T[4] = 0.0f;
    inBattery->matrix.A_T[5] = 0.0f;
    inBattery->matrix.A_T[6] = 0.0f;
    inBattery->matrix.A_T[7] = 0.0f;
    inBattery->matrix.A_T[8] = 0.0f;
    //float A_P[STATE_NUM*STATE_NUM] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    inBattery->matrix.A_P[0] = 0.0f;
    inBattery->matrix.A_P[1] = 0.0f;
    inBattery->matrix.A_P[2] = 0.0f;
    inBattery->matrix.A_P[3] = 0.0f;
    inBattery->matrix.A_P[4] = 0.0f;
    inBattery->matrix.A_P[5] = 0.0f;
    inBattery->matrix.A_P[6] = 0.0f;
    inBattery->matrix.A_P[7] = 0.0f;
    inBattery->matrix.A_P[8] = 0.0f;
    //float A_P_AT[STATE_NUM*STATE_NUM] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    inBattery->matrix.A_P_AT[0] = 0.0f;
    inBattery->matrix.A_P_AT[1] = 0.0f;
    inBattery->matrix.A_P_AT[2] = 0.0f;
    inBattery->matrix.A_P_AT[3] = 0.0f;
    inBattery->matrix.A_P_AT[4] = 0.0f;
    inBattery->matrix.A_P_AT[5] = 0.0f;
    inBattery->matrix.A_P_AT[6] = 0.0f;
    inBattery->matrix.A_P_AT[7] = 0.0f;
    inBattery->matrix.A_P_AT[8] = 0.0f;

    //float P_k1[STATE_NUM*STATE_NUM] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    inBattery->matrix.P_k1[0] = 0.0f;
    inBattery->matrix.P_k1[1] = 0.0f;
    inBattery->matrix.P_k1[2] = 0.0f;
    inBattery->matrix.P_k1[3] = 0.0f;
    inBattery->matrix.P_k1[4] = 0.0f;
    inBattery->matrix.P_k1[5] = 0.0f;
    inBattery->matrix.P_k1[6] = 0.0f;
    inBattery->matrix.P_k1[7] = 0.0f;
    inBattery->matrix.P_k1[8] = 0.0f;

    //float C_T[STATE_NUM*OUTPUT_NUM] = {0.0f, 0.0f, 0.0f};
    inBattery->matrix.C_T[0] = 0.0f;
    inBattery->matrix.C_T[1] = 0.0f;
    inBattery->matrix.C_T[2] = 0.0f;
    //float C_P[STATE_NUM*OUTPUT_NUM] = {0.0f, 0.0f, 0.0f};
    inBattery->matrix.C_P[0] = 0.0f;
    inBattery->matrix.C_P[1] = 0.0f;
    inBattery->matrix.C_P[2] = 0.0f;
    //float C_P_CT[OUTPUT_NUM*OUTPUT_NUM] = {0.0f};
    inBattery->matrix.C_P_CT[0] = 0.0f;

    //float SInv[OUTPUT_NUM*OUTPUT_NUM] = {0.0f};
    inBattery->matrix.SInv[0] = 0.0f;

    //float P_CT[STATE_NUM*OUTPUT_NUM] = {0.0f, 0.0f, 0.0f};
    inBattery->matrix.P_CT[0] = 0.0f;
    inBattery->matrix.P_CT[1] = 0.0f;
    inBattery->matrix.P_CT[2] = 0.0f;

    //float W_T[OUTPUT_NUM*STATE_NUM] = {0.0f, 0.0f, 0.0f};
    inBattery->matrix.W_T[0] = 0.0f;
    inBattery->matrix.W_T[1] = 0.0f;
    inBattery->matrix.W_T[2] = 0.0f;

    //float W_S[STATE_NUM*OUTPUT_NUM] = {0.0f, 0.0f, 0.0f};
    inBattery->matrix.W_S[0] = 0.0f;
    inBattery->matrix.W_S[1] = 0.0f;
    inBattery->matrix.W_S[2] = 0.0f;
    //float W_S_WT[STATE_NUM*STATE_NUM] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    inBattery->matrix.W_S_WT[0] = 0.0f;
    inBattery->matrix.W_S_WT[1] = 0.0f;
    inBattery->matrix.W_S_WT[2] = 0.0f;
    inBattery->matrix.W_S_WT[3] = 0.0f;
    inBattery->matrix.W_S_WT[4] = 0.0f;
    inBattery->matrix.W_S_WT[5] = 0.0f;
    inBattery->matrix.W_S_WT[6] = 0.0f;
    inBattery->matrix.W_S_WT[7] = 0.0f;
    inBattery->matrix.W_S_WT[8] = 0.0f;

    //float A_X[STATE_NUM] = {0.0f, 0.0f, 0.0f};
    inBattery->matrix.A_X[0] = 0.0f;
    inBattery->matrix.A_X[1] = 0.0f;
    inBattery->matrix.A_X[2] = 0.0f;
    //float B_U[STATE_NUM*INPUT_NUM] = {0.0f, 0.0f}; //might be an error because the size should be 3X1
    inBattery->matrix.B_U[0] = 0.0f;
    inBattery->matrix.B_U[1] = 0.0f;
    inBattery->matrix.B_U[2] = 0.0f;
    inBattery->matrix.B_U[3] = 0.0f;
    inBattery->matrix.B_U[4] = 0.0f;
    inBattery->matrix.B_U[5] = 0.0f;

    //float X_k1[STATE_NUM] = {0.0f, 0.0f, 0.0f};
    inBattery->matrix.X_k1[0] = 0.0f;
    inBattery->matrix.X_k1[1] = 0.0f;
    inBattery->matrix.X_k1[2] = 0.0f;
    //float Z_k1[OUTPUT_NUM] = {0.0f};
    inBattery->matrix.Z_k1[0] = 0.0f;
    //float CX_DU[OUTPUT_NUM] = {0.0f};
    inBattery->matrix.CX_DU[0] = 0.0f;
    //float C_X[OUTPUT_NUM] = {0.0f};
    inBattery->matrix.C_X[0] = 0.0f;
    //float D_U[OUTPUT_NUM] = {0.0f};
    inBattery->matrix.D_U[0] = 0.0f;

    //float Z_err[OUTPUT_NUM] = {0.0f};
    inBattery->matrix.Z_err[0] = 0.0f;
	//float W_Zerr[STATE_NUM] = {0.0f};
    inBattery->matrix.W_Zerr[0] = 0.0f;
    inBattery->matrix.W_Zerr[1] = 0.0f;
    inBattery->matrix.W_Zerr[2] = 0.0f;

    // initialize state variables
    for (i = 0; i < STATE_NUM; i++){
        if(i == 0U){
            inBattery->stateX[i] = SOC(initial_v);
        } else {
            inBattery->stateX[i] = 0.0f;
        }
    }

    // initialize the covariance matrix
    for(i = 0; i < STATE_NUM; i++){
        for(j = 0; j < STATE_NUM; j++){
            
            if (i == j){
                inBattery->covP[i*STATE_NUM + j] = inBattery->matrix.covList[i];
            } else {
                inBattery->covP[i*STATE_NUM + j] = 0.0f;
            }
        }
    }

    return;
}
/*
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
*/
void compute_A_B_dt(EKF_Model_14p* inBattery, float dt){

    inBattery->matrix.A[0] = 1.0f;
    inBattery->matrix.A[4] = exp(-dt/(R_CT*C_CT));
    inBattery->matrix.A[8] = exp(-dt/(R_D*C_D));

    inBattery->matrix.B[0] = ((-COULOMB_ETA)*(dt)/Q_CAP); //in example, this equation is only multiplied by ETA when the current is negative
    inBattery->matrix.B[2] = 1 - exp(-dt/(R_CT*C_CT));
    inBattery->matrix.B[4] = 1 - exp(-dt/(R_D*C_D));

    return;
}

void addition_EKF(float* operand_1, float* operand_2, float* result, uint8_t* size,  uint8_t subtract){

    // check if we are doing addition or subtraction based on the 'subtract' boolean
    // if subtract = 0 then it is addition
    // if subtract = 1 then it is subraction
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

    // opDim_1 and opDim_2 contain the dimensions of operand_1 and operand_2 in an array of size 2
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
        return 0;
    }

    if(1 == rows){

        out[0] = 1.0f/in[0];
        return 1;

    } else {

        // create a copy of the current input
        float* copyIn = (float*)calloc(rows*cols, sizeof(float)); //need to change to stm dynamic memory allocation function
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
                    out[i*cols+k] -= out[j*cols+k] * ratio;
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

        //Normalization:
        for (uint16_t i = 0; i < rows; i++) {
            float temp = copyIn[i*cols+i];
            copyIn[i*cols+i]= 1;

            for (uint16_t j = 0; j < cols; j++){
                out[i*cols+j] /= temp;
            }
        }

       free(copyIn);

    }
    return 1;
}

float OCV(float soc){

    float dv = 0.0f;
    float dsoc = BSSR_SOC[1]- BSSR_SOC[0];
    float ocv = 0.0f;

    //using method of linear interpolation
    if(soc <= BSSR_SOC[0]){

        dv = BSSR_OCV[1]-BSSR_OCV[0];
        ocv = (soc - BSSR_SOC[0])*(dv/dsoc)+BSSR_OCV[0];
        return ocv;

    }else if(soc >= BSSR_SOC[LUT_SIZE-1]){

        dv = BSSR_OCV[LUT_SIZE-1]-BSSR_OCV[LUT_SIZE-2];
        ocv = (soc - BSSR_SOC[LUT_SIZE-1])*(dv/dsoc)+BSSR_OCV[LUT_SIZE-1];
        return ocv;

    }else if((soc>BSSR_SOC[0])&&(soc<BSSR_SOC[LUT_SIZE-1])){

        int lowerIndex = (soc - BSSR_SOC[0])/dsoc;
        dv = BSSR_OCV[lowerIndex+1]-BSSR_OCV[lowerIndex];
        ocv = BSSR_OCV[lowerIndex] + ((soc - BSSR_SOC[lowerIndex])*(dv/dsoc));
        return ocv;

    }else{
        return 0;
    }
}

void run_EKF(EKF_Model_14p* inputBatt, float dt, float currentIn, float measuredV){ //deltat, current same for all unit, voltage different for each unit

		// insert code for using APIs
		    compute_A_B_dt(inputBatt, dt);

		    float I_Input = currentIn; // current reading
		    float I_InSign = 0.0f;

		    if (I_Input != 0){
		        I_InSign = (I_Input > 0.0f) ? 1.0f : -1.0f;
		    }
		    inputBatt->matrix.U[0] = I_Input;
		    inputBatt->matrix.U[1] = I_InSign;

		    inputBatt->matrix.V_Measured[0] = measuredV; // voltage reading
		    inputBatt->matrix.V_OCV[0] = OCV(inputBatt->stateX[0]);

		    // compute the a priori state covariance
		    transpose_EKF(inputBatt->matrix.A, inputBatt->matrix.A_T, inputBatt->matrix.dim1);
		    multiply_EKF(inputBatt->matrix.A, inputBatt->covP, inputBatt->matrix.A_P, inputBatt->matrix.dim1, inputBatt->matrix.dim1);
		    multiply_EKF(inputBatt->matrix.A_P, inputBatt->matrix.A_T,inputBatt->matrix.A_P_AT, inputBatt->matrix.dim1, inputBatt->matrix.dim1);
		    addition_EKF(inputBatt->matrix.A_P_AT, inputBatt->matrix.Q, inputBatt->matrix.P_k1, inputBatt->matrix.dim1, 0);

		    // compute the a priori state covariance
		    transpose_EKF(inputBatt->matrix.C, inputBatt->matrix.C_T, inputBatt->matrix.dim3);
		    multiply_EKF(inputBatt->matrix.C, inputBatt->matrix.P_k1, inputBatt->matrix.C_P, inputBatt->matrix.dim3, inputBatt->matrix.dim1);
		    multiply_EKF(inputBatt->matrix.C_P, inputBatt->matrix.C_T, inputBatt->matrix.C_P_CT, inputBatt->matrix.dim3, inputBatt->matrix.dim2);
		    addition_EKF(inputBatt->matrix.C_P_CT, inputBatt->matrix.R, inputBatt->matrix.S, inputBatt->matrix.dim4, 0);

		    // compute Kalman Gain
		    inverse_EKF(inputBatt->matrix.S, inputBatt->matrix.SInv, inputBatt->matrix.dim4);
		    multiply_EKF(inputBatt->matrix.P_k1, inputBatt->matrix.C_T, inputBatt->matrix.P_CT, inputBatt->matrix.dim1, inputBatt->matrix.dim2);
		    multiply_EKF(inputBatt->matrix.P_CT, inputBatt->matrix.SInv, inputBatt->matrix.W, inputBatt->matrix.dim2, inputBatt->matrix.dim4);

		    // update Posteriori covariance
		    transpose_EKF(inputBatt->matrix.W, inputBatt->matrix.W_T, inputBatt->matrix.dim2);
		    multiply_EKF(inputBatt->matrix.W, inputBatt->matrix.S, inputBatt->matrix.W_S, inputBatt->matrix.dim2, inputBatt->matrix.dim4);
		    multiply_EKF(inputBatt->matrix.W_S, inputBatt->matrix.W_T, inputBatt->matrix.W_S_WT, inputBatt->matrix.dim2, inputBatt->matrix.dim3);
		    addition_EKF(inputBatt->matrix.P_k1, inputBatt->matrix.W_S_WT, inputBatt->covP, inputBatt->matrix.dim1, 1);


		    // a priori state estimate
		    multiply_EKF(inputBatt->matrix.A, inputBatt->stateX, inputBatt->matrix.A_X, inputBatt->matrix.dim1, inputBatt->matrix.dim2);
		    multiply_EKF(inputBatt->matrix.B, inputBatt->matrix.U, inputBatt->matrix.B_U, inputBatt->matrix.dim5, inputBatt->matrix.dim6);
		    addition_EKF(inputBatt->matrix.A_X, inputBatt->matrix.B_U, inputBatt->matrix.X_k1, inputBatt->matrix.dim2, 0); //changed

		    // a priori measurement
		    multiply_EKF(inputBatt->matrix.C, inputBatt->matrix.X_k1, inputBatt->matrix.C_X, inputBatt->matrix.dim3, inputBatt->matrix.dim2); //changed
		    multiply_EKF(inputBatt->matrix.D, inputBatt->matrix.U, inputBatt->matrix.D_U, inputBatt->matrix.dim7, inputBatt->matrix.dim6);
		    addition_EKF(inputBatt->matrix.C_X, inputBatt->matrix.D_U, inputBatt->matrix.CX_DU, inputBatt->matrix.dim4, 0);
		    addition_EKF(inputBatt->matrix.V_OCV, inputBatt->matrix.CX_DU, inputBatt->matrix.Z_k1, inputBatt->matrix.dim4, 0);

		    // update of the posteriori state estimate
		    addition_EKF(inputBatt->matrix.V_Measured, inputBatt->matrix.Z_k1, inputBatt->matrix.Z_err, inputBatt->matrix.dim4, 1); //changed
		    multiply_EKF(inputBatt->matrix.W, inputBatt->matrix.Z_err, inputBatt->matrix.W_Zerr, inputBatt->matrix.dim2, inputBatt->matrix.dim4);
		    addition_EKF(inputBatt->matrix.X_k1, inputBatt->matrix.W_Zerr, inputBatt->stateX, inputBatt->matrix.dim2, 0);    // SOC in inputBatt->stateX[0]
}
