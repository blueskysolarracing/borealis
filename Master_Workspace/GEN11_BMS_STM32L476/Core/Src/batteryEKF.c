#include "batteryEKF.h"

#if _WINDOWS
void printMatrix(float* input, uint8_t* size){

    for(int i = 0; i < size[0]; i++){
        for(int j = 0; j < size[1]; j++){
            printf("%.8f  ", input[i*size[1] + j]);
        }
        printf("\n");
    }

}
#endif

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

void initBatteryAlgo(EKF_Battery* inBatteryPack, float* initial_v, float initial_deltaT){

    for(uint8_t unit = 0; unit < NUM_14P_UNITS; unit++){  
        initEKFModel(&inBatteryPack->batteryPack[unit], initial_v[unit]);
    }

    compute_A_B_dt(initial_deltaT);

    return;
}

void initEKFModel(EKF_Model_14p* inBattery, float initial_v){

    uint8_t i = 0;  
    uint8_t j = 0;

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

#if DEBUG_PRINTS
    printf("A initial\n");
    printMatrix(A, dim1);
    printf("\n");
#endif

    return;
}

void init_B_Matrix(){

    B[0] = -COULOMB_ETA*DELTA_T/Q_CAP; //in example, this equation is only multiplied by ETA when the current is negative
    B[2] = 1 - exp(-DELTA_T/(R_CT*C_CT));
    B[4] = 1 - exp(-DELTA_T/(R_D*C_D));

#if DEBUG_PRINTS
    printf("B initial\n");
    printMatrix(B, dim5);
    printf("\n");
#endif

    return;
}

void compute_A_B_dt(float dt){

    A[0] = 1.0f;
    A[4] = exp(-dt/(R_CT*C_CT));
    A[8] = exp(-dt/(R_D*C_D));

    B[0] = -COULOMB_ETA*dt/Q_CAP; //in example, this equation is only multiplied by ETA when the current is negative
    B[2] = 1 - exp(-dt/(R_CT*C_CT));
    B[4] = 1 - exp(-dt/(R_D*C_D));

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
#if _WINDOWS
        printf("Matrix is not square - cannot compute inverse with this method... \n");
#endif
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

#if OFFLINE_TEST

void run_EKF(EKF_Model_14p* inputBatt, uint32_t testDataID){

    // insert code for reading from CSVs
#if DEBUG_PRINTS
    printf("Iteration#%d\n", testDataID);
    printf("Current %f, Voltage %f \n", current[testDataID], voltage[testDataID]);
#endif

    float dt = deltaT[testDataID];
    compute_A_B_dt(dt);

    float I_Input = current[testDataID]; // current reading
    float I_InSign = 0.0f;
    
    if (I_Input != 0){
        I_InSign = (I_Input > 0.0f) ? 1.0f : -1.0f;
    }

    U[0] = I_Input;
    U[1] = I_InSign;

    V_Measured[0] = voltage[testDataID]; // voltage reading
    V_OCV[0] = OCV(inputBatt->stateX[0]);

#if DEBUG_PRINTS
    printf("OCV: %f | %f \n", V_OCV[0], inputBatt->stateX[0]);
    printf("U\n");
    printMatrix(U, dim6);
    printf("A\n");
    printMatrix(A, dim1);
    printf("B\n");
    printMatrix(B, dim5);
    printf("stateX\n");
    printMatrix(inputBatt->stateX, dim2);
    printf("covP\n");
    printMatrix(inputBatt->covP, dim1);
#endif

#else

void run_EKF(EKF_Model_14p* inputBatt, float dt, float currentIn, float measuredV){

    // insert code for using APIs
    compute_A_B_dt(dt);

    float I_Input = currentIn; // current reading
    float I_InSign = 0.0f;

    if (I_Input != 0){
        I_InSign = (I_Input > 0.0f) ? 1.0f : -1.0f;
    }

    V_Measured[0] = measuredV; // voltage reading
    V_OCV[0] = OCV(inputBatt->stateX[0]);

#endif // OFFLINE_TEST

    // compute the a priori state covariance
    transpose_EKF(A, A_T, dim1);

#if DEBUG_PRINTS
    printf("\nA_T\n");
    printMatrix(A_T, dim1);
#endif

    multiply_EKF(A, inputBatt->covP, A_P, dim1, dim1);

#if DEBUG_PRINTS
    printf("\nA_P\n");
    printMatrix(A_P, dim1);
#endif

    multiply_EKF(A_P, A_T, A_P_AT, dim1, dim1);

#if DEBUG_PRINTS
    printf("\nA_P_AT\n");
    printMatrix(A_P_AT, dim1);
#endif

    addition_EKF(A_P_AT, Q, P_k1, dim1, 0);

#if DEBUG_PRINTS
    printf("\nP_k1\n");
    printMatrix(P_k1, dim1);
#endif

    // compute the a priori state covariance
    transpose_EKF(C, C_T, dim3);

#if DEBUG_PRINTS
    printf("\nC\n");
    printMatrix(C, dim3);
    printf("\nC_T\n");
    printMatrix(C_T, dim2);
#endif

    multiply_EKF(C, P_k1, C_P, dim3, dim1);

#if DEBUG_PRINTS
    printf("\nC_P\n");
    printMatrix(C_P, dim3);
#endif

    multiply_EKF(C_P, C_T, C_P_CT, dim3, dim2);

#if DEBUG_PRINTS
    printf("\nC_P_CT\n");
    printMatrix(C_P_CT, dim4);
#endif

    addition_EKF(C_P_CT, R, S, dim4, 0);

#if DEBUG_PRINTS
    printf("\nS\n");
    printMatrix(S, dim4);
#endif

    // compute Kalman Gain
    inverse_EKF(S, SInv, dim4);

#if DEBUG_PRINTS
    printf("S, S_inv: %f, %f",S[0], SInv[0]);
#endif

    multiply_EKF(P_k1, C_T, P_CT, dim1, dim2);

#if DEBUG_PRINTS
    printf("\nP_CT\n");
    printMatrix(P_CT, dim2);
#endif

    multiply_EKF(P_CT, SInv, W, dim2, dim4);

#if DEBUG_PRINTS
    printf("\nW\n");
    printMatrix(W, dim2);
#endif

    // update Posteriori covariance
    transpose_EKF(W, W_T, dim2);

#if DEBUG_PRINTS
    printf("\nW_T\n");
    printMatrix(W_T, dim3);
#endif

    multiply_EKF(W, S, W_S, dim2, dim4);

#if DEBUG_PRINTS
    printf("\nW_S\n");
    printMatrix(W_S, dim2);
#endif

    multiply_EKF(W_S, W_T, W_S_WT, dim2, dim3);

#if DEBUG_PRINTS
    printf("\nW_S_WT\n");
    printMatrix(W_S_WT, dim1);
#endif

    addition_EKF(P_k1, W_S_WT, inputBatt->covP, dim1, 1);

#if DEBUG_PRINTS
    printf("\ninputBatt->covP\n");
    printMatrix(inputBatt->covP, dim1);
#endif

    // a priori state estimate
    multiply_EKF(A, inputBatt->stateX, A_X, dim1, dim2);

#if DEBUG_PRINTS
    printf("\nA_X\n");
    printMatrix(A_X, dim2);
#endif

#if DEBUG_PRINTS
    printf("\nU\n");
    printMatrix(U, dim6);
#endif

    multiply_EKF(B, U, B_U, dim5, dim6);

#if DEBUG_PRINTS
    printf("\nB_U\n");
    printMatrix(B_U, dim2);
#endif

    addition_EKF(A_X, B_U, X_k1, dim3, 0);

#if DEBUG_PRINTS
    printf("\nX_k1\n");
    printMatrix(X_k1, dim2);
#endif

    // a priori measurement
    multiply_EKF(C, X_k1, C_X, dim4, dim3);

#if DEBUG_PRINTS
    printf("Matrix C_X \n");
    printMatrix(C_X, dim4);
#endif

    multiply_EKF(D, U, D_U, dim7, dim6);

#if DEBUG_PRINTS
    printf("Matrix D_U \n");
    printMatrix(D_U, dim4);
#endif

    addition_EKF(C_X, D_U, CX_DU, dim4, 0);
    addition_EKF(V_OCV, CX_DU, Z_k1, dim4, 0);

#if DEBUG_PRINTS
    printf("Predicted output Z_k1: ");
    printMatrix(Z_k1, dim4);
#endif

    // update of the posteriori state estimate
    addition_EKF(V_Measured, Z_k1, Z_err, dim5, 1);

#if DEBUG_PRINTS
    printf("\nZ_err\n");
    printMatrix(Z_err, dim4);
#endif

    multiply_EKF(W, Z_err, W_Zerr, dim2, dim4);

#if DEBUG_PRINTS
    printf("\nW_Zerr\n");
    printMatrix(W_Zerr, dim2);
#endif

    addition_EKF(X_k1, W_Zerr, inputBatt->stateX, dim2, 0);    // SOC in inputBatt->stateX[0]

#if DEBUG_PRINTS
    printf("\nStateX\n");
    printMatrix(inputBatt->stateX, dim2);
#endif

    return;
}
