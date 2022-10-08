#include <stdio.h> 
#include <time.h> 
#include <stdlib.h> 
#include <stdint.h> 
 
#define BBMB_ADDRESS 0x01 
#define PPTMB_ADDRESS 0x02 
#define MCMB_ADDRESS 0x03 
#define DCMB_ADDRESS 0x04 
#define BSSR_SERIAL_START 0xa5 
#define BSSR_SERIAL_ESCAPE 0x5a 
 
#define MAX_PACKET_SIZE 256 
#define HEADER_LENGTH 4 
#define CRC_LENGTH 4 
 
/*PAYLOAD LENGTH HAS TO BE A MULTIPLE OF 4*/ 
//Data ID 00 
#define BUS_METRICS_LENGTH 20 
#define MC2_STATE_LENGTH 8 
//Data ID 01 
#define CELL_METRICS_LENGTH 20 //20 for each cell and there are 29 cell in total, single cell data send at once
#define PPT_METRICS_LENGTH 52 //16 for each ppt and there are 3 ptt in total + 4 non repeating initial bytes 
#define SPEED_PULSE_READING_LENGTH 4 
#define BBOX_STARTUP_LENGTH 4 
//Data ID 02 
#define BSD_LENGTH 4 
#define MOTOR_TEMPERATURE_LENGTH 4 
#define PPTBOX_STARTUP_LENGTH 4 
//Data ID 03 
#define BMS_MCU_STATUS_LENGTH 16 
#define LIGHT_STATE_LENGTH 4 
//Data ID 04 
#define HORN_STATE_LENGTH 4 
//Data ID 05
#define BMS_DATA_REQUEST_LENGTH 8
//Data ID 06
#define RELAY_STATE_LENGTH 4
//Data ID 0B
#define TEXT_STRING_LENGTH 4
//Data ID 0D 
#define LP_BUS_METRICS_LENGTH 20 
//Data ID 0E 
#define CORE_TEMP_LENGTH 4 
//Data ID 0F 
#define  HEARTBEAT_LENGTH 4 
 
//Data ID 00 
uint8_t Bus_Metrics_Payload[BUS_METRICS_LENGTH]; 
uint8_t MC2_State_Payload[MC2_STATE_LENGTH]; 
//Data ID 01 
uint8_t Cell_Metrics_Payload[CELL_METRICS_LENGTH]; 
uint8_t PPT_Metrics_Payload[PPT_METRICS_LENGTH]; 
uint8_t Speed_Pulse_Reading_Payload[SPEED_PULSE_READING_LENGTH]; 
uint8_t BBox_Startup_Payload[BBOX_STARTUP_LENGTH]; 
//Data ID 02 
uint8_t BSD_Payload[BSD_LENGTH]; 
uint8_t Motor_Temperature_Payload[MOTOR_TEMPERATURE_LENGTH]; 
uint8_t PPTBox_Startup_Payload[PPTBOX_STARTUP_LENGTH]; 
//Data ID 03 
uint8_t BMS_MCU_Status_Payload[BMS_MCU_STATUS_LENGTH]; 
uint8_t Light_State_Payload[LIGHT_STATE_LENGTH]; 
//Data ID 04 
uint8_t Horn_State_Payload[HORN_STATE_LENGTH];
//Data ID 05
uint8_t BMS_Data_request[BMS_DATA_REQUEST_LENGTH];
//Data ID 06
uint8_t Relay_State[RELAY_STATE_LENGTH];
//Data ID 0B
uint8_t Text_String[TEXT_STRING_LENGTH];
//Data ID 0D 
uint8_t LP_Bus_Metrics_Payload[LP_BUS_METRICS_LENGTH]; 
//Data ID 0E 
uint8_t Core_Temp_Payload[CORE_TEMP_LENGTH]; 
//Data ID 0F 
uint8_t Heartbeat_Payload[HEARTBEAT_LENGTH]; 
 
//SeqNum is [0,255] 
uint8_t BBMB_SeqNum = 0; 
uint8_t PPTMB_SeqNum = 0; 
uint8_t MCMB_SeqNum = 0; 
uint8_t DCMB_SeqNum = 0; 
 
/* DATA ASSUMPTIONS: 
 * Temp Range: 20-60 
 * Max Temp Range: 20-30 
 * Min Temp Range: 40-60 
 * Reference: Aymon 
 * 
 * Current Range: 0-2 (6 digits after decimal) 
 * Reference: Visual inspection from GenX test telemetry data 
 * 
 * Voltage Range: 2-5 (6 digits after decimal) 
 * Reference: Visual inspection from GenX test telemetry data 
 * 
 * Bus_Metrics_Generator: 
 *   samples of aggregate(0x03): 0x01 
 * Reference: GenX PPTMB -> main.c -> static void adcTask(const void *pv); 
 * 
 * PPT_Metrics_Generator: 
 *   PPT numbers: 7 
 *   # of Sample Aggregate: 0x01 
 * Reference: GenX PPTMB -> main.c -> static void adcTask(const void *pv); 
 * 
 * Speed_Pulse_Reading_And_MOT_LED_State_Generator: 
 *   MOT_LED state: 0-1 
 *   Speed_pulse Hz(uint16_t count): 0-255 
 * Reference: GenX MCMB -> main.c -> static void spdTmr(TimerHandle_t xTimer); 
 * 
 * MC2_State_Generator: 
 *   5 digital Buttons: 00000-11110 = 0-30 
 *   Acc Pot (outputVal): 0 or 0xff 
 * Reference: GenX DCMB -> main.c -> static void mc2StateTmr(TimerHandle_t xTimer); 
 * 
 * BBox_Startup_Generator: 
 *   State (ignition_state): 0-1 
 * Reference: GenX DCMB -> main.c -> void ignition_check(uint8_t data); 
 * 
 * PPTBox_Startup_Generator: 
 *   State (array_state): 0-1 
 * Reference: GenX DCMB -> main.c -> void ignition_check(uint8_t data); 
 * 
 * Light_State_Generator: 
 *   left, right, brake: 0-1 
 * Reference: GenX DCMB -> main.c -> static void lightsTmr(TimerHandle_t xTimer); 
 * 
 * Horn_State_Generator: 
 *   State (horn_on): 0-1 
 * Reference: GenX DCMB -> main.c -> static void buttonCheck(uint8_t state); 
 * */ 
 
//Generate a random value in [max, min] 
//One byte can express number 0-255, thus max<=255 && min>=0 && max>min 
uint8_t getRandomValue(int min, int max){
     if (max <= 255 && min>=0 && max>min) { 
        uint8_t r = rand() % (max + 1 - min) + min; 
        return r; 
    } 
} 
 
//Insert num of random value in p starting at p[startPos] 
void insertRandomValue(uint8_t* p, int startPos, int num, int min, int max){ 
    for(int i = startPos; i<startPos+num; i++){ 
        uint8_t r = getRandomValue(min, max); 
        *(p+i) = r; 
    } 
} 
 
//Data ID 00 
void Bus_Metrics_Generator(uint8_t* p){ 
    *p = 0x00; 
    *(p+1) = 0x00; 
    *(p+2) = 0x00; 
    *(p+3) = 0x01; 
    insertRandomValue(p, 4, 8, 0, 255); 
    *(p+12) = 0x00; 
    *(p+13) = 0x00; 
    *(p+14) = 0x00; 
    *(p+15) = 0x00; 
    *(p+16) = 0x00; 
    *(p+17) = 0x00; 
    *(p+18) = 0x00; 
    *(p+19) = 0x00; 
} 
void MC2_State_Generator(uint8_t* p){ 
    *p = 0x00; 
    insertRandomValue(p, 1, 1, 0, 30); 
    *(p+2) = 0xff * getRandomValue(0,1); 
    *(p+3) = 0x00; 
    *(p+4) = 0x00; 
    *(p+5) = 0x00; 
    *(p+6) = 0x00; 
    *(p+7) = 0x00; 
} 
//Data ID 01 
void Cell_Metrics_Generator(uint8_t* p, uint8_t cellNum){ 
    *(p) = 0x01; 
    *(p+1) = cellNum; 
    *(p+2) = 0x00; 
    *(p+3) = 0x01; 
    insertRandomValue(p, 4, 16, 0, 255); 
} 
void PPT_Metrics_Generator(uint8_t* p){ 
    *p = 0x01; 
    *(p+1) = 7; 
    *(p+2) = 0x00; 
    *(p+3) = 0x01; 
    for(int i =0; i<3; i++){ 
        *(p+4+i*16) = 0x00; 
        *(p+5+i*16) = 0x00; 
        *(p+6+i*16) = 0x00; 
        *(p+7+i*16) = 0x00; 
        insertRandomValue(p, 8+i*16, 4, 0, 255); 
        *(p+12+i*16) = 0x00; 
        *(p+13+i*16) = 0x00; 
        *(p+14+i*16) = 0x00; 
        *(p+15+i*16) = 0x00; 
        *(p+16+i*16) = 0x00;
        *(p+17+i*16) = 0x00; 
        *(p+18+i*16) = 0x00; 
        *(p+19+i*16) = 0x00; 
    } 
} 
void Speed_Pulse_Reading_Generator(uint8_t* p){ 
    *p = 0x01; 
    insertRandomValue(p, 1, 1, 0, 255); 
    *(p+2) = 0x00; 
    *(p+3) = 0x00; 
} 
void BBox_Startup_Generator(uint8_t* p){ 
    *p = 0x01; 
    insertRandomValue(p, 1, 1, 0, 1); 
    *(p+2) = 0x00; 
    *(p+3) = 0x00; 
} 
//Data ID 02 
void BSD_Generator(uint8_t* p){ 
    *p = 0x02; 
    insertRandomValue(p, 1, 3, 0, 255); 
} 
void Motor_Temperature_Generator(uint8_t* p){ 
    *p = 0x02; 
    insertRandomValue(p, 1, 1, 0, 40); 
    *(p+2) = 0x00; 
    *(p+3) = 0x00; 
} 
void PPTBox_Startup_Generator(uint8_t* p){ 
    *p = 0x02; 
    insertRandomValue(p, 1, 1, 0, 1); 
    *(p+2) = 0x00; 
    *(p+3) = 0x00; 
} 
//Data ID 03 
void BMS_MCU_Status_Generator(uint8_t* p){ 
    *p = 0x03; 
    insertRandomValue(p, 1, 15, 0, 255); 
} 
void Light_State_Generator(uint8_t* p){ 
    *p = 0x03; 
    insertRandomValue(p, 1, 3, 0, 1); 
} 
//Data ID 04 
void Horn_State_Generator(uint8_t* p){ 
    *p = 0x04; 
    insertRandomValue(p, 1, 1, 0, 1); 
    *(p+2) = 0x00; 
    *(p+3) = 0x00; 
} 
//Data ID 05
void BMS_Data_request_Generator(uint8_t* p){
    *p = 0x05;
    insertRandomValue(p,1,7,0,255); //insering random values for ID and current
}
//Data ID 06
void Relay_State_Generator(uint8_t* p){
    *p = 0x05;
    insertRandomValue(p,1,7,0,1); //inserting 0 or 1 for all the other values
}
//Data ID 0B
void Text_String_Generator(uint8_t* p){
    *p = 0x0b;
    insertRandomValue(p,1,7,0,26); //inserting random numbers from 0 26
}
//Data ID 0D 
void LP_Bus_Metrics_Generator(uint8_t* p){ 
    *p = 0x0d; 
    *(p+1) = 0x00; 
    *(p+2) = 0x00; 
    *(p+3) = 0x01; 
    insertRandomValue(p, 4, 8, 0, 255); 
    *(p+12) = 0x00; 
    *(p+13) = 0x00; 
    *(p+14) = 0x00; 
    *(p+15) = 0x00; 
    *(p+16) = 0x00; 
    *(p+17) = 0x00; 
    *(p+18) = 0x00;
     *(p+19) = 0x00; 
} 
//Data ID 0E 
void Core_Temp_Generator(uint8_t* p){ 
    *p = 0x0e; 
    insertRandomValue(p, 1, 3, 20, 60); 
} 
//Data ID 0F 
void Heartbeat_Generator(uint8_t* p){ 
    *p = 0x0f; 
    *(p+1) = 0x00; 
    *(p+2) = 0x00; 
    *(p+3) = 0x00; 
} 
 
void printPayload(uint8_t * p, int l) { 
    for (int i = 0; i < l; i++) { 
        printf("%3d ", *(p + i)); 
    } 
    printf("\n"); 
} 
 
void dummySend(uint8_t payloadLength, uint8_t senderAddress, uint8_t* seqNum, uint8_t* payload){ 
    uint8_t buf[HEADER_LENGTH + MAX_PACKET_SIZE + CRC_LENGTH] = {0}; 
 
    uint16_t buf_pos = 0; 
 
    buf[buf_pos] = BSSR_SERIAL_START; 
    buf_pos++; 
 
    if(payloadLength == BSSR_SERIAL_START || payloadLength == BSSR_SERIAL_ESCAPE){ 
        buf[buf_pos] = BSSR_SERIAL_ESCAPE; 
        buf_pos++; 
        buf[buf_pos] = payloadLength; 
        buf_pos++; 
    } else{ 
        buf[buf_pos] = payloadLength; 
        buf_pos++; 
    } 
 
    buf[buf_pos] = senderAddress; 
    buf_pos++; 
 
    if(*seqNum == BSSR_SERIAL_START || *seqNum == BSSR_SERIAL_ESCAPE){ 
        buf[buf_pos] = BSSR_SERIAL_ESCAPE; 
        buf_pos++; 
        buf[buf_pos] = *seqNum; 
        buf_pos++; 
    } else{ 
        buf[buf_pos] = *seqNum; 
        buf_pos++; 
    } 
    (*seqNum)++; 
 
    for(int i=0; i<payloadLength; i++){ 
        if(*(payload+i) == BSSR_SERIAL_ESCAPE || *(payload+i) == BSSR_SERIAL_START){ 
            buf[buf_pos] = BSSR_SERIAL_ESCAPE; 
            buf_pos++; 
            buf[buf_pos] = *(payload+i); 
            buf_pos++; 
        } else{ 
            buf[buf_pos] = *(payload+i); 
            buf_pos++; 
        }
        } 
 
    /*uint32_t crc_result = ~HAL_CRC_Calculate(&hcrc, (uint32_t*)buf, (uint32_t)buf_pos); 
    for(int i=0; i<4; i++){ 
        uint8_t crc = (crc_result>>(8*(3-i))) & 255; 
        if(crc == BSSR_SERIAL_ESCAPE || crc == BSSR_SERIAL_START){ 
            buf[buf_pos] = BSSR_SERIAL_ESCAPE; 
            buf_pos++; 
            buf[buf_pos] = crc; 
            buf_pos++; 
        } else{ 
            buf[buf_pos] = crc; 
            buf_pos++; 
        } 
    }*/ 
    //pretend crc 
    buf_pos +=4; 
 
    if(buf_pos%4 != 0) { 
        int paddingNum = 4 - buf_pos % 4; 
        for (int i = paddingNum; i > 0; i--) { 
            buf[buf_pos] = 0x00; 
            buf_pos++; 
        } 
    } 
 
    printPayload(buf, buf_pos); 
 
} 
 
void BBMB(){ 
 
    //There are 7 data types in BBMB 
    for(int i=0; i<7; i++){ 
 
        switch(i){ 
            case 0: //Send Bus Metrics Data 
            { 
                Bus_Metrics_Generator(Bus_Metrics_Payload); 
                dummySend(BUS_METRICS_LENGTH, BBMB_ADDRESS, &BBMB_SeqNum, Bus_Metrics_Payload); 
                break; 
            } 
            /*case 1: //Send Cell Metrics Data 
            { 
                for(int i=0; i<29; i++) { 
                    Cell_Metrics_Generator(Cell_Metrics_Payload, i+1); 
                    dummySend(CELL_METRICS_LENGTH, BBMB_ADDRESS, &BBMB_SeqNum, Cell_Metrics_Payload); 
                } 
                break; 
            } */
            case 2: //Send BSD Data 
            { 
                BSD_Generator(BSD_Payload); 
                dummySend(BSD_LENGTH, BBMB_ADDRESS, &BBMB_SeqNum, BSD_Payload); 
                break; 
            } 
            case 3: //Send BMS MCU Status Data 
            { 
                BMS_MCU_Status_Generator(BMS_MCU_Status_Payload); 
                dummySend(BMS_MCU_STATUS_LENGTH, BBMB_ADDRESS, &BBMB_SeqNum, BMS_MCU_Status_Payload); 
                break; 
            } 
            /*case 4: //Send LP Bus Metrics Data 
            {
                 LP_Bus_Metrics_Generator(LP_Bus_Metrics_Payload); 
                dummySend(LP_BUS_METRICS_LENGTH, BBMB_ADDRESS, &BBMB_SeqNum, LP_Bus_Metrics_Payload); 
                break; 
            } */
            case 5: //send BMS data request
            {
                BMS_Data_request_Generator(BMS_Data_request);
                dummySend(BMS_DATA_REQUEST_LENGTH, BBMB_ADDRESS, &BBMB_SeqNum, BMS_Data_request); 
                break;
            }
            case 6: //send relay state
            {
                Relay_State_Generator(Relay_State);
                dummySend(RELAY_STATE_LENGTH, BBMB_ADDRESS, &BBMB_SeqNum, Relay_State); 
                break;
            }
            case 11: //send text string
            {
                Text_String_Generator(Text_String);
                dummySend(TEXT_STRING_LENGTH, BBMB_ADDRESS, &BBMB_SeqNum, Text_String); 
                break;
            }
            case 13: //send LP Bus Metric
            {
                LP_Bus_Metrics_Generator(LP_Bus_Metrics_Payload);
                dummySend(LP_BUS_METRICS_LENGTH, BBMB_ADDRESS, &BBMB_SeqNum, LP_Bus_Metrics_Payload); 
                break;
            }
            case 14: //Send Core Temp Data 
            { 
                Core_Temp_Generator(Core_Temp_Payload); 
                dummySend(CORE_TEMP_LENGTH, BBMB_ADDRESS, &BBMB_SeqNum, Core_Temp_Payload); 
                break; 
            } 
            case 15: //Send Heartbeat Data 
            { 
                Heartbeat_Generator(Heartbeat_Payload); 
                dummySend(HEARTBEAT_LENGTH, BBMB_ADDRESS, &BBMB_SeqNum, Heartbeat_Payload); 
                break; 
            } 
        } 
    } 
} 
 
void PPTMB(){ 
 
    //There are 5 data types in PPTMB 
    for(int i=0; i<5; i++){ 
 
        switch(i){ 
            case 0: //Send Bus Metrics Data 
            { 
                Bus_Metrics_Generator(Bus_Metrics_Payload); 
                dummySend(BUS_METRICS_LENGTH, PPTMB_ADDRESS, &PPTMB_SeqNum, Bus_Metrics_Payload); 
                break; 
            } 
            case 1: //Send PPT Metrics Data 
            { 
                PPT_Metrics_Generator(PPT_Metrics_Payload); 
                dummySend(PPT_METRICS_LENGTH, PPTMB_ADDRESS, &PPTMB_SeqNum, PPT_Metrics_Payload); 
                break; 
            } 
            case 2: //Send LP Bus Metrics Data 
            { 
                LP_Bus_Metrics_Generator(LP_Bus_Metrics_Payload); 
                dummySend(LP_BUS_METRICS_LENGTH, PPTMB_ADDRESS, &PPTMB_SeqNum, LP_Bus_Metrics_Payload); 
                break; 
            } 
            case 3: //Send Core Temp Data 
            { 
                Core_Temp_Generator(Core_Temp_Payload); 
                dummySend(CORE_TEMP_LENGTH, PPTMB_ADDRESS, &PPTMB_SeqNum, Core_Temp_Payload); 
                break; 
            } 
            case 4: //Send Heartbeat Data 
            { 
                Heartbeat_Generator(Heartbeat_Payload); 
                dummySend(HEARTBEAT_LENGTH, PPTMB_ADDRESS, &PPTMB_SeqNum, Heartbeat_Payload); 
                break; 
            } 
        } 
    } 
} 
 
void MCMB(){ 
 
    //There are 6 data types in MCMB 
    for(int i=0; i<6; i++){
        switch(i){ 
            case 0: //Send Bus Metrics Data 
            { 
                Bus_Metrics_Generator(Bus_Metrics_Payload); 
                dummySend(BUS_METRICS_LENGTH, MCMB_ADDRESS, &MCMB_SeqNum, Bus_Metrics_Payload); 
                break; 
            } 
            case 1: //Send Speed Pulse Reading Data 
            { 
                Speed_Pulse_Reading_Generator(Speed_Pulse_Reading_Payload); 
                dummySend(SPEED_PULSE_READING_LENGTH, MCMB_ADDRESS, &MCMB_SeqNum, Speed_Pulse_Reading_Payload);
                break; 
            } 
            case 2: //Send Motor Temperature Data 
            { 
                Motor_Temperature_Generator(Motor_Temperature_Payload); 
                dummySend(MOTOR_TEMPERATURE_LENGTH, MCMB_ADDRESS, &MCMB_SeqNum, Motor_Temperature_Payload); 
                break; 
            } 
            case 3: //Send LP Bus Metrics Data 
            { 
                LP_Bus_Metrics_Generator(LP_Bus_Metrics_Payload); 
                dummySend(LP_BUS_METRICS_LENGTH, MCMB_ADDRESS, &MCMB_SeqNum, LP_Bus_Metrics_Payload); 
                break; 
            } 
            case 4: //Send Core Temp Data 
            { 
                Core_Temp_Generator(Core_Temp_Payload); 
                dummySend(CORE_TEMP_LENGTH, MCMB_ADDRESS, &MCMB_SeqNum, Core_Temp_Payload); 
                break; 
            } 
            case 5: //Send Heartbeat Data 
            { 
                Heartbeat_Generator(Heartbeat_Payload); 
                dummySend(HEARTBEAT_LENGTH, MCMB_ADDRESS, &MCMB_SeqNum, Heartbeat_Payload); 
                break; 
            } 
        } 
    } 
} 
 
void DCMB(){ 
 
    //There are 8 data types in DCMB 
    for(int i=0; i<8; i++){ 
 
        switch(i){ 
            case 0: //Send MC2 State Data 
            { 
                MC2_State_Generator(MC2_State_Payload); 
                dummySend(MC2_STATE_LENGTH, DCMB_ADDRESS, &DCMB_SeqNum, MC2_State_Payload); 
                break; 
            } 
            case 1: //Send BBox Startup Data 
            { 
                BBox_Startup_Generator(BBox_Startup_Payload); 
                dummySend(BBOX_STARTUP_LENGTH, DCMB_ADDRESS, &DCMB_SeqNum, BBox_Startup_Payload); 
                break; 
            } 
            case 2: //Send PPTBox Startup Data 
            { 
                PPTBox_Startup_Generator(PPTBox_Startup_Payload); 
                dummySend(PPTBOX_STARTUP_LENGTH, DCMB_ADDRESS, &DCMB_SeqNum, PPTBox_Startup_Payload);
                break; 
            } 
            case 3: //Send Light State Data 
            { 
                Light_State_Generator(Light_State_Payload); 
                dummySend(LIGHT_STATE_LENGTH, DCMB_ADDRESS, &DCMB_SeqNum, Light_State_Payload); 
                break; 
            } 
            case 4: //Send Horn State Data 
            { 
                Horn_State_Generator(Horn_State_Payload); 
                dummySend(HORN_STATE_LENGTH, DCMB_ADDRESS, &DCMB_SeqNum, Horn_State_Payload); 
                break; 
            } 
            case 5: //Send LP Bus Metrics Data 
            { 
                LP_Bus_Metrics_Generator(LP_Bus_Metrics_Payload); 
                dummySend(LP_BUS_METRICS_LENGTH, DCMB_ADDRESS, &DCMB_SeqNum, LP_Bus_Metrics_Payload); 
                break; 
            } 
            case 6: //Send Core Temp Data 
            { 
                Core_Temp_Generator(Core_Temp_Payload); 
                dummySend(CORE_TEMP_LENGTH, DCMB_ADDRESS, &DCMB_SeqNum, Core_Temp_Payload); 
                break; 
            } 
            case 7: //Send Heartbeat Data 
            { 
                Heartbeat_Generator(Heartbeat_Payload); 
                dummySend(HEARTBEAT_LENGTH, DCMB_ADDRESS, &DCMB_SeqNum, Heartbeat_Payload); 
                break; 
            } 
        } 
    } 
} 
 
int main() { 
    //call srand() once in the program 
    srand(time(NULL)); 
 
    for(int i=0; i<10; i++){ 
        BBMB(); 
        printf("\n"); 
        PPTMB(); 
        printf("\n"); 
        MCMB(); 
        printf("\n"); 
        DCMB(); 
        printf("\n"); 
    } 
 
    return 0; 
} 
 
void testPayloadGenerator(){ 
    //test Cell_Metrics_Generator() 
    printf("test Cell_Metrics_Generator()\n"); 
    for(int i=0; i<10; i++){ 
        Cell_Metrics_Generator(Cell_Metrics_Payload,i+1); 
        printPayload(Cell_Metrics_Payload, CELL_METRICS_LENGTH); 
    } 
 
    //test PPT_Metrics_Generator() 
    printf("test PPT_Metrics_Generator()\n");
    for(int i=0; i<10; i++){ 
        PPT_Metrics_Generator(PPT_Metrics_Payload); 
        printPayload(PPT_Metrics_Payload, PPT_METRICS_LENGTH); 
    } 
 
    //test MC2_State_Generator() 
    printf("test MC2_State_Generator()\n"); 
    for(int i=0; i<10; i++){ 
        MC2_State_Generator(MC2_State_Payload); 
        printPayload(MC2_State_Payload, MC2_STATE_LENGTH); 
    } 
 
    //test BBox_Startup_Generator() 
    printf("test BBox_Startup_Generator()\n"); 
    for(int i=0; i<10; i++){ 
        BBox_Startup_Generator(BBox_Startup_Payload); 
        printPayload(BBox_Startup_Payload, BBOX_STARTUP_LENGTH); 
    } 
 
    //test PPTBox_Startup_Generator() 
    printf("PPTBox_Startup_Generator()\n"); 
    for(int i=0; i<10; i++){ 
        PPTBox_Startup_Generator(PPTBox_Startup_Payload); 
        printPayload(PPTBox_Startup_Payload, PPTBOX_STARTUP_LENGTH); 
    } 
 
    //test Light_State_Generator() 
    printf("Light_State_Generator()\n"); 
    for(int i=0; i<10; i++){ 
        Light_State_Generator(Light_State_Payload); 
        printPayload(Light_State_Payload, LIGHT_STATE_LENGTH); 
    } 
 
    //test Horn_State_Generator 
    printf("Horn_State_Generator\n"); 
    for(int i=0; i<10; i++){ 
        Horn_State_Generator(Horn_State_Payload); 
        printPayload(Horn_State_Payload, HORN_STATE_LENGTH); 
    } 
 
    //test Core_Temp_Generator() 
    printf("test Core_Temp_Generator()\n"); 
    for(int i=0; i<10; i++){ 
        Core_Temp_Generator(Core_Temp_Payload); 
        printPayload(Core_Temp_Payload, CORE_TEMP_LENGTH); 
    } 
 
    //test Heartbeat_Generator() 
    printf("test Heartbeat_Generator()\n"); 
    Heartbeat_Generator(Heartbeat_Payload); 
    printPayload(Heartbeat_Payload, HEARTBEAT_LENGTH); 
}
