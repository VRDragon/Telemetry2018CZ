/** 
  ******************************************************************************
  * @file    CAN2018.h
  * @author  SGT Generated by tool
  * @version V0.1.0 (generator)
  * @date    30-July-2018
  * @brief   CAN protocol application/PROFILE layer for use in SGT formula student electric 2018
  ******************************************************************************
    Header contains all CAN bus message structures and defines of signals
    for STUBA Green Team formula 2018 created by Marek Laszlo and Marius Rak
  ******************************************************************************
  Do not change document and messages structure!

  HOW TO RECEIVE:
  -do your init of CAN device in main.c
  -define used structures by uncommenting in CAN2018.h
  -use extern for CAN2018.h data structures in your main.c file
  -send messages using declared functions. Received structures are saved automatically in receive structures
  -work with data from structure :)

  HOW TO TRANSMIT:
  -fill structure of message you want to send
  -call correspondent transmit function which will fill the TX  message structure and transmit

  ******************************************************************************/
#ifndef CAN2018_H_
#define CAN2018_H_

//#define Tx_BBOX_power 1
#define Rx_BBOX_power 1
#define ID_BBOX_power 0x10
#define DLC_BBOX_power 6
/* BBOX_power
 * 
 *
 * ID 0x10
 * Message size DLC: 6 bytes
 * Message is sent by node: 
 * Message contains:
 *                   rep start len type        comment
 * power            | A |  0 | 16 |  int16_t | vykon
 * current          | B | 16 | 16 |  int16_t | prud celkovy
 * voltage          | C | 32 | 16 | uint16_t | celkove napatie
 */

//             bit
//     |7|6|5|4|3|2|1|0|
//   |0|A|A|A|A|A|A|A|A|
//   |1|A|A|A|A|A|A|A|A|
// b |2|B|B|B|B|B|B|B|B|
// y |3|B|B|B|B|B|B|B|B|
// t |4|C|C|C|C|C|C|C|C|
// e |5|C|C|C|C|C|C|C|C|
//   |6| | | | | | | | |
//   |7| | | | | | | | |
typedef struct
{
    int16_t power;
    int16_t current;
    uint16_t voltage;
} BBOX_power_TypeDef;

#ifdef Tx_BBOX_power
void Tx_BBOX_power_Data(CAN_HandleTypeDef* hcan, BBOX_power_TypeDef* BBOX_power_Data);
#endif

//#define Tx_wheel_RPM 1
#define Rx_wheel_RPM 1
#define ID_wheel_RPM 0x15
#define DLC_wheel_RPM 8
/* wheel_RPM
 * 
 *
 * ID 0x15
 * Message size DLC: 8 bytes
 * Message is sent by node: 
 * Message contains:
 *                   rep start len type        comment
 * front_right      | A |  0 | 16 | uint16_t | prave predne 
 * front_left       | B | 16 | 16 | uint16_t | lave predne
 * rear_left        | C | 32 | 16 | uint16_t | lave zadne
 * rear_right       | D | 48 | 16 | uint16_t | prave zadne
 */

//             bit
//     |7|6|5|4|3|2|1|0|
//   |0|A|A|A|A|A|A|A|A|
//   |1|A|A|A|A|A|A|A|A|
// b |2|B|B|B|B|B|B|B|B|
// y |3|B|B|B|B|B|B|B|B|
// t |4|C|C|C|C|C|C|C|C|
// e |5|C|C|C|C|C|C|C|C|
//   |6|D|D|D|D|D|D|D|D|
//   |7|D|D|D|D|D|D|D|D|
typedef struct
{
    uint16_t front_right;
    uint16_t front_left;
    uint16_t rear_left;
    uint16_t rear_right;
} wheel_RPM_TypeDef;

#ifdef Tx_wheel_RPM
void Tx_wheel_RPM_Data(CAN_HandleTypeDef* hcan, wheel_RPM_TypeDef* wheel_RPM_Data);
#endif

//#define Tx_BBOX_status 1
#define Rx_BBOX_status 1
#define ID_BBOX_status 0x20
#define DLC_BBOX_status 3
/* BBOX_status
 * 
 *
 * ID 0x20
 * Message size DLC: 2 bytes
 * Message is sent by node: BMS_Control_Board
 * Message contains:
 *                   rep start len type        comment
 * TSMS             | A |  0 |  1 |  uint8_t | 0:Napatie nepritomne 1:Napatie pritomne
 * SHD_IN           | B |  1 |  1 |  uint8_t | 0:Napatie nepritomne 1:Napatie pritomne
 * SHD_OUT          | C |  2 |  1 |  uint8_t | 0:Napatie nepritomne 1:Napatie pritomne
 * AIR_N            | D |  3 |  1 |  uint8_t | 0:zaporny air vypnuty 1:zaporny air zopnuty
 * AIR_P            | E |  4 |  1 |  uint8_t | 0:kladny air vypnuty 1:kladny air zopnuty
 * PRECH_60V        | F |  5 |  1 |  uint8_t | 0:prech rele OFF, a napatie menej ako 60V 1:prech rele ON alebo napatie viac ako 60V
 * IMD_OK           | G |  6 |  1 |  uint8_t | 0:chybovy stav 1:IMD OK - signal pre mosfet ON
 * BMS_OK           | H |  7 |  1 |  uint8_t | 0:chybovy stav 1:IMD OK - signal pre mosfet ON
 * SIGNAL_ERROR     | I |  8 |  1 |  uint8_t | 0:chyba v prudovej sonde  1:prudova sonda ok
 * SHD_RESET        | J |  9 |  1 |  uint8_t | 0:resetuje sa SHD chyba 1:normal stav bez resetu
 * SHD_EN           | K | 10 |  1 |  uint8_t | 0:SHD_vypnuty 1:SHD_moze byt
 * POLARITY         | L | 11 |  1 |  uint8_t | 0:prud tecie do boxu  1:prud tecie z boxu
 * FANS             | M | 12 |  1 |  uint8_t | 0: ventilatory vypnute 1:ventilatory zapnute
 * STM_temp         | N | 13 |  8 |   int8_t | teplota procesoru
 */

//             bit
//     |7|6|5|4|3|2|1|0|
//   |0|A|B|C|D|E|F|G|H|
//   |1|I|J|K|L|M|N|N|N|
// b |2|N|N|N|N|N| | | |
// y |3| | | | | | | | |
// t |4| | | | | | | | |
// e |5| | | | | | | | |
//   |6| | | | | | | | |
//   |7| | | | | | | | |
typedef struct
{
    uint8_t TSMS;
    uint8_t SHD_IN;
    uint8_t SHD_OUT;
    uint8_t AIR_N;
    uint8_t AIR_P;
    uint8_t PRECH_60V;
    uint8_t IMD_OK;
    uint8_t BMS_OK;
    uint8_t SIGNAL_ERROR;
    uint8_t SHD_RESET;
    uint8_t SHD_EN;
    uint8_t POLARITY;
    uint8_t FANS;
    int8_t STM_temp;
} BBOX_status_TypeDef;

#ifdef Tx_BBOX_status
void Tx_BBOX_status_Data(CAN_HandleTypeDef* hcan, BBOX_status_TypeDef* BBOX_status_Data);
#endif

//#define Tx_FU_Values_1 1
#define Rx_FU_Values_1 1
#define ID_FU_Values_1 0x25
#define DLC_FU_Values_1 6
/* FU_Values_1
 * 
 *
 * ID 0x25
 * Message size DLC: 6 bytes
 * Message is sent by node: Front Unit
 * Message contains:
 *                   rep start len type        comment
 * apps1            | A |  0 |  8 |  uint8_t | 0-100%
 * apps2            | B |  8 |  8 |  uint8_t | 0-100%
 * brake1           | C | 16 |  8 |  uint8_t | psi
 * brake2           | D | 24 |  8 |  uint8_t | psi
 * error            | E | 32 | 16 | uint16_t | errors
 */

//             bit
//     |7|6|5|4|3|2|1|0|
//   |0|A|A|A|A|A|A|A|A|
//   |1|B|B|B|B|B|B|B|B|
// b |2|C|C|C|C|C|C|C|C|
// y |3|D|D|D|D|D|D|D|D|
// t |4|E|E|E|E|E|E|E|E|
// e |5|E|E|E|E|E|E|E|E|
//   |6| | | | | | | | |
//   |7| | | | | | | | |
typedef struct
{
    uint8_t apps1;
    uint8_t apps2;
    uint8_t brake1;
    uint8_t brake2;
    uint16_t error;
} FU_Values_1_TypeDef;

#ifdef Tx_FU_Values_1
void Tx_FU_Values_1_Data(CAN_HandleTypeDef* hcan, FU_Values_1_TypeDef* FU_Values_1_Data);
#endif

//#define Tx_BMS_State 1
#define Rx_BMS_State 1
#define ID_BMS_State 0x50
#define DLC_BMS_State 8
/* BMS_State
 * Basic message from BMS
 *
 * ID 0x50
 * Message size DLC: 8 bytes
 * Message is sent by node: BMS Master
 * Message contains:
 *                   rep start len type        comment
 * BMS_Mode         | A |  0 |  8 |  uint8_t | Mode of whole BMS
 * BMS_Faults       | B |  8 | 16 | uint16_t | Fault bits
 * CellVolt_L       | C | 24 |  8 |  uint8_t | Lowest cell voltage, (0-255x10+2000) mV
 * CellVolt_H       | D | 32 |  8 |  uint8_t | Highest cell voltage, (0-255x10+2000) mV	
 * CellTemp_L       | E | 40 |  8 |  uint8_t | Lowest cell temp, 0-255 degC
 * CellTemp_H       | F | 48 |  8 |  uint8_t | Highest cell temp, 0-255 degC	
 * BMS_Ident        | G | 56 |  8 |  uint8_t | Identifier of the battery box
 */

//             bit
//     |7|6|5|4|3|2|1|0|
//   |0|A|A|A|A|A|A|A|A|
//   |1|B|B|B|B|B|B|B|B|
// b |2|B|B|B|B|B|B|B|B|
// y |3|C|C|C|C|C|C|C|C|
// t |4|D|D|D|D|D|D|D|D|
// e |5|E|E|E|E|E|E|E|E|
//   |6|F|F|F|F|F|F|F|F|
//   |7|G|G|G|G|G|G|G|G|
typedef struct
{
    uint8_t BMS_Mode;
    uint16_t BMS_Faults;
    uint8_t CellVolt_L;
    uint8_t CellVolt_H;
    uint8_t CellTemp_L;
    uint8_t CellTemp_H;
    uint8_t BMS_Ident;
} BMS_State_TypeDef;

#ifdef Tx_BMS_State
void Tx_BMS_State_Data(CAN_HandleTypeDef* hcan, BMS_State_TypeDef* BMS_State_Data);
#endif

//#define Tx_ECU_State 1
#define Rx_ECU_State 1
#define ID_ECU_State 0x60
#define DLC_ECU_State 8
/* ECU_State
 * 
 *
 * ID 0x60
 * Message size DLC: 8 bytes
 * Message is sent by node: ECU
 * Message contains:
 *                   rep start len type        comment
 * ECU_Status       | A |  0 |  8 |  uint8_t | ECU_Status word
 * FL_AMK_Status    | B |  8 |  8 |  uint8_t | FL_Status word
 * FR_AMK_Status    | C | 16 |  8 |  uint8_t | FR_Status word
 * RL_AMK_Status    | D | 24 |  8 |  uint8_t | RL_Status word
 * RR_AMK_Status    | E | 32 |  8 |  uint8_t | RR_Status word
 * TempMotor_H      | F | 40 |  8 |  uint8_t | Highest temp. of motors
 * TempInverter_H   | G | 48 |  8 |  uint8_t | Highest temp. of inverters
 * TempIGBT_H       | H | 56 |  8 |  uint8_t | Highest temp. of IGBT
 */

//             bit
//     |7|6|5|4|3|2|1|0|
//   |0|A|A|A|A|A|A|A|A|
//   |1|B|B|B|B|B|B|B|B|
// b |2|C|C|C|C|C|C|C|C|
// y |3|D|D|D|D|D|D|D|D|
// t |4|E|E|E|E|E|E|E|E|
// e |5|F|F|F|F|F|F|F|F|
//   |6|G|G|G|G|G|G|G|G|
//   |7|H|H|H|H|H|H|H|H|
typedef struct
{
    uint8_t ECU_Status;
    uint8_t FL_AMK_Status;
    uint8_t FR_AMK_Status;
    uint8_t RL_AMK_Status;
    uint8_t RR_AMK_Status;
    uint8_t TempMotor_H;
    uint8_t TempInverter_H;
    uint8_t TempIGBT_H;
} ECU_State_TypeDef;

#ifdef Tx_ECU_State
void Tx_ECU_State_Data(CAN_HandleTypeDef* hcan, ECU_State_TypeDef* ECU_State_Data);
#endif

//#define Tx_FU_Values_2 1
#define Rx_FU_Values_2 1
#define ID_FU_Values_2 0x70
#define DLC_FU_Values_2 7
/* FU_Values_2
 * 
 *
 * ID 0x70
 * Message size DLC: 6 bytes
 * Message is sent by node: Front Unit
 * Message contains:
 *                   rep start len type        comment
 * steer            | A |  0 |  8 |   int8_t | -127 = L, 127 = R
 * susp_FL          | B |  8 | 12 | uint16_t | 0-4095 RAW data
 * susp_FR          | C | 20 | 12 | uint16_t | 0-4095 RAW data
 * brake_pos        | D | 32 |  8 |  uint8_t | 0-100%
 * RTD              | E | 40 |  1 |  uint8_t | bool
 * BOTS             | F | 41 |  1 |  uint8_t | bool
 * SHDB             | G | 42 |  1 |  uint8_t | bool
 * INERTIA_SW       | H | 43 |  1 |  uint8_t | bool
 * reserve          | I | 44 |  8 |  uint8_t | reserve
 */

//             bit
//     |7|6|5|4|3|2|1|0|
//   |0|A|A|A|A|A|A|A|A|
//   |1|B|B|B|B|B|B|B|B|
// b |2|B|B|B|B|C|C|C|C|
// y |3|C|C|C|C|C|C|C|C|
// t |4|D|D|D|D|D|D|D|D|
// e |5|E|F|G|H|I|I|I|I|
//   |6|I|I|I|I| | | | |
//   |7| | | | | | | | |
typedef struct
{
    int8_t steer;
    uint16_t susp_FL;
    uint16_t susp_FR;
    uint8_t brake_pos;
    uint8_t RTD;
    uint8_t BOTS;
    uint8_t SHDB;
    uint8_t INERTIA_SW;
    uint8_t reserve;
} FU_Values_2_TypeDef;

#ifdef Tx_FU_Values_2
void Tx_FU_Values_2_Data(CAN_HandleTypeDef* hcan, FU_Values_2_TypeDef* FU_Values_2_Data);
#endif

//#define Tx_Interconnect 1
#define Rx_Interconnect 1
#define ID_Interconnect 0x80
#define DLC_Interconnect 6
/* Interconnect
 * 
 *
 * ID 0x80
 * Message size DLC: 5 bytes
 * Message is sent by node: 
 * Message contains:
 *                   rep start len type        comment
 * car_state        | A |  0 |  8 |  uint8_t | Formula state
 * left_w_pump      | B |  8 |  1 |  uint8_t | lava pumpa na vodu
 * right_w_pump     | C |  9 |  1 |  uint8_t | prava pumpa na vodu
 * brake_red        | D | 10 |  1 |  uint8_t | brzdove svetlo cervene
 * brake_white      | E | 11 |  1 |  uint8_t | brzdove svetlo biele
 * tsas             | F | 12 |  1 |  uint8_t | TSAS
 * killswitch_R     | G | 13 |  1 |  uint8_t | 0: killswitch OFF 1:killswitch ON
 * killswitch_L     | H | 14 |  1 |  uint8_t | 0: killswitch OFF 1:killswitch ON
 * reserve          | I | 15 |  8 |  uint8_t | pre dalsie pouzitie
 * susp_RR          | J | 23 | 12 | uint16_t | 0 -4095 raw 
 * susp_RL          | K | 35 | 12 | uint16_t | 0 - 4095 raw
 */

//             bit
//     |7|6|5|4|3|2|1|0|
//   |0|A|A|A|A|A|A|A|A|
//   |1|B|C|D|E|F|G|H|I|
// b |2|I|I|I|I|I|I|I|J|
// y |3|J|J|J|J|J|J|J|J|
// t |4|J|J|J|K|K|K|K|K|
// e |5|K|K|K|K|K|K|K| |
//   |6| | | | | | | | |
//   |7| | | | | | | | |
typedef struct
{
    uint8_t car_state;
    uint8_t left_w_pump;
    uint8_t right_w_pump;
    uint8_t brake_red;
    uint8_t brake_white;
    uint8_t tsas;
    uint8_t killswitch_R;
    uint8_t killswitch_L;
    uint8_t reserve;
    uint16_t susp_RR;
    uint16_t susp_RL;
} Interconnect_TypeDef;

#ifdef Tx_Interconnect
void Tx_Interconnect_Data(CAN_HandleTypeDef* hcan, Interconnect_TypeDef* Interconnect_Data);
#endif

//#define Tx_BMS_Voltages 1
#define Rx_BMS_Voltages 1
#define ID_BMS_Voltages 0x90
#define DLC_BMS_Voltages 8
/* BMS_Voltages
 * Cell voltages sent in FullMode
 *
 * ID 0x90
 * Message size DLC: 8 bytes
 * Message is sent by node: BMS Master
 * Message contains:
 *                   rep start len type        comment
 * BMS_VoltIdent    | A |  0 |  8 |  uint8_t | Identifier of sent voltages
 * BMS_Volt1        | B |  8 |  8 |  uint8_t | Cell voltage, (0-255x10+2000) mV
 * BMS_Volt2        | C | 16 |  8 |  uint8_t | Cell voltage, (0-255x10+2000) mV
 * BMS_Volt3        | D | 24 |  8 |  uint8_t | Cell voltage, (0-255x10+2000) mV
 * BMS_Volt4        | E | 32 |  8 |  uint8_t | Cell voltage, (0-255x10+2000) mV
 * BMS_Volt5        | F | 40 |  8 |  uint8_t | Cell voltage, (0-255x10+2000) mV
 * BMS_Volt6        | G | 48 |  8 |  uint8_t | Cell voltage, (0-255x10+2000) mV
 * BMS_Volt7        | H | 56 |  8 |  uint8_t | Cell voltage, (0-255x10+2000) mV
 */

//             bit
//     |7|6|5|4|3|2|1|0|
//   |0|A|A|A|A|A|A|A|A|
//   |1|B|B|B|B|B|B|B|B|
// b |2|C|C|C|C|C|C|C|C|
// y |3|D|D|D|D|D|D|D|D|
// t |4|E|E|E|E|E|E|E|E|
// e |5|F|F|F|F|F|F|F|F|
//   |6|G|G|G|G|G|G|G|G|
//   |7|H|H|H|H|H|H|H|H|
typedef struct
{
    uint8_t BMS_VoltIdent;
    uint8_t BMS_Volt1;
    uint8_t BMS_Volt2;
    uint8_t BMS_Volt3;
    uint8_t BMS_Volt4;
    uint8_t BMS_Volt5;
    uint8_t BMS_Volt6;
    uint8_t BMS_Volt7;
} BMS_Voltages_TypeDef;

#ifdef Tx_BMS_Voltages
void Tx_BMS_Voltages_Data(CAN_HandleTypeDef* hcan, BMS_Voltages_TypeDef* BMS_Voltages_Data);
#endif

//#define Tx_BMS_Temps 1
#define Rx_BMS_Temps 1
#define ID_BMS_Temps 0x95
#define DLC_BMS_Temps 8
/* BMS_Temps
 * Cell temperatures sent in FullMode
 *
 * ID 0x95
 * Message size DLC: 8 bytes
 * Message is sent by node: BMS Master
 * Message contains:
 *                   rep start len type        comment
 * BMS_TempIdent    | A |  0 |  8 |  uint8_t | Identifier of sent temperatures
 * BMS_Temp1        | B |  8 |  8 |  uint8_t | Cell temperature, 0-255 degC
 * BMS_Temp2        | C | 16 |  8 |  uint8_t | Cell temperature, 0-255 degC
 * BMS_Temp3        | D | 24 |  8 |  uint8_t | Cell temperature, 0-255 degC
 * BMS_Temp4        | E | 32 |  8 |  uint8_t | Cell temperature, 0-255 degC
 * BMS_Temp5        | F | 40 |  8 |  uint8_t | Cell temperature, 0-255 degC
 * BMS_Temp6        | G | 48 |  8 |  uint8_t | Cell temperature, 0-255 degC
 * BMS_Temp7        | H | 56 |  8 |  uint8_t | Cell temperature, 0-255 degC
 */

//             bit
//     |7|6|5|4|3|2|1|0|
//   |0|A|A|A|A|A|A|A|A|
//   |1|B|B|B|B|B|B|B|B|
// b |2|C|C|C|C|C|C|C|C|
// y |3|D|D|D|D|D|D|D|D|
// t |4|E|E|E|E|E|E|E|E|
// e |5|F|F|F|F|F|F|F|F|
//   |6|G|G|G|G|G|G|G|G|
//   |7|H|H|H|H|H|H|H|H|
typedef struct
{
    uint8_t BMS_TempIdent;
    uint8_t BMS_Temp1;
    uint8_t BMS_Temp2;
    uint8_t BMS_Temp3;
    uint8_t BMS_Temp4;
    uint8_t BMS_Temp5;
    uint8_t BMS_Temp6;
    uint8_t BMS_Temp7;
} BMS_Temps_TypeDef;

#ifdef Tx_BMS_Temps
void Tx_BMS_Temps_Data(CAN_HandleTypeDef* hcan, BMS_Temps_TypeDef* BMS_Temps_Data);
#endif

#endif /* CAN2018_H_ */
