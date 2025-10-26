/* USER CODE BEGIN Header */
/*
  TODO:
  UART Control (low)
  Get Mot temp conversion ratio
  SDMMC Log  
  Encoder Error
  RTC
  DONE:
  SDMMC Test with FATFS without MDMA
  ADC DMA
  Bypass SD Card detetion in BSP_PlatformIsDetected
  Invalidate DCache after ACD finish conversion
  OCP OTP Error
  SDMMC with FATFS and IDMA
  Change CAN send data
*/

// #pragma GCC optimize("O2")
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "fatfs.h"
#include "fdcan.h"
#include "i2c.h"
#include "mdma.h"
#include "rtc.h"
#include "sdmmc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nturt_inv/nturt_inv.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

NTURT_INV_TypeDef nih = { // nturt_inv_handle
  .nturt_main_run = true,
  .angle_encoder = {
    .enc_index = 0,
    .enc_sum = 0,
    .enc_buf = {0},
    .pole_pairs = 1,
    .enc_dir = 1,
    .zero_electric_angle = ZERO_ELECTRIC_ANGLE,
    .shaft_angle = 0.0f,
    .period = CCR,
  },
  .SIC_T = {
    .adc_to_SIC_T_lut = {-750,-696,-608,-553,-512,-479,-452,-428,-407,-388,-370,-355,-340,-326,-314,-301,-290,-279,-269,-259,-250,-241,-232,-224,-216,-208,-200,-193,-186,-179,-172,-166,-160,-153,-147,-142,-136,-130,-125,-119,-114,-109,-104,-99,-94,-89,-84,-80,-75,-71,-66,-62,-58,-53,-49,-45,-41,-37,-33,-29,-26,-22,-18,-14,-11,-7,-4,0,2,6,9,13,16,19,22,26,29,32,35,38,41,44,47,50,53,56,59,62,65,68,71,73,76,79,82,84,87,90,92,95,98,100,103,105,108,110,113,115,118,120,123,125,128,130,132,135,137,139,142,144,146,149,151,153,155,158,160,162,164,167,169,171,173,175,177,179,182,184,186,188,190,192,194,196,198,200,202,204,206,208,210,212,214,216,218,220,222,224,226,228,230,231,233,235,237,239,241,243,245,246,248,250,252,254,256,257,259,261,263,265,266,268,270,272,273,275,277,279,280,282,284,286,287,289,291,292,294,296,297,299,301,302,304,306,307,309,311,312,314,316,317,319,321,322,324,325,327,329,330,332,333,335,337,338,340,341,343,345,346,348,349,351,352,354,355,357,359,360,362,363,365,366,368,369,371,372,374,375,377,378,380,381,383,384,386,387,389,390,392,393,395,396,398,399,401,402,404,405,407,408,409,411,412,414,415,417,418,420,421,422,424,425,427,428,430,431,432,434,435,437,438,440,441,442,444,445,447,448,449,451,452,454,455,456,458,459,461,462,463,465,466,468,469,470,472,473,474,476,477,479,480,481,483,484,485,487,488,490,491,492,494,495,496,498,499,501,502,503,505,506,507,509,510,511,513,514,515,517,518,519,521,522,523,525,526,528,529,530,532,533,534,536,537,538,540,541,542,544,545,546,548,549,550,552,553,554,556,557,558,560,561,562,564,565,566,568,569,570,572,573,574,576,577,578,580,581,582,583,585,586,587,589,590,591,593,594,595,597,598,599,601,602,603,605,606,607,609,610,611,613,614,615,617,618,619,621,622,623,625,626,627,629,630,631,633,634,635,636,638,639,640,642,643,644,646,647,648,650,651,652,654,655,656,658,659,660,662,663,664,666,667,668,670,671,672,674,675,677,678,679,681,682,683,685,686,687,689,690,691,693,694,695,697,698,699,701,702,703,705,706,708,709,710,712,713,714,716,717,718,720,721,723,724,725,727,728,729,731,732,733,735,736,738,739,740,742,743,745,746,747,749,750,751,753,754,756,757,758,760,761,763,764,765,767,768,770,771,772,774,775,777,778,780,781,782,784,785,787,788,789,791,792,794,795,797,798,800,801,802,804,805,807,808,810,811,813,814,815,817,818,820,821,823,824,826,827,829,830,832,833,835,836,838,839,840,842,843,845,846,848,849,851,852,854,856,857,859,860,862,863,865,866,868,869,871,872,874,875,877,878,880,882,883,885,886,888,889,891,893,894,896,897,899,900,902,904,905,907,908,910,912,913,915,916,918,920,921,923,925,926,928,929,931,933,934,936,938,939,941,943,944,946,948,949,951,953,954,956,958,959,961,963,965,966,968,970,971,973,975,977,978,980,982,984,985,987,989,991,992,994,996,998,1000,1001,1003,1005,1007,1009,1010,1012,1014,1016,1018,1020,1021,1023,1025,1027,1029,1031,1033,1034,1036,1038,1040,1042,1044,1046,1048,1050,1052,1054,1055,1057,1059,1061,1063,1065,1067,1069,1071,1073,1075,1077,1079,1081,1083,1085,1087,1089,1091,1093,1095,1097,1100,1102,1104,1106,1108,1110,1112,1114,1116,1118,1121,1123,1125,1127,1129,1131,1134,1136,1138,1140,1142,1145,1147,1149,1151,1154,1156,1158,1160,1163,1165,1167,1169,1172,1174,1176,1179,1181,1184,1186,1188,1191,1193,1195,1198,1200,1203,1205,1208,1210,1213,1215,1218,1220,1223,1225,1228,1230,1233,1235,1238,1241,1243,1246,1248,1251,1254,1256,1259,1262,1264,1267,1270,1273,1275,1278,1281,1284,1287,1289,1292,1295,1298,1301,1304,1307,1310,1312,1315,1318,1321,1324,1327,1330,1333,1337,1340,1343,1346,1349,1352,1355,1358,1362,1365,1368,1371,1375,1378,1381,1385,1388,1391,1395,1398,1402,1405,1409,1412,1416,1419,1423,1426,1430,1434,1437,1441,1445,1448,1452,1456,1460,1464,1468,1471,1475,1479,1483,1487,1491,1496,1500,1504,1508,1512,1516,1521,1525,1529,1534,1538,1542,1547,1551,1556,1561,1565,1570,1575,1579,1584,1589,1594,1599,1604,1609,1614,1619,1624,1629,1635,1640,1645,1651,1656,1662,1667,1673,1678,1684,1690,1696,1702,1708,1714,1720,1726,1733,1739,1745,1752,1758,1765,1772,1779,1785,1792,1800,1807,1814,1821,1829,1836,1844,1852,1860,1868,1876,1884,1892,1901,1909,1918,1927,1936,1945,1954,1964,1973,1983,1993,2003,2014,2024,2035,2046,2057,2068,2080,2091,2103,2116,2128,2141,2154,2168,2181,2195,2210,2224,2239,2255,2271,2287,2304,2321,2338,2356,2375,2394,2414,2435,2456,2478,2501,2524,2549,2574,2600,2628,2657,2686,2718,2750,2785,2821,2859,2899,2942,2987,3035,3087,3142,3202,3266,3336,3412,3497,3590,3695,3814,3951,4111,4301,4534,4830,5227,5804,6778,9092},
    .U = 0,
    .V = 0,
    .W = 0,
    .max = 0
  },
  .Mot_T = {
    .adc_to_Mot_T_lut = {0},
    .T = 0,
    .cal_par_I = MOT_CURR
  },
  .MCU_T = {
    .cal_par_1 = 0.0f,
    .cal_par_2 = 0.0f,
    .T = 0,
    .adc_to_MCU_T_lut = {0}
  },
  .inv_state = STATE_INIT,
  .err_state = ERROR_NONE,
  .DCbus = {
    .report_DCV = 0,
    .report_DCA = 0
  }
};

// for uart debug, not used currently
__attribute__((section("._RAM_Area"))) static char tdata[500]; 

// for sd card logging
__attribute__((section("._RAM_Area"))) FATFS SD_FatFS_RAM;  /* File system object for SD card logical drive */
__attribute__((section("._RAM_Area"))) FIL SD_File;     /* File object */
__attribute__((section("._RAM_Area"))) logger_t SD_log_buf[2][3600];//Logging Buffers

uint8_t SD_wr_log_buf_num = 0;
uint16_t SD_wr_log_index = 0;
RTC_DateTypeDef SD_log_date;
RTC_TimeTypeDef SD_log_time;
uint8_t SD_log_prev_sec = 0;
uint16_t SD_log_subsec = 0;

int max_sd_buf = 0;
const char TestFPath[] = {"Test.bin"};
char TextFPath[80];

// for execution timing analysis
#ifdef TIMING
int max_time = 0;
int min_time = INT32_MAX;
int prev_time = 0;
int max_btw = 0;
int max_sprint = 0;
int max_sdwrite = 0;
int sd_td;
uint32_t loop_time;
#endif

/// hardware 
// for hard ware oc
uint8_t oc_hw_buf[HW_OC_TIME] = {0};
uint16_t oc_hw_index = 0;
uint16_t oc_hw_sum = 0;

// for soft oc
uint8_t oc_sw_buf[SOFT_OC_TIME] = {0};
uint16_t oc_sw_index = 0;
uint16_t oc_sw_sum = 0;

// DC voltage reading
float voltage_power_supply = 440;

/// FOC and motor control variables
float open_loop_timestamp = 0;
const float phase_amp_per_LSB = -0.1031436f;   // phase_amp_per_LSB = 3.3/15.626e-3/adc1_range
float filt_RPM;
float filt_Iq;
float filt_Id;
float Ia;
float Iq_controller_output;
float Id_controller_output;
int enable_hw_oc = 0; // hardware overcurrent protection enable flag
uint8_t enable_dc_control = 0;
const float derate_a = 1/(DERATE_START-DERATE_END);
const float derate_b = DERATE_END/(DERATE_START-DERATE_END); 

/// LEDs and status indicators
int indexLED = 0;
int indexHeartbeat = 0;
int indexStatus = 0;
int indexTimer = 0;
uint32_t prev_new_file_time = 0;
int prev_sd = 0;
int prevWhileTest = 0;
uint8_t got_date = 0;
uint8_t prev_got_date = 0;

int freq = FREQ;
float Ts = (float)1/FREQ;
/// float angle_prev=-1.0f;
const float torque_constant = 0.291f; //Nm/A
const float electrical_constant = 0.031f; //rad/s/V
const float max_torque = 25;
const float max_current = 70;
float percent_trq_request = 0.0f;
float prev_percent_trq_request = 0.0f;
float abs_prev_percent = 0.0f;
uint16_t current_offset[4];
float current_phase[3];
const float DCVPLSB = 0.00897;     // DCVPLSB = 451*3.3/adc3_range 
const float DCAPLSB = 0.0402930f;   // DCAPLSB = 3.3/20e-3/adc1_range 
float max_ramp = 1/FREQ/RAMP_TIME_DERATE;
uint8_t fast_stop_enable = 1;

// inverter state and error state


/// CAN
// CAN Headers
FDCAN_TxHeaderTypeDef HeartBeatHeader = { .Identifier = CAN_ID_HEARTBEAT+MOT_ID,.IdType = FDCAN_STANDARD_ID,.TxFrameType = FDCAN_DATA_FRAME,
                                          .DataLength = FDCAN_DLC_BYTES_1,.ErrorStateIndicator = FDCAN_ESI_ACTIVE,.BitRateSwitch = FDCAN_BRS_OFF,
                                          .FDFormat = FDCAN_CLASSIC_CAN,.TxEventFifoControl = FDCAN_STORE_TX_EVENTS,.MessageMarker = 0x01};
FDCAN_TxHeaderTypeDef TempHeader      = { .Identifier = CAN_ID_TEMPERATURE+MOT_ID,.IdType = FDCAN_STANDARD_ID,.TxFrameType = FDCAN_DATA_FRAME,
                                          .DataLength = FDCAN_DLC_BYTES_6,.ErrorStateIndicator = FDCAN_ESI_ACTIVE,.BitRateSwitch = FDCAN_BRS_OFF,
                                          .FDFormat = FDCAN_CLASSIC_CAN,.TxEventFifoControl = FDCAN_STORE_TX_EVENTS,.MessageMarker = 0x02};
FDCAN_TxHeaderTypeDef StateHeader     = { .Identifier = CAN_ID_STATE+MOT_ID,.IdType = FDCAN_STANDARD_ID,.TxFrameType = FDCAN_DATA_FRAME,
                                          .DataLength = FDCAN_DLC_BYTES_4,.ErrorStateIndicator = FDCAN_ESI_ACTIVE,.BitRateSwitch = FDCAN_BRS_OFF,
                                          .FDFormat = FDCAN_CLASSIC_CAN,.TxEventFifoControl = FDCAN_STORE_TX_EVENTS,.MessageMarker = 0x03};
FDCAN_TxHeaderTypeDef StatusHeader    = { .Identifier = CAN_ID_STATUS+MOT_ID,.IdType = FDCAN_STANDARD_ID,.TxFrameType = FDCAN_DATA_FRAME,
                                          .DataLength = FDCAN_DLC_BYTES_6,.ErrorStateIndicator = FDCAN_ESI_ACTIVE,.BitRateSwitch = FDCAN_BRS_OFF,
                                          .FDFormat = FDCAN_CLASSIC_CAN,.TxEventFifoControl = FDCAN_STORE_TX_EVENTS,.MessageMarker = 0x04};
FDCAN_TxHeaderTypeDef PerameterHeader = { .Identifier = CAN_ID_PERAM+MOT_ID,.IdType = FDCAN_STANDARD_ID,.TxFrameType = FDCAN_DATA_FRAME,
                                          .DataLength = FDCAN_DLC_BYTES_5,.ErrorStateIndicator = FDCAN_ESI_ACTIVE,.BitRateSwitch = FDCAN_BRS_OFF,
                                          .FDFormat = FDCAN_CLASSIC_CAN,.TxEventFifoControl = FDCAN_STORE_TX_EVENTS,.MessageMarker = 0x05};
FDCAN_RxHeaderTypeDef RxHeader1;
int isSent = 1;
uint16_t control;
int CAN_Timer = 0;




// DMA ADC buffers
__attribute__((section("._ADC1_Area"))) uint16_t DMA_ADC1_arr[4] = {0};
__attribute__((section("._ADC2_Area"))) uint16_t DMA_ADC2_arr[4] = {0};
__attribute__((section("._ADC3_Area"))) uint16_t DMA_ADC3_arr[6] = {0};

lpf_t filter_current_Iq= {.Tf=QTF,.y_prev=0.0f}; //Tf=20ms
lpf_t filter_current_Id= {.Tf=DTF,.y_prev=0.0f}; //Tf=20ms
lpf_t filter_current_Iabc[3] = {{.Tf = ABCTF,.y_prev=0.0f},{.Tf = ABCTF,.y_prev=0.0f},{.Tf = ABCTF,.y_prev=0.0f}}; //Tf=2ms
lpf_t filter_crrent_DC_Iabc[3] = {{.Tf = DCTF,.y_prev=0.0f},{.Tf = DCTF,.y_prev=0.0f},{.Tf = DCTF,.y_prev=0.0f}}; //Tf=2ms
lpf_t filter_RPM = {.Tf=RPMTF,.y_prev=0.0f};
lpf_t filter_report_torque = {.Tf=0.008f,.y_prev=0.0f}; //Tf=100ms
pidc_t pid_controller_current_Iq = {.P=QKP,.I=QKI,.D=QKD,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.error_prev=0,.output_prev=0,.integral_prev=0};
pidc_t pid_controller_current_Id = {.P=DKP,.I=DKI,.D=DKD,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.error_prev=0,.output_prev=0,.integral_prev=0};
pidc_t pid_controller_current_OCP = {.P=1.0f,.I=1.0f,.D=PID_D,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.error_prev=0,.output_prev=0,.integral_prev=0};
pidc_t pid_controller_current_Ia = {.P=1.0f,.I=1.0f,.D=PID_D,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.error_prev=0,.output_prev=0,.integral_prev=0};
pidc_t pid_controller_current_Iabc[3] = {
    {.P=DCKP,.I=DCKI,.D=PID_D,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.error_prev=0,.output_prev=0,.integral_prev=0},
    {.P=DCKP,.I=DCKI,.D=PID_D,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.error_prev=0,.output_prev=0,.integral_prev=0},
    {.P=DCKP,.I=DCKI,.D=PID_D,.output_ramp=PID_RAMP,.limit=PID_LIMIT,.error_prev=0,.output_prev=0,.integral_prev=0}
};

void UART_TX_Send(UART_HandleTypeDef *huart,const char *format,...)
{
  va_list list;
  va_start(list,format);  
  vsnprintf(tdata,sizeof(tdata),format,list);
  if(isSent)
  {
    HAL_UART_Transmit_DMA(huart,(const uint8_t*)tdata,sizeof(tdata));
    isSent = 0;
  }
  va_end(list);
}

void CAN1_SetMsg(FDCAN_TxHeaderTypeDef *pTxHeader, const uint8_t *pTxData)
{
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, pTxHeader, pTxData);
    /* 启动FDCAN模块 */
    HAL_FDCAN_Start(&hfdcan1);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  nturt_inv_init_1(&nih);
  volatile FRESULT res;                                 /* FatFs function common result code */
	uint32_t byteswritten;                     /* File write/read counts */
	// uint8_t wtext[] = "This is STM32 working with FatFs\n"; /* File write buffer */
  pid_controller_current_Ia.limit = nih.PID.V_max;
  pid_controller_current_Id.limit = nih.PID.V_max;
  pid_controller_current_Iq.limit = nih.PID.V_max;
  pid_controller_current_OCP.limit = nih.PID.V_max;
  // uint8_t wlooptext[] = "This is STM32 working with FatFs in main loop\n"; /* File write buffer */
  // SD_log_buf[0][0].LGSTATE = 0;
  for (size_t i = 0; i < 1024; i++) nih.Mot_T.adc_to_Mot_T_lut[i] = (int16_t)10*((float)(1650-(3300*i/1024))/nih.Mot_T.cal_par_I/3.795-1000/3.795);
  
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_MDMA_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  MX_USART1_UART_Init();
  MX_ADC3_Init();
  MX_TIM1_Init();
  MX_ADC2_Init();
  MX_I2C2_Init();
  MX_TIM5_Init();
  MX_RTC_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_TIM3_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */


  nturt_inv_init_2(&nih);
  // Calibrate ADC
  HAL_Delay(100);
  HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET_LINEARITY,ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2,ADC_CALIB_OFFSET_LINEARITY,ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc3,ADC_CALIB_OFFSET_LINEARITY,ADC_SINGLE_ENDED);
  HAL_Delay(100);

  // Start Timers
  HAL_TIM_Base_Start(&htim5);
  HAL_TIM_Base_Start(&htim2);

  //Init SD files
  BSP_SD_Init();
  HAL_SD_InitCard(&hsd1);

  volatile int sdcard_status = HAL_SD_GetCardState(&hsd1);  
  if(sdcard_status == HAL_SD_CARD_TRANSFER)
  {
    res = f_mount(&SD_FatFS_RAM,(TCHAR const*)SDPath,1);
    // res = f_mount(&SDFatFS,"0",0);
    if (res != FR_OK)
    {
      Error_Handler();
    }
    else
    {
      #ifdef SDDEBUG
      res = f_open(&SD_File,TestFPath,FA_CREATE_ALWAYS | FA_WRITE);
      if (res != FR_OK)
      {
        Error_Handler();
      }
      else
      {
        set_one(log_test);
        f_write(&SD_File,&log_test,sizeof(log_test),(void *)&byteswritten);
        res = f_close(&SD_File);
        if(res!=FR_OK)
        {
          Error_Handler();
        }
      }
      #endif
    }
  }
  

  // Init ADC DMA
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)DMA_ADC1_arr,4);
  HAL_ADC_Start_DMA(&hadc2,(uint32_t*)DMA_ADC2_arr,4);
  HAL_ADC_Start_DMA(&hadc3,(uint32_t*)DMA_ADC3_arr,6);
  // HAL_MDMA_Start_IT(&hmdma_mdma_channel0_dma1_stream2_tc_0,(uint32_t)tmp_DMA_ADC1_arr,(uint32_t)DMA_ADC1_arr,8,1);
  // HAL_MDMA_Start_IT(&hmdma_mdma_channel1_dma1_stream1_tc_0,(uint32_t)tmp_DMA_ADC2_arr,(uint32_t)DMA_ADC2_arr,8,1);
  // HAL_MDMA_Start_IT(&hmdma_mdma_channel2_dma1_stream4_tc_0,(uint32_t)tmp_DMA_ADC3_arr,(uint32_t)DMA_ADC3_arr,12,1);
  // HAL_ADC_Start_IT(&hadc3);

  // HAL_GPIO_WritePin(Motor_Enable_GPIO_Port, Motor_Enable_Pin, GPIO_PIN_SET);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_Delay(1000);
  calibrateOffsets(current_offset,DMA_ADC1_arr);
  // HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);

  //Wait for GATE READY Signal
  #ifdef WAIT_GATE_READY
  while (HAL_GPIO_ReadPin(GATE_Ready_GPIO_Port,GATE_Ready_Pin) != GPIO_PIN_SET)
  {
    HAL_Delay(1);
  }
  #endif
   
  #ifdef CAL_ZERO_ANGLE
  float angle_integrate = 0.0f;
  HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_SET);
  setPhaseVoltage(25,0,_electricalAngle(M_PI*1.5f,pole_pairs),TIM1,0,0,0);
  for (size_t i = 0; i < 2000; i++)
  {
    nturt_inv_update_encoder_angle_now(DMA_ADC2_arr,&angle_now);
    HAL_Delay(10);
    SCB_InvalidateDCache_by_Addr(DMA_ADC2_arr,sizeof(DMA_ADC2_arr));
    if (i >= 1500)
    {
      angle_integrate += angle_now;
    }
  }
  
  // uint16_t read_raw=read(&hspi1, SPI1_CSn_GPIO_Port,SPI1_CSn_Pin,AS5048A_ANGLE);
  float raw_angle;
  SCB_InvalidateDCache_by_Addr(DMA_ADC2_arr,sizeof(DMA_ADC2_arr));
  nturt_inv_update_encoder_angle_now(DMA_ADC2_arr,&raw_angle);
  raw_angle = angle_integrate/500.0f;
  zero_electric_angle=_electricalAngle(raw_angle,pole_pairs);
  setPhaseVoltage(0,0,_electricalAngle(M_PI*1.5f,pole_pairs),TIM1,0,0,0);
  HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
  #endif

  UART_TX_Send(&huart1,"zero_electric_angle: %i \n",(int) floor(nih.angle_encoder.zero_electric_angle/M_PI*180));
  
  Config_Fdcan1();

  // while (!got_date)
  // {
  //   HAL_Delay(10);
  // }  

  //Get DateTime
  HAL_RTC_GetDate(&hrtc,&SD_log_date,RTC_FORMAT_BIN);
  HAL_RTC_GetTime(&hrtc,&SD_log_time,RTC_FORMAT_BIN);  

  snprintf(TextFPath, sizeof(TextFPath),FILENAME,(int)SD_log_date.Year+2000,(int)SD_log_date.Month,(int)SD_log_date.Date,(int)SD_log_time.Hours,(int)SD_log_time.Minutes,(int)SD_log_time.Seconds);
  // snprintf(TextFPath, sizeof(TextFPath),"text.bin");
  res = f_open(&SD_File,TextFPath,FA_CREATE_ALWAYS|FA_WRITE);
  if(res == FR_OK)
  {
    f_close(&SD_File);
  }
  f_open(&SD_File,TextFPath,FA_OPEN_APPEND|FA_WRITE);

  HAL_GPIO_WritePin(LED_ERR_GPIO_Port,LED_ERR_Pin,GPIO_PIN_RESET);
  nih.inv_state = STATE_READY;
  nih.err_state = ERROR_NONE;
  // INV_Statustypedef last_state = inv_state;

  #ifdef TIMING
  prev_time = __HAL_TIM_GET_COUNTER(&htim5);
  #endif

  HAL_GPIO_WritePin(LED_SD_GPIO_Port,LED_SD_Pin,GPIO_PIN_SET);
  HAL_TIM_Base_Start_IT(&htim1); 
  HAL_TIM_Base_Start_IT(&htim3);
  prev_sd = __HAL_TIM_GET_COUNTER(&htim2);
  prev_new_file_time = __HAL_TIM_GET_COUNTER(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // continue;
    volatile int sd_now = __HAL_TIM_GET_COUNTER(&htim2);
    // uint32_t whileTest = sd_now;
    if (sd_now - prev_new_file_time >= 3000000)
    {
      f_close(&SD_File);
      snprintf(TextFPath, sizeof(TextFPath),FILENAME,(int)SD_log_date.Year+2000,(int)SD_log_date.Month,(int)SD_log_date.Date,(int)SD_log_time.Hours,(int)SD_log_time.Minutes,(int)SD_log_time.Seconds);
      // snprintf(TextFPath, sizeof(TextFPath),"text.bin");
      res = f_open(&SD_File,TextFPath,FA_CREATE_ALWAYS|FA_WRITE);
      if(res == FR_OK)
      {
        f_close(&SD_File);
      }
      f_open(&SD_File,TextFPath,FA_OPEN_APPEND|FA_WRITE);
      prev_new_file_time = sd_now;
    }    
    if (sd_now - prev_sd >= 1000)
    {
      HAL_GPIO_WritePin(LED_SD_GPIO_Port,LED_SD_Pin,GPIO_PIN_SET);

      if(prev_got_date != got_date)
      {
        f_close(&SD_File);
        snprintf(TextFPath, sizeof(TextFPath),FILENAME,(int)SD_log_date.Year+2000,(int)SD_log_date.Month,(int)SD_log_date.Date,(int)SD_log_time.Hours,(int)SD_log_time.Minutes,(int)SD_log_time.Seconds);
        // snprintf(TextFPath, sizeof(TextFPath),"text.bin");
        res = f_open(&SD_File,TextFPath,FA_CREATE_ALWAYS|FA_WRITE);
        if(res == FR_OK)
        {
          f_close(&SD_File);
        }
        f_open(&SD_File,TextFPath,FA_OPEN_APPEND|FA_WRITE);
      }
            
      
      __disable_irq();
      uint8_t buf_num_to_sd = SD_wr_log_buf_num;
      uint16_t index_to_sd = SD_wr_log_index;
      SD_wr_log_buf_num^=0x1;
      SD_wr_log_index = 0;
      __enable_irq();

      if(max_sd_buf<index_to_sd)
      {
        max_sd_buf = index_to_sd;
      }

      res = f_write(&SD_File,SD_log_buf[buf_num_to_sd],index_to_sd*sizeof(logger_t),(void *)&byteswritten);
      while (res != FR_OK)
      {
        if (res == FR_DISK_ERR)
        {
          f_open(&SD_File,TextFPath,FA_OPEN_APPEND|FA_WRITE);
        }
        else
        {
          break;
        }
        res = f_write(&SD_File,SD_log_buf[buf_num_to_sd],index_to_sd*sizeof(logger_t),(void *)&byteswritten);
        if (res == FR_OK)
        {
          break;
        }
      }
      f_sync(&SD_File);

      prev_got_date = got_date;

      #ifdef TIMING
      sd_td = __HAL_TIM_GET_COUNTER(&htim2) - sd_now;
      if (max_sdwrite < sd_td)
      {
        max_sdwrite = sd_td;
      }      
      #endif
      prev_sd = sd_now;
      

      HAL_GPIO_WritePin(LED_SD_GPIO_Port,LED_SD_Pin,GPIO_PIN_RESET);
    }
    if (sd_now - prev_sd < 0)
    {
      prev_sd = sd_now;
    }
    if (sd_now - prev_new_file_time >= 3000000)
    {
      prev_new_file_time = sd_now;
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 32;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SDMMC;
  PeriphClkInitStruct.PLL2.PLL2M = 3;
  PeriphClkInitStruct.PLL2.PLL2N = 150;
  PeriphClkInitStruct.PLL2.PLL2P = 8;
  PeriphClkInitStruct.PLL2.PLL2Q = 20;
  PeriphClkInitStruct.PLL2.PLL2R = 4;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim1)
  {
    if(!nturt_inv_toggle_and_get_run_state(&nih)) return; // skip once every two trigger

    HAL_GPIO_TogglePin(LED_TIM_GPIO_Port,LED_TIM_Pin);
    uint16_t ADC1_arr[4] = {0};
    uint16_t ADC2_arr[4] = {0};
    uint16_t ADC3_arr[6] = {0};
    // Read ADC values from DMA buffers
    SCB_InvalidateDCache_by_Addr(DMA_ADC1_arr,sizeof(DMA_ADC1_arr));
    for (size_t i = 0; i < 4; i++)
    {
      ADC1_arr[i] = DMA_ADC1_arr[i];
    }    
    SCB_InvalidateDCache_by_Addr(DMA_ADC2_arr,sizeof(DMA_ADC2_arr));
    for (size_t i = 0; i < 4; i++)
    {
      ADC2_arr[i] = DMA_ADC2_arr[i];
    }    
    SCB_InvalidateDCache_by_Addr(DMA_ADC3_arr,sizeof(DMA_ADC3_arr));
    for (size_t i = 0; i < 6; i++)
    {
      ADC3_arr[i] = DMA_ADC3_arr[i];
    }    
    if(nih.inv_state == STATE_RUNNING)
    {      
      float delta = percent_trq_request - prev_percent_trq_request;
      delta = _constrain(delta,-max_ramp,max_ramp);
      if(prev_percent_trq_request != 0.0f && percent_trq_request == 0.0f)
      {
        if(fast_stop_enable)
        {
          PID_integral_reset(&pid_controller_current_Iq);
          PID_integral_reset(&pid_controller_current_Id);
          for (size_t i = 0; i < 3; i++)
          {
            PID_integral_reset(&pid_controller_current_Iabc[i]);
          }
        }
        prev_percent_trq_request = 0.0f;
      }
      prev_percent_trq_request = prev_percent_trq_request + delta;
      prev_percent_trq_request = percent_trq_request;
      HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_SET);
    }
    else
    {
      prev_percent_trq_request = 0.0f;
      HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
    }
    if(nih.inv_state == STATE_READY|| nih.inv_state == STATE_ERROR)
    {
      PID_reset(&pid_controller_current_Id);
      PID_reset(&pid_controller_current_Iq);
      PID_reset(&pid_controller_current_OCP);
      PID_reset(&pid_controller_current_Ia);
      for (size_t i = 0; i < 3; i++)
      {
        PID_reset(&pid_controller_current_Iabc[i]);
      }
      
      // pid_controller_current_Iq.output_prev = dir*filt_RPM/electrical_constant;
    }
    
    #ifdef TIMING
    uint32_t tick_start = __HAL_TIM_GET_COUNTER(&htim5);     
    #endif

    indexLED++;
    indexHeartbeat++;
    indexStatus++;
    CAN_Timer++;
    
    float angle_pll;
    nturt_inv_update_encoder_angle_now(&nih, ADC2_arr,&angle_pll);
    int8_t enc_err = 0;
    // angle_now = angle_pll;
    if(nih.angle_encoder.angle_now != nih.angle_encoder.angle_now)
    {
      enc_err = 1;
    }
    for (size_t i = 0; i < 4; i++)
    {
      if(ADC2_arr[i] < ENC_UV)
      {
        #ifndef OPEN_LOOP_SPEED
        Enter_ERROR_State(ERROR_ENC);
        #endif
      }
    }
    nih.angle_encoder.enc_sum -= nih.angle_encoder.enc_buf[nih.angle_encoder.enc_index];
  
    nih.angle_encoder.enc_buf[nih.angle_encoder.enc_index] = enc_err;
    nih.angle_encoder.enc_sum += nih.angle_encoder.enc_buf[nih.angle_encoder.enc_index];
  
    if (nih.angle_encoder.enc_sum > ENC_TIME/2)
  
    {
      if (nih.inv_state == STATE_RUNNING)
      {
        #ifndef OPEN_LOOP_SPEED
        Enter_ERROR_State(ERROR_ENC);
        #endif
      }    
    }

    nih.angle_encoder.angle_now = _normalizeAngle(nih.angle_encoder.angle_now);

    #ifdef OPEN_LOOP_SPEED
    angle_now = _normalizeAngle(last_angle + open_loop_rpm_var/ 60.0f / freq * 2 * M_PI * 4);
    last_angle = angle_now;
    // setPhaseVoltage(Iq_controller_output, Id_controller_output, _electricalAngle(angle_now, pole_pairs),TIM1);    
    #endif
    
    voltage_power_supply = (float)ADC3_arr[0]*DCVPLSB;
    nih.PID.V_max = voltage_power_supply;
    // float filtered_angle = LowPassFilter_operator(angle_now,&filter);
    
    float angular_vel = 0.0f;
    float raw_RPM = (float)nih.angle_encoder.enc_dir * angular_vel/4/2/M_PI*60;
    filt_RPM = LowPassFilter_operator((float)nih.angle_encoder.enc_dir * angular_vel/4/2/M_PI*60,&filter_RPM);
    // float target_torque = max_torque*prev_percent_trq_request;
    float max_derate = _constrain((float)DERATE_END/DERATE_START-(float)abs(filt_RPM)/DERATE_START,0.0f,1.0f);
    // prev_percent_trq_request = _constrain(prev_percent_trq_request,-max_derate,max_derate);
    float target_Iq = max_current*prev_percent_trq_request;
    float filtered_Iabc[3] = {0.0f};
    float current_phase_dc[3] = {0.0f};
    int8_t soft_oc_detected = 0;
    oc_sw_sum = oc_sw_sum - oc_sw_buf[oc_sw_index];
    for (int i=0;i<3;i++){
       	current_phase[i] =(float) (ADC1_arr[i]-current_offset[i])*phase_amp_per_LSB;
        //OCP
        filtered_Iabc[i] = LowPassFilter_operator(current_phase[i],&filter_current_Iabc[i]);
        current_phase_dc[i] = LowPassFilter_operator(current_phase[i],&filter_crrent_DC_Iabc[i]);
        if (current_phase[i] > ACAOCP||current_phase[i] < -ACAOCP)
        {
          soft_oc_detected = 1;
        }
        // current_phase[i] = filtered_Iabc[i];
    }
    oc_sw_buf[oc_sw_index] = soft_oc_detected;
    oc_sw_sum += oc_sw_buf[oc_sw_index];
    if (oc_sw_sum > SOFT_OC_TIME/2)
    {
      if (nih.inv_state == STATE_RUNNING)
      {
        Enter_ERROR_State(ERROR_INSTANT_OC);
      }    
    }
    oc_sw_index++;
    if(oc_sw_index == SOFT_OC_TIME)
    {
      oc_sw_index = 0;
    }

    pid_controller_current_Ia.limit = nih.PID.V_max;
    pid_controller_current_Id.limit = nih.PID.V_max;
    pid_controller_current_Iq.limit = nih.PID.V_max;
    pid_controller_current_OCP.limit = nih.PID.V_max;
    pid_controller_current_Iabc[0].limit = nih.PID.V_max;
    pid_controller_current_Iabc[1].limit = nih.PID.V_max;
    pid_controller_current_Iabc[2].limit = nih.PID.V_max;
    
    float Id,Iq;
    cal_Idq(current_phase, _electricalAngle(nih.angle_encoder.angle_now, nih.angle_encoder.pole_pairs), &Id, &Iq);
    filt_Iq=LowPassFilter_operator(Iq,&filter_current_Iq);
    filt_Id=LowPassFilter_operator(Id,&filter_current_Id);
    // filt_Iq = Iq;
    // filt_Id = Id;
    Ia = sqrt(filt_Id*filt_Id+filt_Iq*filt_Iq);

    Iq_controller_output=PID_operator(target_Iq-filt_Iq,&pid_controller_current_Iq);
    Id_controller_output=PID_operator(-filt_Id,&pid_controller_current_Id);
    
    float Iabc_controller_output[3] = {0.0f};
    for (size_t i = 0; i < 3; i++)
    {
      Iabc_controller_output[i] = PID_operator(-current_phase_dc[i],&pid_controller_current_Iabc[i]);
    }
    if(abs(filt_RPM) > 1000 && enable_dc_control == 0)
    {
      enable_dc_control = 1;
    }
    else if (abs(filt_RPM) < 750 && enable_dc_control == 1)
    {
      enable_dc_control = 0;
    }    
    if(!enable_dc_control)
    {
      for (size_t i = 0; i < 3; i++)
      {
        Iabc_controller_output[i] = 0.0f;
        PID_reset(&pid_controller_current_Iabc[i]);
      } 
    }
    
    float IqOC_controller_output;
    if(Iq_controller_output > 0)
    {
      IqOC_controller_output = _constrain(PID_operator(SOFTOCP-Ia,&pid_controller_current_OCP),-Iq_controller_output,0);
    }
    else
    {
      IqOC_controller_output = _constrain(PID_operator(-SOFTOCP-Ia,&pid_controller_current_OCP),0,-Iq_controller_output);
    }
    
    // setPhaseVoltage(_constrain(Iq_controller_output+IqOC_controller_output,-voltage_power_supply/2,voltage_power_supply/2),  _constrain(Id_controller_output,-voltage_power_supply/2,voltage_power_supply/2), _electricalAngle(angle_now, pole_pairs),TIM1);
    // setPhaseVoltage(percent_trq_request*100, 0, _electricalAngle(angle_now, pole_pairs),TIM1);
    setPhaseVoltage(Iq_controller_output, Id_controller_output, _electricalAngle(nih.angle_encoder.angle_now, nih.angle_encoder.pole_pairs),TIM1,-Iabc_controller_output[0],-Iabc_controller_output[1],-Iabc_controller_output[2]);
    

    if (indexLED == freq/2)
    {
    	if(nih.inv_state == STATE_READY)
      {
        HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin);
      }
      else if(nih.inv_state == STATE_ERROR)
      {
        HAL_GPIO_TogglePin(LED_ERR_GPIO_Port,LED_ERR_Pin);
      }
      
    	// HAL_GPIO_TogglePin(LED_ERR_GPIO_Port, LED_ERR_Pin);
      // UART_TX_Send(&huart1,"ping");
    	indexLED=0;
    }

    nih.DCbus.report_DCV = (uint16_t) roundf(voltage_power_supply*100);
    int16_t report_DCA = (int16_t) roundf((float)(ADC1_arr[3]-current_offset[3])*DCAPLSB*100);
    if (indexHeartbeat == freq/10)
    {
      CAN_Send_Temp(ADC3_arr);
      CAN_Send_State(nih.DCbus.report_DCV,report_DCA);
      CAN_Send_Heartbeat();
      CAN_Send_Perameter();
      // CAN_Send_Heartbeat();
      indexHeartbeat = 0;
    }

    uint16_t report_status = 0;
    if(nih.inv_state == STATE_READY) 
    {
      report_status |= REPORT_STATUS_READY;
    }else if(nih.inv_state == STATE_RUNNING)
    {
      report_status |= REPORT_STATUS_ENABLED;
    }else if(nih.inv_state == STATE_ERROR)
    {
      report_status |= REPORT_STATUS_FAULT;
    }
    if(ADC3_arr[0]*DCVPLSB > 55)
    {
      report_status |= REPORT_STATUS_HV;
    }

    if (indexStatus == freq/100)
    {
      int16_t report_RPM = (int16_t) roundf(filt_RPM);
      int16_t report_torque = (int16_t) roundf(filt_Iq/max_current*1000);
      CAN_Send_Status(report_status,report_torque,report_RPM);
      indexStatus = 0;
    }

    //Logging
    int16_t IU_100 = (int16_t)roundf(current_phase[0]*100);
    int16_t IV_100 = (int16_t)roundf(current_phase[1]*100);
    int16_t IW_100 = (int16_t)roundf(current_phase[2]*100);
    HAL_RTC_GetDate(&hrtc,&SD_log_date,RTC_FORMAT_BIN);
    HAL_RTC_GetTime(&hrtc,&SD_log_time,RTC_FORMAT_BIN);
    if(SD_log_time.Seconds != SD_log_prev_sec)
    {
      SD_log_subsec = 0;
      SD_log_prev_sec = SD_log_time.Seconds;
    }
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGHR = SD_log_time.Hours;
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGMIN = SD_log_time.Minutes;
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGSEC = SD_log_time.Seconds;
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGERR = nih.err_state;
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGSUBSEC = SD_log_subsec;
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGDCV = nih.DCbus.report_DCV;
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGDCA = report_DCA;
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGIU = IU_100;
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGIV = IV_100;
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGIW = IW_100;
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGVQ = (int16_t) roundf(Iq_controller_output*10);
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGVD = (int16_t) roundf(Id_controller_output*10);
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGSINE = ADC2_arr[0]-ADC2_arr[1];
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGCOS = ADC2_arr[2]-ADC2_arr[3];
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGANG = (uint16_t) roundf(nih.angle_encoder.angle_now*100*180/M_PI);
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGTCMD = (int16_t) roundf(prev_percent_trq_request*1000);
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGSTATE = report_status;
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGVU = TIM1->CCR1;
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGVV = TIM1->CCR2;
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGVW = TIM1->CCR3;
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGRPM = (int16_t) roundf(filt_RPM);
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGID = (int16_t) roundf(filt_Id*100);
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGIQ = (int16_t) roundf(filt_Iq*10);
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGZERO = (uint16_t) oc_sw_sum;
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGDCIU = (int16_t) roundf(current_phase_dc[0]*100);
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGDCIV = (int16_t) roundf(current_phase_dc[1]*100);
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGDCIW = (int16_t) roundf(current_phase_dc[2]*100);
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGVA = (int16_t) roundf(angle_pll*10);
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGVB = (int16_t) roundf(0);
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGVC = (int16_t) roundf(Iabc_controller_output[2]*10);
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGRMSIU = (uint16_t) roundf(-1);
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGRMSIV = (uint16_t) roundf(-1);
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGRMSIW = (uint16_t) roundf(-1);
    SD_log_buf[SD_wr_log_buf_num][SD_wr_log_index%3600].LGLOGBUF = SD_wr_log_index;

        
    SD_log_subsec++;
    SD_wr_log_index++;

    if(nih.inv_state == STATE_RUNNING)
    {
      oc_hw_sum -= oc_hw_buf[oc_hw_index];
      if (HAL_GPIO_ReadPin(OC_Fault_GPIO_Port,OC_Fault_Pin) == GPIO_PIN_RESET)
      {
        oc_hw_buf[oc_hw_index] = 1;
      }
      else
      {
        oc_hw_buf[oc_hw_index] = 0;
      }
      oc_hw_sum += oc_hw_buf[oc_hw_index];
      if (oc_hw_sum > HW_OC_TIME/2)
      {
        Enter_ERROR_State(ERROR_HW_OC);
      }
    }
    oc_hw_index++;
    if(oc_hw_index == HW_OC_TIME)
    {
      oc_hw_index = 0;
    }

    #ifdef CAN_OT_FAULT
    //CAN fault detect
    if(CAN_Timer == freq && nih.inv_state == STATE_RUNNING)
    {
      enable_hw_oc = 0;
      HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LED_ERR_GPIO_Port,LED_ERR_Pin,GPIO_PIN_RESET);
      nih.inv_state = STATE_READY;
    }
    #endif

    #ifdef TIMING
    loop_time = __HAL_TIM_GET_COUNTER(&htim5)-tick_start;
    if (loop_time > max_time)
    {
      max_time = loop_time;
    }
    if (loop_time < min_time)
    {
      min_time = loop_time;
    }
    int btw_time = tick_start - prev_time;
    if (btw_time > max_btw)
    {
      max_btw = btw_time;
    }
    prev_time = tick_start;
    if (indexTimer == freq*10)
    {
      max_time = 0;
      min_time = INT16_MAX;
      max_btw = 0;
      indexTimer = 0;
      max_sdwrite = 0;
    }
    indexTimer++;
    #endif

    HAL_GPIO_TogglePin(LED_TIM_GPIO_Port,LED_TIM_Pin);
  }  
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN)
{
  if(GPIO_PIN == GPIO_PIN_3)
  {
    Enter_ERROR_State(ERROR_GATE);
  }
  if(GPIO_PIN == GPIO_PIN_6)
  {
    if (nih.inv_state == STATE_RUNNING)
    {
      // Enter_ERROR_State(ERROR_HW_OC);
    }     
  }
  if(GPIO_PIN == GPIO_PIN_5)
  {
    // f_write(&SD_File,write_buffer,sizeof(write_buffer),&written);
    f_sync(&SD_File);
    while (1)
    {
      /* code */
    }    
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	isSent = 1;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)!=0)
  {
    HAL_GPIO_TogglePin(LED_CAN_GPIO_Port,LED_CAN_Pin);
    uint8_t RxData1[6];
    HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0,&RxHeader1,RxData1);
    if(hfdcan->Instance == FDCAN1)
    {
      if(RxHeader1.Identifier == CAN_ID_CONTROL+MOT_ID)
      {
        int16_t torque_command;
        control = RxData1[0] | (uint16_t)RxData1[1] << 8;
        if(nih.inv_state != STATE_ERROR)
        {
          // enable
          if(control & CTRL_ENABLE && voltage_power_supply >= 20 && nih.inv_state == STATE_READY && HAL_GPIO_ReadPin(GATE_Ready_GPIO_Port,GATE_Ready_Pin) == GPIO_PIN_SET )
          {
            nih.inv_state = STATE_RUNNING;
            HAL_GPIO_WritePin(LED_ERR_GPIO_Port,LED_ERR_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED_RUN_GPIO_Port,LED_RUN_Pin,GPIO_PIN_SET);
            percent_trq_request = 0;
            enable_hw_oc = 1;
            for (size_t i = 0; i < HW_OC_TIME; i++)
            {
              oc_hw_buf[i] = 0;
            }
            oc_hw_sum = 0;
            oc_hw_index = 0;

            oc_sw_sum = 0;
            oc_sw_index = 0;
            for (size_t i = 0; i < SOFT_OC_TIME; i++)
            {
              oc_sw_buf[i] = 0;
            }

            nih.angle_encoder.enc_sum = 0;
          
            nih.angle_encoder.enc_index = 0;
            for (size_t i = 0; i < ENC_TIME; i++)
            {
              nih.angle_encoder.enc_buf[i] = 0;
            }
            
            
            HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_SET);
          // disable
          }else if(!(control & CTRL_ENABLE) && nih.inv_state == STATE_RUNNING) 
          {
            nih.inv_state = STATE_READY;
            HAL_GPIO_WritePin(LED_ERR_GPIO_Port,LED_ERR_Pin,GPIO_PIN_RESET);
            percent_trq_request = 0;
            enable_hw_oc = 0;
            HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
          }
          else
          {
            enable_hw_oc = 0;
            HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
          }
        }
        // fault reset
        if((control & CTRL_FAULT_RESET) && nih.inv_state == STATE_ERROR ) 
        {
          nih.inv_state = STATE_READY;
          nih.err_state = ERROR_NONE;
        }
        torque_command = RxData1[2] | RxData1[3] << 8;
        percent_trq_request = (float)torque_command/1000;
        CAN_Timer = 0;
      }      
    }
  }  
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
  if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE)!=0)
  {
    HAL_GPIO_TogglePin(LED_CAN_GPIO_Port,LED_CAN_Pin);
    uint8_t RxData1[16];
    HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO1,&RxHeader1,RxData1);
    if(hfdcan->Instance == FDCAN1)
    {
      if(RxHeader1.Identifier == 0x100)
      {
        volatile uint32_t sec_from_midnight = RxData1[0] | RxData1[1] << 8 | RxData1[2] << 16 | RxData1[3] << 24;
        volatile uint16_t day_from_1984 = RxData1[4] | RxData1 [5] << 8;
        sec_from_midnight/=1000;
        time_t now = day_from_1984*86400+sec_from_midnight+441763200;
        struct tm now_tm;
        gmtime_r(&now,&now_tm);

        set_time(now_tm.tm_hour,now_tm.tm_min,now_tm.tm_sec);
        set_date(now_tm.tm_year-100,now_tm.tm_mon+1,now_tm.tm_mday,now_tm.tm_wday);
        got_date = 1;
      }
      else if (RxHeader1.Identifier == 0x200)
      {
        if(nih.inv_state == STATE_RUNNING)
        {
          return;
        }
        float val;
        uint8_t bytes[4] = {RxData1[1],RxData1[2],RxData1[3],RxData1[4]};
        memcpy(&val,&bytes,sizeof(val));
        switch (RxData1[0])
        {
        case 0x1:
          pid_controller_current_Iq.P = val;  
          break;

        case 0x2:
          pid_controller_current_Iq.I = val;
          break;
        
        case 0x3:
          pid_controller_current_Iq.D = val;
          break;

        case 0x11:
          pid_controller_current_Id.P = val;
          break;

        case 0x12:
          pid_controller_current_Id.I = val;
          break;
        
        case 0x13:
          pid_controller_current_Id.D = val;
          break;

        case 0x21:
          for (size_t i = 0; i < 3; i++)
          {
            pid_controller_current_Iabc[i].P = val;
          }
          break;
        
        case 0x22:
          for (size_t i = 0; i < 3; i++)
          {
            pid_controller_current_Iabc[i].I = val;
          }
          break;
        
        case 0x23:
          for (size_t i = 0; i < 3; i++)
          {
            pid_controller_current_Iabc[i].D = val;
          }
          break;

        case 0x31:
          nih.angle_encoder.zero_electric_angle = val;
          break;
        
        case 0x41:
          fast_stop_enable = (uint8_t) val;
          break;

        default:
          break;
        }
      }
      else if (RxHeader1.Identifier == 0x720+MOT_ID)
      {
        float val;
        uint8_t bytes[4] = {RxData1[1],RxData1[2],RxData1[3],RxData1[4]};
        memcpy(&val,&bytes,sizeof(val));
        switch (RxData1[0])
        {
        case 0x21:
          nih.Mot_T.cal_par_I = val;
          for (size_t i = 0; i < 1024; i++)
          {
            nih.Mot_T.adc_to_Mot_T_lut[i] = (int16_t)10*((float)(1650-(3300*i/1024))/nih.Mot_T.cal_par_I/3.795-1000/3.795);
          }
        
        default:
          break;
        }
      }
    }
  }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
  if(hfdcan->Instance == FDCAN1)
  {
    HAL_GPIO_TogglePin(LED_CAN_ERR_GPIO_Port,LED_CAN_ERR_Pin);
    MX_FDCAN1_Init();
    Config_Fdcan1();
  }
}

void Config_Fdcan1(void)
{
  FDCAN_FilterTypeDef CAN1RxFilterConfig;
  CAN1RxFilterConfig.IdType = FDCAN_STANDARD_ID;
  CAN1RxFilterConfig.FilterIndex = 0;
  CAN1RxFilterConfig.FilterType = FDCAN_FILTER_DUAL;
  CAN1RxFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  CAN1RxFilterConfig.FilterID1 = CAN_ID_CONTROL+MOT_ID;
  CAN1RxFilterConfig.FilterID2 = 0x7FF;
  CAN1RxFilterConfig.RxBufferIndex = 0;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1,&CAN1RxFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  // FDCAN_FilterTypeDef CAN1RxFilterConfig;
  CAN1RxFilterConfig.IdType = FDCAN_STANDARD_ID;
  CAN1RxFilterConfig.FilterIndex = 1;
  CAN1RxFilterConfig.FilterType = FDCAN_FILTER_DUAL;
  CAN1RxFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  CAN1RxFilterConfig.FilterID1 = 0x100;
  CAN1RxFilterConfig.FilterID2 = 0x7FF;
  CAN1RxFilterConfig.RxBufferIndex = 0;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1,&CAN1RxFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  #ifdef CAN_CONFIG
  CAN1RxFilterConfig.IdType = FDCAN_STANDARD_ID;
  CAN1RxFilterConfig.FilterIndex = 2;
  CAN1RxFilterConfig.FilterType = FDCAN_FILTER_DUAL;
  CAN1RxFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  CAN1RxFilterConfig.FilterID1 = 0x200;
  CAN1RxFilterConfig.FilterID2 = 0x7FF;
  CAN1RxFilterConfig.RxBufferIndex = 0;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1,&CAN1RxFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  #endif

  CAN1RxFilterConfig.IdType = FDCAN_STANDARD_ID;
  CAN1RxFilterConfig.FilterIndex = 3;
  CAN1RxFilterConfig.FilterType = FDCAN_FILTER_DUAL;
  CAN1RxFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  CAN1RxFilterConfig.FilterID1 = 0x720+MOT_ID;
  CAN1RxFilterConfig.FilterID2 = 0x7FF;
  CAN1RxFilterConfig.RxBufferIndex = 0;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1,&CAN1RxFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_FULL,0) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1,FDCAN_RX_FIFO0,FDCAN_RX_FIFO_OVERWRITE);
  if (HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO1_NEW_MESSAGE,0) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_BUS_OFF,0) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_FDCAN_Start(&hfdcan1);
}

void Enter_ERROR_State(INV_Errortypedef error)
{
  nih.inv_state = STATE_ERROR;
  nih.err_state = error;
  enable_hw_oc = 0;
  HAL_GPIO_WritePin(Motor_Enable_GPIO_Port,Motor_Enable_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_RUN_GPIO_Port,LED_RUN_Pin,GPIO_PIN_RESET);
  LowPassFilter_reset(&filter_current_Iabc[0]);
  LowPassFilter_reset(&filter_current_Iabc[1]);
  LowPassFilter_reset(&filter_current_Iabc[2]);
  LowPassFilter_reset(&filter_current_Iq);
  LowPassFilter_reset(&filter_current_Id);
  // LowPassFilter_reset(&filter_RPM);
}

void CAN_Send_State(uint16_t DCV, int16_t DCA)
{
  uint8_t StateData[4];
  StateData[0] = DCV;
  StateData[1] = DCV >> 8;
  StateData[2] = DCA;
  StateData[3] = DCA >> 8;
  CAN1_SetMsg(&StateHeader,StateData);
}

void CAN_Send_Status(uint16_t report_status ,int16_t FB_Torque, int16_t Speed)
{
  uint8_t StatusData[6];
  StatusData[0] = report_status;
  StatusData[1] = nih.err_state;
  StatusData[2] = FB_Torque;
  StatusData[3] = FB_Torque >> 8;
  StatusData[4] = Speed;
  StatusData[5] = Speed >> 8;
  // StatusData[6] = err_state;
  CAN1_SetMsg(&StatusHeader,StatusData);
}

void CAN_Send_Temp(uint16_t ADC_arr[6])
{
  uint8_t TempData[6];
  nih.Mot_T.T = nih.Mot_T.adc_to_Mot_T_lut[ADC_arr[2]>>6];
  nih.MCU_T.T = nih.MCU_T.adc_to_MCU_T_lut[ADC_arr[1]>>6];
  nih.SIC_T.U = nih.SIC_T.adc_to_SIC_T_lut[ADC_arr[3]>>6];
  nih.SIC_T.V = nih.SIC_T.adc_to_SIC_T_lut[ADC_arr[4]>>6];
  nih.SIC_T.W = nih.SIC_T.adc_to_SIC_T_lut[ADC_arr[5]>>6];
  nih.SIC_T.max = maxint16(nih.SIC_T.U,maxint16(nih.SIC_T.V,nih.SIC_T.W));
  //OTP
  if(maxint16(nih.SIC_T.max,nih.MCU_T.T) > MOS_OTP)
  {
    // Enter_ERROR_State(ERROR_INV_OT);
  }
  if(nih.Mot_T.T > MOT_OTP || nih.Mot_T.T < MOT_UTP)
  {
    Enter_ERROR_State(ERROR_MOT_OT);
  }
  TempData[0] = nih.SIC_T.max;
  TempData[1] = nih.SIC_T.max >> 8;
  TempData[2] = nih.MCU_T.T;
  TempData[3] = nih.MCU_T.T >> 8;
  TempData[4] = nih.Mot_T.T;
  TempData[5] = nih.Mot_T.T >> 8;
  CAN1_SetMsg(&TempHeader,TempData);
}

void set_time (uint8_t hr, uint8_t min, uint8_t sec)
{
	RTC_TimeTypeDef sTime = {0};
	sTime.Hours = hr;
	sTime.Minutes = min;
	sTime.Seconds = sec;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}
}

void set_date (uint8_t year, uint8_t month, uint8_t date, uint8_t day)  // monday = 1
{
	RTC_DateTypeDef sDate = {0};
	sDate.WeekDay = day;
	sDate.Month = month;
	sDate.Date = date;
	sDate.Year = year;
	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x2345);  // backup register
}

void CAN_Send_Heartbeat(void)
{
  uint8_t HBData = 0x05;
  CAN1_SetMsg(&HeartBeatHeader,&HBData);
}

void CAN_Send_Perameter(void)
{
  static uint8_t count;
  count++;
  if(count == 9)
  {
    count = 0;
  }
  uint8_t PeramData[5] = {0};
  PeramData[0] = count+1;
  uint8_t bytes[4] = {0};
  float val = 0.0f;
  uint8_t head = 0x0;
  switch(count)
  {
    case 0:
      head = 0x1;
      val = pid_controller_current_Iq.P;
      break;

    case 1:
      head = 0x2;
      val = pid_controller_current_Iq.I;
      break;
    
    case 2:
      head = 0x3;
      val = pid_controller_current_Iq.D;
      break;

    case 3:
      head = 0x11;
      val = pid_controller_current_Id.P;
      break;

    case 4:
      head = 0x12;
      val = pid_controller_current_Id.I;
      break;
    
    case 5:
      head = 0x13;
      val = pid_controller_current_Id.D;
      break;

    case 6:
      head = 0x21;
      val = pid_controller_current_Iabc[0].P;
      break;

    case 7:
      head = 0x22;
      val = pid_controller_current_Iabc[0].I;
      break;

    case 8:
      head = 0x23;
      val = pid_controller_current_Iabc[0].D;
      break;

    default:
      break;
  }
  PeramData[0] = head;
  PeramData[0] = count+1;
  memcpy(&bytes,&val,4);
  for (size_t i = 0; i < 4; i++)
  {
    PeramData[i+1] = bytes[i];
  }  
  CAN1_SetMsg(&PerameterHeader,PeramData);
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x24000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
