// ECE 6110 - Final Project - IoT Optical Heart Rate Plotter
#include "main.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "MAX30105.h"
#include "heartRate.h"

#define SSID     "Pixel_8244"
#define PASSWORD "6159160679"
#define PORT           80

#define WIFI_WRITE_TIMEOUT 10000
#define WIFI_READ_TIMEOUT  10000
#define SOCKET                 0

// The entire HTML webpage is stored in this char array,
// better make it big
//static  uint8_t http[16384];
static  uint8_t  IP_Addr[4];

static uint8_t  currentTemp = 0;
static uint16_t currentDist = 0;

char bufferBad[50] = {0};
char bpmDisplay[50] = {0};

// Damping factor for the webserver
int dampingFactor = 100;

// This stores the portion of the javascript responsible for
// holding the heart rate values. This needs to be separate so it
// can be formatted using sprintf
char graphArray[300] = {0};

//Increase this for more averaging. 4 is good.
const uint8_t RATE_SIZE = 4;

//Array of heart rates
uint8_t rates[RATE_SIZE];
uint8_t rateSpot = 0;

//Time at which the last beat occurred
long lastBeat = 0;

// Current bpm
float beatsPerMinute;

// Average of the previous bpm readings
int beatAvg;

// This array is 10 samples long, for 10 seconds
// of heart rate data
uint8_t beatArray[10] = {0};

// I2C handle for the heart rate monitor
I2C_HandleTypeDef hi2c1;

// Timer handle for the timer interrupt
TIM_HandleTypeDef htim16;

// UART handle so we can send data over the terminal
UART_HandleTypeDef huart1;

// Initializes the heart rate sensor (this will call the constructor)
MAX30105 particleSensor;

// General peripheral and other system init functions
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART1_UART_Init(void);

// Nice function for printing over UART
void serialPrint(char buffer[]);

// All wifi-related functions
static  WIFI_Status_t SendWebPage( uint8_t temperature, uint16_t proxData);
static  int wifi_server(void);
static  int wifi_start(void);
static  int wifi_connect(void);
static  bool WebServerProcess(void);

int main(void)
{

  // HAL and system clock init
  HAL_Init();
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  // Initialize other peripherals
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();

  // The I2C read address of the heart rate sensor
  static const uint8_t MAX30102_READ_ADDRESS = 0xAF;

  // Starts the timer and enables interrupts (which trigger
  // depending on the timer's ARR)
  HAL_TIM_Base_Start(&htim16);
  __HAL_TIM_ENABLE_IT(&htim16, TIM_IT_UPDATE );

   // Initializes the heart-rate sensor
   particleSensor.begin(&hi2c1, MAX30102_READ_ADDRESS);
   particleSensor.setup();

    char bufferBad[50] = {0};

    serialPrint("****** WIFI Web Server Initialization ****** \n\r");

    // Calls the wifi server function -
    //
    // NOTE - once this is called, the server runs until it is stopped
    wifi_server();

    while(1) {

    }

}

// Wifi Stuff
static int wifi_start(void)
{
uint8_t  MAC_Addr[6];

/*Initialize and use WIFI module */
if(WIFI_Init() ==  WIFI_STATUS_OK)
{
  serialPrint("ES-WIFI Initialized.\n\r");
  if(WIFI_GetMAC_Address(MAC_Addr) == WIFI_STATUS_OK)
  {

    char macAdd[60] = {0};
    sprintf(macAdd,"> es-wifi module MAC Address : %X:%X:%X:%X:%X:%X\n\r",
            MAC_Addr[0],
            MAC_Addr[1],
            MAC_Addr[2],
            MAC_Addr[3],
            MAC_Addr[4],
            MAC_Addr[5]);

    serialPrint(macAdd);

  }
  else
  {
    serialPrint("> ERROR : CANNOT get MAC address\n\r");
    return -1;
  }
}
else
{
  return -1;
}
return 0;
}


int wifi_connect(void)
{

wifi_start();

char connecting[100] = {0};
sprintf(connecting,"\nConnecting to %s , %s\n\r",SSID,PASSWORD);
serialPrint(connecting);

if( WIFI_Connect(SSID, PASSWORD, WIFI_ECN_WPA2_PSK) == WIFI_STATUS_OK)
{
  if(WIFI_GetIP_Address(IP_Addr) == WIFI_STATUS_OK)
  {

    char connectMes[100] = {0};
    sprintf(connectMes,"> es-wifi module connected: got IP Address : %d.%d.%d.%d\n\r",
            IP_Addr[0],
            IP_Addr[1],
            IP_Addr[2],
            IP_Addr[3]);

    serialPrint(connectMes);

  }
  else
  {
		  serialPrint(" ERROR : es-wifi module CANNOT get IP address\n\r");
    return -1;
  }
}
else
{
		 serialPrint("ERROR : es-wifi module NOT connected\n\r");
   return -1;
}
return 0;
}


int wifi_server(void)
{
bool StopServer = false;

serialPrint("Running HTML Server test\n\r");
if (wifi_connect()!=0) return -1;


if (WIFI_STATUS_OK!=WIFI_StartServer(SOCKET, WIFI_TCP_PROTOCOL, 1, "", PORT))
{
  serialPrint("ERROR: Cannot start server.\n\r");
}

char logMes[100] = {0};
sprintf(logMes,"Server is running and waiting for an HTTP  Client connection to %d.%d.%d.%d\n\r",IP_Addr[0],IP_Addr[1],IP_Addr[2],IP_Addr[3]);
serialPrint(logMes);

do
{
  uint8_t RemoteIP[4];
  uint16_t RemotePort;

  while (WIFI_STATUS_OK != WIFI_WaitServerConnection(SOCKET,2,RemoteIP,&RemotePort))
  {

	  // Reset the damping count
	  int countTime = 0;

	  // This is a damping factor for the webserver process
	  // This causes the webserver to check for connections
	  // at a slower rate than normal, thus allowing more time for collecting
	  // optical IR samples, which needs to happened very rapidly
	  while (countTime < dampingFactor) {

	  long irValue = particleSensor.getIR();

	  	  if (checkForBeat(irValue) == true)
	  	  {
	  	    //We sensed a beat!
	  	    long delta = HAL_GetTick() - lastBeat;
	  	    lastBeat = HAL_GetTick();

	  	    beatsPerMinute = 60 / (delta / 1000.0);

	  	    if (beatsPerMinute < 255 && beatsPerMinute > 20)
	  	    {
	  	      rates[rateSpot++] = (uint8_t)beatsPerMinute; //Store this reading in the array
	  	      rateSpot %= RATE_SIZE; //Wrap variable

	  	      //Take average of readings
	  	      beatAvg = 0;
	  	      for (uint8_t x = 0 ; x < RATE_SIZE ; x++)
	  	        beatAvg += rates[x];
	  	      beatAvg /= RATE_SIZE;
	  	    }
	  	  }

	  	  // Set the average beat count to 0 if there is no finger placed on the
	  	  // sensor. Under normal light conditions, the IR value is about 1000 with no
	  	  // finger on the sensor, but should never rise above 5000
	  	  if (irValue < 5000) {

	  		  beatAvg = 0;

	  	  }

	  	  // Lol, of your heart-rate fits in this condition, sound an alarm
	  	  if (((beatAvg < 40)||(beatAvg > 120)) && (beatAvg != 0)) {

	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);

	  	  }

	  	  else {

	  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

	  	  }

	  	  sprintf(bpmDisplay, "IR= %d , BPM= %d , Avg BPM= %d \n\r", irValue, (uint8_t)beatsPerMinute, (uint8_t)beatAvg);

	  	  HAL_UART_Transmit(&huart1, (uint8_t*)bpmDisplay , strlen(bpmDisplay), HAL_MAX_DELAY);

	  	  countTime++;

	  }

  }

  char conMes[100] = {0};
  sprintf(conMes,"Client connected %d.%d.%d.%d:%d\n\r",RemoteIP[0],RemoteIP[1],RemoteIP[2],RemoteIP[3],RemotePort);
  serialPrint(conMes);

  StopServer=WebServerProcess();

  if(WIFI_CloseServerConnection(SOCKET) != WIFI_STATUS_OK)
  {
    serialPrint("ERROR: failed to close current Server connection\n\r");
    return -1;
  }
}
while(StopServer == false);

if (WIFI_STATUS_OK!=WIFI_StopServer(SOCKET))
{
  serialPrint("ERROR: Cannot stop server.\n\r");
}

serialPrint("Server is stop\n\r");
return 0;
}

static bool WebServerProcess(void)
{

uint16_t  respLen;

static   uint8_t resp[1024];
bool    stopserver=false;

if (WIFI_STATUS_OK == WIFI_ReceiveData(SOCKET, resp, 1000, &respLen, WIFI_READ_TIMEOUT))
{

 char getByte[40] = {0};
 sprintf(getByte, "get %d byte(s) from server\n\r",respLen);
 serialPrint(getByte);


 if( respLen > 0)
 {
    if(strstr((char *)resp, "GET")) /* GET: put web page */
    {

      if(SendWebPage( currentTemp, currentDist) != WIFI_STATUS_OK)
      {
        serialPrint("> ERROR : Cannot send web page\n\r");
      }
      else
      {
        serialPrint("Send page after  GET command\n\r");
      }
     }
     else if(strstr((char *)resp, "POST"))/* POST: received info */
     {
       serialPrint("Post request\n\r");

		// Put stuff here for POST requests

       if(strstr((char *)resp, "stop_server"))
       {
         if(strstr((char *)resp, "stop_server=0"))
         {
           stopserver = false;
         }
         else if(strstr((char *)resp, "stop_server=1"))
         {
           stopserver = true;
         }
       }

       if(SendWebPage( currentTemp, currentDist) != WIFI_STATUS_OK)
       {
         serialPrint("> ERROR : Cannot send web page\n\r");
       }
       else
       {
         serialPrint("Send Page after POST command\n\r");
       }
     }
   }
}
else
{
  serialPrint("Client close connection\n\r");
}
return stopserver;

}


// This is where the nice graph is created. Essentially, the graph is just javascript that
// plots a really slick-looking dynamic plot (the library I used called it a spline plot).
//
// The javascript basically includes a 2-D representation of the previous 10 seconds of
// hear-rate readings. The javascript also includes a 1-second auto-refresh, which automatically
// requests more data every 1 second. After the browser requests new data, this function is called
// and it constructs the new graph and then sends using the wifi drivers
static WIFI_Status_t SendWebPage( uint8_t temperature, uint16_t proxData)
{

uint16_t SentDataLength;
WIFI_Status_t ret;

// Reset the webpage to 0
uint8_t http5[16384] = {0};

//char jScript[] = "<!DOCTYPE HTML>\r\n<html>\r\n <meta http-equiv=\"refresh\" content=\"1\">\r\n<head>\r\n<script>\r\nwindow.onload = function () {\r\n\r\nvar dps = [];\r\nvar chart = new CanvasJS.Chart(\"chartContainer\", {\r\n\texportEnabled: true,\r\n\ttitle :{\r\n\t\ttext: \"Heart Rate Over Time\"\r\n\t},\r\n  axisY :{\r\n\t\ttitle: \"Average Beats\",\r\n\t\tsuffix: \"bpm\"\r\n\t},\r\n  axisX :{\r\n\t\ttitle: \"Last 10 Seconds\",\r\n\t\tsuffix: \"s\"\r\n\t},\r\n\tdata: [{\r\n\t\ttype: \"spline\",\r\n\t\tmarkerSize: 0,\r\n\t\tdataPoints: dps \r\n\t}]\r\n});\r\n\r\nvar updateInterval = 1000000;\r\nvar dataLength = 10; \r\n\r\nvar updateChart = function (count) {\r\n\r\n  dps.push({ x:10 ,y:1});\r\n  dps.push({ x:9 ,y:1});\r\n  dps.push({ x:8 ,y:4});\r\n  dps.push({ x:7 ,y:5});\r\n  dps.push({ x:6 ,y:7});\r\n  dps.push({ x:5 ,y:8});\r\n  dps.push({ x:4 ,y:10});\r\n  dps.push({ x:3 ,y:1});\r\n  dps.push({ x:2 ,y:1});\r\n  dps.push({ x:1 ,y:4});\r\n  \r\n  chart.render();\r\n};\r\n\r\nupdateChart(dataLength); \r\nsetInterval(function(){ updateChart() }, updateInterval); \r\n\r\n}\r\n</script>\r\n</head>\r\n<body>\r\n<div id=\"chartContainer\" style=\"height: 300px; width: 100%;\"></div>\r\n<script src=\"https://canvasjs.com/assets/script/canvasjs.min.js\"></script> \r\n</body>\r\n</html>";

// This is the start of the graph javacript
char jScript[] = "<!DOCTYPE HTML>\r\n<html>\r\n <meta http-equiv=\"refresh\" content=\"1\">\r\n<head>\r\n<script>\r\nwindow.onload = function () {\r\n\r\nvar dps = [];\r\nvar chart = new CanvasJS.Chart(\"chartContainer\", {\r\n\texportEnabled: true,\r\n\ttitle :{\r\n\t\ttext: \"Heart Rate Over Time\"\r\n\t},\r\n  axisY :{\r\n\t\ttitle: \"Average Beats\",\r\n\t\tsuffix: \"bpm\"\r\n\t},\r\n  axisX :{\r\n\t\ttitle: \"Last 10 Seconds\",\r\n\t\tsuffix: \"s\"\r\n\t},\r\n\tdata: [{\r\n\t\ttype: \"spline\",\r\n\t\tmarkerSize: 0,\r\n\t\tdataPoints: dps \r\n\t}]\r\n});\r\n\r\nvar updateInterval = 1000000;\r\nvar dataLength = 10; \r\n\r\nvar updateChart = function (count) {\r\n\r\n  ";

// This is where the new values are spliced into the existing graph-infrastructure, everything else
// is the same. Basically, this sprintf line accesses the previous 10 samples and formats them in the correct positions
// so that the javascript plot will recognize them as valid points on the graph
sprintf(graphArray,"dps.push({ x:10 ,y:%d});\r\n  dps.push({ x:9 ,y:%d});\r\n  dps.push({ x:8 ,y:%d});\r\n  dps.push({ x:7 ,y:%d});\r\n  dps.push({ x:6 ,y:%d});\r\n  dps.push({ x:5 ,y:%d});\r\n  dps.push({ x:4 ,y:%d});\r\n  dps.push({ x:3 ,y:%d});\r\n  dps.push({ x:2 ,y:%d});\r\n  dps.push({ x:1 ,y:%d});\r\n",beatArray[0], beatArray[1],beatArray[2],beatArray[3],beatArray[4],beatArray[5],beatArray[6],beatArray[7],beatArray[8],beatArray[9]);

// Just the end of the plot javascript
char jScriptEnd[] = "  \r\n  chart.render();\r\n};\r\n\r\nupdateChart(dataLength); \r\nsetInterval(function(){ updateChart() }, updateInterval); \r\n\r\n}\r\n</script>\r\n</head>\r\n<body>\r\n<div id=\"chartContainer\" style=\"height: 300px; width: 100%;\"></div>\r\n<script src=\"https://canvasjs.com/assets/script/canvasjs.min.js\"></script> \r\n</body>\r\n</html>";

// These next 3 lines are string concatenation, which basically just copies the strings
// contents to the http5, which holds all the values for the webpage
strcat((char *)http5, jScript);
strcat((char *)http5, graphArray);
strcat((char *)http5, jScriptEnd);


// Send the page off
ret = WIFI_SendData(0, (uint8_t *)http5, strlen((char *)http5), &SentDataLength, WIFI_WRITE_TIMEOUT);

if((ret == WIFI_STATUS_OK) && (SentDataLength != strlen((char *)http5)))
{
  ret = WIFI_STATUS_ERROR;
}

return ret;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

// This timer is configured for 1 second intervals
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 7999;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 9999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPSGRF_915_SPI3_CSN_GPIO_Port, SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A5_Pin ARD_A4_Pin ARD_A3_Pin ARD_A2_Pin
                           ARD_A1_Pin ARD_A0_Pin */
  GPIO_InitStruct.Pin = ARD_A5_Pin|ARD_A4_Pin|ARD_A3_Pin|ARD_A2_Pin
                          |ARD_A1_Pin|ARD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
  GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D4_Pin */
  GPIO_InitStruct.Pin = ARD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(ARD_D4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D7_Pin */
  GPIO_InitStruct.Pin = ARD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D3_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D6_Pin */
  GPIO_InitStruct.Pin = ARD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin
                           SPSGRF_915_SDN_Pin ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DFSDM1_DATIN2_Pin DFSDM1_CKOUT_Pin */
  GPIO_InitStruct.Pin = DFSDM1_DATIN2_Pin|DFSDM1_CKOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_DFSDM1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : QUADSPI_CLK_Pin QUADSPI_NCS_Pin OQUADSPI_BK1_IO0_Pin QUADSPI_BK1_IO1_Pin
                           QUAD_SPI_BK1_IO2_Pin QUAD_SPI_BK1_IO3_Pin */
  GPIO_InitStruct.Pin = QUADSPI_CLK_Pin|QUADSPI_NCS_Pin|OQUADSPI_BK1_IO0_Pin|QUADSPI_BK1_IO1_Pin
                          |QUAD_SPI_BK1_IO2_Pin|QUAD_SPI_BK1_IO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : INTERNAL_I2C2_SCL_Pin INTERNAL_I2C2_SDA_Pin */
  GPIO_InitStruct.Pin = INTERNAL_I2C2_SCL_Pin|INTERNAL_I2C2_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : INTERNAL_UART3_TX_Pin INTERNAL_UART3_RX_Pin */
  GPIO_InitStruct.Pin = INTERNAL_UART3_TX_Pin|INTERNAL_UART3_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin
                           PMOD_IRQ_EXTI12_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|ARD_D2_Pin|HTS221_DRDY_EXTI15_Pin
                          |PMOD_IRQ_EXTI12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_ID_Pin USB_OTG_FS_DM_Pin USB_OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_ID_Pin|USB_OTG_FS_DM_Pin|USB_OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : INTERNAL_SPI3_SCK_Pin INTERNAL_SPI3_MISO_Pin INTERNAL_SPI3_MOSI_Pin */
  GPIO_InitStruct.Pin = INTERNAL_SPI3_SCK_Pin|INTERNAL_SPI3_MISO_Pin|INTERNAL_SPI3_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
switch (GPIO_Pin)
{
  case (GPIO_PIN_1):
  {
    SPI_WIFI_ISR();
    break;
  }

  // This triggers if the blue-button
  // interrupt is triggered. This is the only
  // way to reset the alarm
  case (GPIO_PIN_13):
  	{

      	//alarm = false;

  	}
  default:
  {
    break;
  }
}
}

/**
* @brief  SPI3 line detection callback.
* @param  None
* @retval None
*/
void SPI3_IRQHandler(void)
{
HAL_SPI_IRQHandler(&hspi);
}


// Made a function for printing over the USART, as I was tired of copy-pasting
// the obscenely long HAL transmit function
void serialPrint(char buffer[]) {

	HAL_UART_Transmit(&huart1, (uint8_t*)buffer , strlen(buffer), HAL_MAX_DELAY);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check to make sure the correct timer called this interrupt
  if (htim == &htim16 )
  {

	  //serialPrint("Interrupt works! \n\r");

	  // Catch the current avg beat value
	  uint8_t heartBeatAvg  = (uint8_t)beatAvg;

	  // Shift the beat array over by 1
	  for (int i=0; i<9;i++) {

		  beatArray[9-i] = beatArray[8-i];

	  }

	  // Store the latest value
	  beatArray[0] = heartBeatAvg;

	  // The array has now been fully shifted by 1

  }

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
