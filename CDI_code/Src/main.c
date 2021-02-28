/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FLASH_PAGE.h"
#include "SerialComm.h"
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
#define nSteps                        64u
#define TDutyTriggerK               1309u     //1.0ms for clock 72MHz
#define TIntervPulseInv              655u     //0.5ms
#define TPulseInv                   4582u     //3,5ms
#define EngineSpeedPeriod_Min     785455u     //100rpm
#define EngineSpeedPeriod_Max       5236u     //15000rpm
#define TMR2_16bits                65536u
#define RPM_const               78545455u
#define FALSE                          0u
#define TRUE                           1u
#define OFF                            0u
#define ON                             1u

//Global variable
enum Interruption_type {INT_FROM_CH1, INT_FROM_CH2, INT_FROM_CH3, INT_FROM_CH4} int_types;
enum Event_status {EMPTY, PROGRAMMED} status;
enum Engine_States {STOPPED, CRANKING, ACCELERATION, STEADY_STATE, DECELERATION, OVERSPEED} engstates;
enum engineSpeed {LOW, HIGH} pulseMngmt;

typedef struct system_info
{
    uint16_t Engine_Speed_old;
    uint16_t Engine_Speed;
    uint16_t engineSpeedPred;
    uint16_t engineSpeedFiltered;
    uint16_t avarageEngineSpeed;
    int32_t  deltaEngineSpeed;
    uint32_t Rising_Edge_Counter;
    uint32_t triggerEventCounter;
    uint32_t inversorEventCounter;
    uint32_t Measured_Period;
    uint32_t TDuty_Input_Signal;
    uint32_t tdutyInputSignalPred;
    uint32_t tdutyInputSignalPredLinear;
    uint8_t  sensorAngDisplecementMeasured;
    uint32_t TStep;
    uint8_t  nAdv;
    uint8_t  Cutoff_IGN;
    uint8_t  Update_calc;
    uint32_t nOverflow;
    uint8_t  nOverflow_RE;
    uint8_t  nOverflow_FE;
}system_vars;

volatile system_vars scenario = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

typedef struct timerproperty
{
    uint32_t counter;
    uint8_t  timer_program;
}timer_status;

typedef struct pulseManagement
{
    uint8_t engSpeed;
    timer_status timerCtrl[4];
}programSheet;

programSheet request = {0, {{0,0}, {0,0}, {0,0}, {0,0}}};
programSheet Pulse_Program = { 0, {{0,0}, {0,0}, {0,0}, {0,0}}};

typedef struct Scheduler
{
    uint8_t  program;
    uint32_t target_time;
}sched_var;

sched_var array_sched_var[3];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

void systemInitialization(void)
{	
	  copyCalibFlashToRam();
	
    //Identify if there are some data recorded
    if (calibFlashBlock.Calibration_RAM.Max_Engine_Speed == 0u)
    {
				initializeCalibOnRAM();  
        saveCalibRamToFlash();      
    }
	  		
    //Communication
    transmstatus = TRANSMISSION_DONE;
    receptstatus = RECEPTION_DONE;
}

void Turn_OFF_Int_input(void)
{
    HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);
}

void Turn_ON_Int_input(void)
{
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
}

void Set_Ouput_LED(void)
{
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
}

void Set_Ouput_Trigger(uint8_t Value)
{
    if (Value == ON)
    {
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
    }
}

void Set_Ouput_InterruptionTest(void)
{
    HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_6);
}

void Set_Ouput_Inversor(uint8_t Value)
{
    if (Value == ON)
    {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
    }
}

// A iterative binary search function. It returns
// location of x in given array arr[l..r] if present,
// otherwise -1
uint8_t binarySearch(volatile uint16_t array[], uint8_t first, uint8_t last, uint16_t search)
{
    uint8_t middle;

    middle = (first+last)>>1;

    while (first <= last)
    {
        if((search >= array[middle])&&
           (search <= array[middle+1]))
        {
            return middle;
        }
        else if(search > array[middle+1])
        {
            first = middle+1;
        }
        else //search < array[middle]
        {
            last = middle;
        }

        middle = (first + last)>>1;
    }

    return (255u);
}

//This function was prepared to return a 8 bits value, however is saturated  in 64
//Its mandatory in rpm array there are some difference value between two adjacent fields, if do not respect will cause an error return 0xFF
uint8_t linearInterpolation(uint16_t value, volatile uint16_t x_array[], volatile uint8_t y_array[])
{
    uint8_t interp_index;
    uint8_t interp_res;

    //Advance saturation for array min and max
    if(value<x_array[0])
    {
        return(y_array[0]);
    }
    else if(value>x_array[11])
    {
        return(y_array[11]);
    }

    interp_index = binarySearch(x_array, 0, 11, value);

    if(((x_array[interp_index+1]-x_array[interp_index])!=0)&&(interp_index!=255u))
    {
        interp_res = (((y_array[interp_index+1]-y_array[interp_index])*(value-x_array[interp_index]))/(x_array[interp_index+1]-x_array[interp_index]))+y_array[interp_index];
        if(interp_res>64u)
        {
            interp_res = 64u;
        }
        return(interp_res);
    }
    else
    {
        return(255u);
    }
}

void Cut_Igntion(void)
{
    if(scenario.Engine_Speed>calibFlashBlock.Calibration_RAM.Max_Engine_Speed)
    {
        scenario.Cutoff_IGN = ON;
    }
    else
    {
        scenario.Cutoff_IGN = OFF;
    }
}

void Engine_STOP_test(void)
{
    static uint8_t program;
    static uint32_t initial_value;

    if(program == FALSE)
    {
        initial_value = scenario.Rising_Edge_Counter;
        program = TRUE;
    }
    else
    {
        if(scenario.Rising_Edge_Counter == initial_value)
        {
            scenario.Rising_Edge_Counter = 0u;
            scenario.Engine_Speed = 0u;
            program = FALSE;
        }
    }
}

void Timeout(uint32_t period, void (*func)(void), sched_var var[], uint8_t pos, uint8_t *resp_var)
{
    uint32_t counter;

    counter = HAL_GetTick();

    if(var[pos].program == FALSE)
    {
        var[pos].target_time = counter+period;
        var[pos].program = TRUE;
    }

    if(counter>=var[pos].target_time)
    {
        var[pos].program = FALSE;
        *resp_var = TRUE;
    }
    else
    {
        *resp_var = FALSE;
    }
}

uint8_t digitalFilter8bits(uint8_t var, uint8_t k)
{
    static uint8_t varOld = 0u;
    uint8_t varFiltered;

    varFiltered = var + (((varOld-var)*k)/255u);
    varOld = var;

    return(varFiltered);
}

uint16_t digitalFilter16bits(uint16_t var, uint8_t k)
{
    static uint16_t varOld = 0u;
    uint16_t varFiltered;

    varFiltered = var + (((varOld-var)*k)/255u);
    varOld = var;

    return(varFiltered);
}

void Statistics(void)
{
    scenario.engineSpeedFiltered = digitalFilter16bits(scenario.Engine_Speed, 50u);
    scenario.avarageEngineSpeed = (scenario.avarageEngineSpeed+scenario.Engine_Speed)>>1;
    scenario.sensorAngDisplecementMeasured = (scenario.TDuty_Input_Signal*360u)/scenario.Measured_Period;
}

void Task_Fast(void)
{
    //HAL_IWDG_Init(&hiwdg);
}

void Task_Medium(void)
{
    Set_Ouput_LED();
    Cut_Igntion();
    receiveData();

    if(flgTransmition)
    {
        transmitSystemInfo();
    }

    Statistics();
}

void Task_Slow(void)
{
    Engine_STOP_test();
}

void Periodic_task(uint32_t period, void (*func)(void), sched_var var[], uint8_t pos)
{
    volatile uint32_t counter;

    counter = HAL_GetTick();

    if(var[pos].program == FALSE)
    {
        var[pos].target_time = counter+period;
        var[pos].program = TRUE;
    }

    if(counter>=var[pos].target_time)
    {
        var[pos].program = FALSE;
        (*func)();
    }
}

uint32_t predictionCalc(uint32_t period)
{
//http://www.mstarlabs.com/control/engspeed.html
//http://www.megamanual.com/ms2/alphabeta.htm#:~:text=MegaSquirt%2DII%20code%20version%202.83,2nd%20derivative'%20prediction%20options).
    static int32_t errt;
    static int32_t dts;
    static int32_t dtpred;
    static int32_t tddts;
    static int32_t tddtpred;
    static int32_t t2dddts;

    errt = period - dtpred;

    //alpha-beta-gamma filter prediction
    dts = dtpred + ((calibFlashBlock.Calibration_RAM.alpha * errt) / 100u);
    tddts = tddtpred + ((calibFlashBlock.Calibration_RAM.beta * errt) / 100u);
    t2dddts = t2dddts + ((calibFlashBlock.Calibration_RAM.gamma * errt) / 100u);
    dtpred = dts + tddts + (t2dddts >> 1u);
    tddtpred = tddts + t2dddts;

    return(tddtpred);
}

uint8_t Ignition_nTime(uint16_t eng_speed)
{
    static uint8_t advance;

    advance = linearInterpolation(eng_speed, calibFlashBlock.Calibration_RAM.BP_Engine_Speed, calibFlashBlock.Calibration_RAM.BP_Timing_Advance);
    return(advance);
}

void Set_Pulse_Program(void)
{
    static uint32_t Event1, Event2, Event3, Event4;

    Set_Ouput_InterruptionTest();

    scenario.Measured_Period += scenario.nOverflow_RE*TMR2_16bits;

    //Engine speed must be greater than 100rpm and less than 15000rpm to consider Measured_Period useful for calculations
    if((scenario.Measured_Period<=EngineSpeedPeriod_Min)&&
       (scenario.Measured_Period>=EngineSpeedPeriod_Max))
    {
        scenario.Engine_Speed = RPM_const/scenario.Measured_Period;
        scenario.Engine_Speed_old = scenario.Engine_Speed;
        scenario.deltaEngineSpeed = scenario.Engine_Speed-scenario.Engine_Speed_old;

        //Linear prediction
        if((scenario.Engine_Speed<<1u)>scenario.Engine_Speed_old)
        {
            scenario.engineSpeedPred = (scenario.Engine_Speed<<1u)-scenario.Engine_Speed_old;
            scenario.tdutyInputSignalPredLinear = (RPM_const*calibFlashBlock.Calibration_RAM.sensorAngDisplecement)/(scenario.engineSpeedPred*360u);
        }
        else
        {
            scenario.engineSpeedPred = 0u;
        }

        //For calculus purpose I decided to use the linear prediction
        scenario.Engine_Speed = scenario.engineSpeedPred;

        if(scenario.Engine_Speed>=calibFlashBlock.Calibration_RAM.BP_Engine_Speed[0])
        {
            request.engSpeed = HIGH;
            scenario.TDuty_Input_Signal += scenario.nOverflow_FE*TMR2_16bits;
            scenario.tdutyInputSignalPred = predictionCalc(scenario.TDuty_Input_Signal);
            //scenario.TStep = scenario.TDuty_Input_Signal/nSteps;
            scenario.TStep = scenario.tdutyInputSignalPredLinear/nSteps;
            scenario.nAdv = Ignition_nTime(scenario.Engine_Speed);
            Event1 = scenario.TStep*scenario.nAdv;
        }
        else
        {
            //Will be trigger at the same routine that treats the falling edge detection
            request.engSpeed = LOW;
            //This value was set to generate the rising edge pulse immediately
            scenario.nAdv = 0u;
            Event1 = 0u;
        }

        Event2 = Event1+TDutyTriggerK;
        Event3 = Event2+TIntervPulseInv;
        Event4 = Event3+TPulseInv;

        //Event 1 - Generates Rising Edge for trigger signal using TMR4 to generate the event
        request.timerCtrl[0].counter = Event1%65536u;

        //Event 2 - Generates Falling Edge for trigger signal using TMR4 to generate the event
        request.timerCtrl[1].counter = Event2%65536u;

        //Event 3 - Generates Rising Edge for inversor signal using TMR4 to generate the event
        request.timerCtrl[2].counter = Event3%65536u;

        //Event 4 - Generates Falling Edge for inversor signal using TMR4 to generate the event
        request.timerCtrl[3].counter = Event4%65536u;
    }
    else if(scenario.Measured_Period<EngineSpeedPeriod_Max)
    {
        //set flag Overspeed
        //register the max engine speed
    }
    else
    {
        //Underspeed
        //maybe I don´t need to register this because will happening all engine start event
    }

    Pulse_Program.timerCtrl[0].timer_program = EMPTY;
    Pulse_Program.timerCtrl[1].timer_program = EMPTY;
    Pulse_Program.timerCtrl[2].timer_program = EMPTY;
    Pulse_Program.timerCtrl[3].timer_program = EMPTY;

    Set_Ouput_InterruptionTest();
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
   HAL_Init();

  /* USER CODE BEGIN Init */
	systemInitialization();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

    //I don´t know the difference between these two different statment
    //HAL_TIM_Base_Start_IT(&htim2);
    __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);

    //New part regarding UART communication
    //https://deepbluembedded.com/how-to-receive-uart-serial-data-with-stm32-dma-interrupt-polling/
    //__HAL_UART_ENABLE_IT(&huart3, UART_IT_TXE);
    //__HAL_UART_ENABLE_IT(&huart3, UART_IT_TC);
    //__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
    //__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    //HAL_UART_Transmit_IT();
    //HAL_UART_Receive_IT();
    //HAL_UART_IRQHandler();

    //__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    //__HAL_DMA_ENABLE_IT(&hdma_usart3_rx, DMA_IT_TC);
    //hdma_usart3_rx.Instance->CR &
    //HAL_UART_Receive_IT(&huart3, UART3_rxBuffer, 12);

    HAL_UART_Receive_DMA(&huart3, (uint8_t*)UART3_rxBuffer, sizeof(UART3_rxBuffer));


    //HAL_TIM_Base_Start_IT(&htim4);

    //Maybe I don´t need initiate the timers...

    //HAL_TIM_Base_Start_IT(&htim4);
    /* Enable the TIM Capture/Compare 1 interrupt */
    //__HAL_TIM_ENABLE_IT(htim, TIM_IT_CC1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (TRUE)
  {
    //Update the pulse calc scenario
    if (scenario.Update_calc == TRUE)
    {
        Set_Pulse_Program();
        scenario.Update_calc = FALSE;
    }

    //Scheduler
    Periodic_task(  20,&Task_Fast,   array_sched_var, 0);
    Periodic_task( 100,&Task_Medium, array_sched_var, 1);
    Periodic_task(1000,&Task_Slow,   array_sched_var, 2);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 55;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 3;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 55;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA3 PA4 PA5
                           PA7 PA8 PA9 PA10
                           PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 PB13 PB14
                           PB15 PB3 PB4 PB5
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void Pulse_Generator_Scheduler(void)
{
    uint8_t i;

    for(i=0;i<4;i++)
    {
       Pulse_Program.timerCtrl[i].counter = request.timerCtrl[i].counter;
    }

    Set_Ouput_Trigger(OFF);
    Set_Ouput_Inversor(OFF);

    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,Pulse_Program.timerCtrl[0].counter);
    Pulse_Program.timerCtrl[0].timer_program = PROGRAMMED;

    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,Pulse_Program.timerCtrl[1].counter);
    Pulse_Program.timerCtrl[1].timer_program = PROGRAMMED;

    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,Pulse_Program.timerCtrl[2].counter);
    Pulse_Program.timerCtrl[2].timer_program = PROGRAMMED;

    __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,Pulse_Program.timerCtrl[3].counter);
    Pulse_Program.timerCtrl[3].timer_program = PROGRAMMED;

    __HAL_TIM_SET_COUNTER(&htim4,0u);
    HAL_TIM_OC_Start_IT(&htim4,TIM_CHANNEL_1);
    HAL_TIM_OC_Start_IT(&htim4,TIM_CHANNEL_2);
    HAL_TIM_OC_Start_IT(&htim4,TIM_CHANNEL_3);
    HAL_TIM_OC_Start_IT(&htim4,TIM_CHANNEL_4);
}

void TurnOffAllPulseInt(void)
{
    HAL_TIM_OC_Stop_IT(&htim4,TIM_CHANNEL_1);
    HAL_TIM_OC_Stop_IT(&htim4,TIM_CHANNEL_2);
    HAL_TIM_OC_Stop_IT(&htim4,TIM_CHANNEL_3);
    HAL_TIM_OC_Stop_IT(&htim4,TIM_CHANNEL_4);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2)
    {
        scenario.nOverflow++;
    }
}

void Rising_Edge_Event(void)
{
    scenario.Measured_Period = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_1);
    scenario.nOverflow_RE = scenario.nOverflow;
    __HAL_TIM_SET_COUNTER(&htim2,0u);
    scenario.nOverflow = 0u;
    Pulse_Program.engSpeed = request.engSpeed;

    if((Pulse_Program.engSpeed == HIGH)&&
    (scenario.Cutoff_IGN == OFF))
    {
        Pulse_Generator_Scheduler();
    }

    scenario.Rising_Edge_Counter++;
}

void Falling_Edge_Event(void)
{
    scenario.TDuty_Input_Signal = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_2);
    scenario.nOverflow_FE = scenario.nOverflow;

    if((Pulse_Program.engSpeed == LOW)&&
    (scenario.Cutoff_IGN == OFF))
    {
        Pulse_Generator_Scheduler();
    }

    if (scenario.Rising_Edge_Counter>=2u)
    {
        scenario.Update_calc = TRUE;        //set zero after Engine Stop was detected
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if((htim->Instance == TIM2)&&
       (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
    {
        Rising_Edge_Event();
    }

    if((htim->Instance == TIM2)&&
       (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2))
    {
        Falling_Edge_Event();
    }
}

void Treat_Int(uint8_t program)
{
    switch(program)
    {
        case INT_FROM_CH1: Set_Ouput_Trigger(ON);
                           Pulse_Program.timerCtrl[0].timer_program = EMPTY;
                           scenario.triggerEventCounter++;
                           break;

        case INT_FROM_CH2: Set_Ouput_Trigger(OFF);
                           Pulse_Program.timerCtrl[1].timer_program = EMPTY;
                           break;

        case INT_FROM_CH3: Set_Ouput_Inversor(ON);
                           Pulse_Program.timerCtrl[2].timer_program = EMPTY;
                           scenario.inversorEventCounter++;
                           break;

        case INT_FROM_CH4: Set_Ouput_Inversor(OFF);
                           Pulse_Program.timerCtrl[3].timer_program = EMPTY;
                           TurnOffAllPulseInt();
                           break;

        default          : break;
    }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if((htim->Instance == TIM4)&&
       (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
    {
        Treat_Int(INT_FROM_CH1);
    }

    if((htim->Instance == TIM4)&&
       (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2))
    {
        Treat_Int(INT_FROM_CH2);
    }

    if((htim->Instance == TIM4)&&
       (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3))
    {
        Treat_Int(INT_FROM_CH3);
    }

    if((htim->Instance == TIM4)&&
       (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4))
    {
        Treat_Int(INT_FROM_CH4);
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
