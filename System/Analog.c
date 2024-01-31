#ifndef _ADC_FILE
#define _ADC_FILE

#include "Analog.h"
#include "BLDC.h"
#include "Timer_ISR.h"
#include "Timer.h"
#include "GPIO_Init.h"
#include "PI_Cale.h"
#include "User_Config.h"
#define ADC1_DR_Address 0x40012440

ADCSamp ADCSampPare = ADCSamp_DEFAULTS;

float UserRequireSpeed = 0; //�û���Ҫ���ٶ�

float Aver_Current = 0;
float DC_TotalCurrent = 0;
volatile uint16_t RegularConvData_Tab[4] = {0};
uint8_t Flag_Stall = 0;
uint8_t DELTADUTYCYCLE = 1;
uint8_t Flag_Compensata = 0;
uint16_t V_bus_Compensata = 0;
uint16_t Global_Current = 0;
uint16_t U_Bemf = 0, V_Bemf = 0, W_Bemf = 0;
uint16_t motor_a_stall_cnt = 0, motor_b_stall_cnt = 0, motor_c_stall_cnt = 0;
uint16_t Filter_Coff = 0;
uint8_t FILTER_LONG = 3;
uint16_t DelayTime_Star = 2000;
/*****************************************************************************
 �� �� ��  : ADC_Configuration
 ��������  : ADC��ʼ��
 �������  : ��
 �������  : void
*****************************************************************************/
void ADC_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	// ʱ��
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	ADC_ClockModeConfig(ADC1, ADC_ClockMode_SynClkDiv4); // ADCʱ�ӷ�Ƶ  48M/4
	DMA_DeInit(DMA1_Channel1);							 // ����ͨ��1
	ADC_DeInit(ADC1);
	/*****GPIO����*****/
	GPIO_InitStructure.GPIO_Pin = Current_Collect_Pin | Bat_Collect_Pin | Adjust_R_Pin | Inst_Curr_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//	/*****DMA1 Channel1 Config****/
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address; // ADC���ݴ��Դ��ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RegularConvData_Tab; // Ŀ�껺���ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;					  // ��������Դ
	DMA_InitStructure.DMA_BufferSize = 4;								  // ���AD�����ݴ�С,������һ���ɼ�7������
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; // �趨�Ĵ����ڴ��ַ����  ÿ��DMA������Ĵ����е�ֵ�����ڴ�

	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // �������ݿ��  ����
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;			// �ڴ�����ݿ��
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;								// ѭ������
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;							// DMAͨ�����ȼ�
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel1, ENABLE);

	/*****ADC Config****/
	ADC_StructInit(&ADC_InitStructure);					   // ��ʼ��ADC�ṹ
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b; // 12λ����
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	   // �涨ģʽװ�������ڷ�����ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_TRGO;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising; // �����ش���
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;						  // ���ݶ���Ϊ�Ҷ���
	ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;				  // ADC��ɨ�跽��
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_OverrunModeCmd(ADC1, ENABLE);
	/* Convert the ADC1 Channel 0,1,2,4,5,6,7 with 55.5 Cycles as sampling time */

	ADC_ChannelConfig(ADC1, Current_Channel | BatVol_Channel | Adjust_R_Channel | Inst_Curr_Channel, ADC_SampleTime_7_5Cycles); // ADC_SampleTime_7_5Cycles

	//  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC , ENABLE);

	/* ADC Calibration */
	ADC_GetCalibrationFactor(ADC1);

	/* Enable ADCperipheral[PerIdx] */
	ADC_Cmd(ADC1, ENABLE);
	while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN))
		; /* Wait the ADCEN falg */

	/* DMA1 Channel1 enable */
	ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular); // �������

	ADC_DMACmd(ADC1, ENABLE);	 // ADC_DMAʹ��
	ADC_StartOfConversion(ADC1); /* ADC1 regular Software Start Conv */
}
/*****************************************************************************
 �� �� ��  : StorageDataCyclic
 ��������  : ���ݴ洢����
 �������  : ��
 �������  : void
*****************************************************************************/
void StorageDataCyclic(BUFFER_TYPE *buffer, uint16_t data)
{
	*(buffer->array + buffer->bufferIndex) = data;
	buffer->bufferIndex++;
	if (buffer->bufferIndex >= BUFFER_SIZE)
	{
		buffer->bufferIndex = 0;
	}
}
/*****************************************************************************
 �� �� ��  : Filter_AverageCalc
 ��������  : ��������
 �������  : ��
 �������  : void
*****************************************************************************/

uint16_t Filter_AverageCalc(uint16_t *buffer, uint16_t n)
{
	uint16_t Max = 0, Min = 0, j = 0;
	uint16_t Temp = 0;

	Max = Min = Temp = *buffer;

	for (j = 1; j < n; j++)
	{
		if (Max < *(buffer + j))
		{
			Max = *(buffer + j);
		}
		else if (Min > *(buffer + j))
		{
			Min = *(buffer + j);
		}
		else
		{
		}

		Temp += *(buffer + j);
	}

	Temp -= (Max + Min);

	if (n > 2)
	{
		Temp /= (n - 2);
	}

	return Temp;
}
/*****************************************************************************
 �� �� ��  : BemfCheck
 ��������  : �޸�λ�û�ȡ
 �������  : ��
 �������  : void
*****************************************************************************/
void BemfCheck(void)
{
	static uint8_t Sense = 0;

	if (GetComparatorState())
	{
		Sense = 1;
	}
	else
	{
		Sense = 0;
	}
	switch (Motor.Current_Phase)
	{

	case 1: // UV
	{
		if (!Sense)
		{
			CalcSpeedTime();
			MProtect.Stall_Cnt1 = 0;
		}
	}
	break;
	case 2: // UW
	{
		if (Sense)
		{
			CalcSpeedTime();
			MProtect.Stall_Cnt2 = 0;
		}
	}
	break;

	case 3:
	{
		if (!Sense)
		{
			CalcSpeedTime();
			MProtect.Stall_Cnt3 = 0;
		}
	}
	break;

	case 4:
	{
		if (Sense)
		{
			CalcSpeedTime();
			MProtect.Stall_Cnt4 = 0;
		}
	}
	break;

	case 5:
	{
		if (!Sense)
		{
			CalcSpeedTime();
			MProtect.Stall_Cnt5 = 0;
		}
	}
	break;

	case 6:
	{
		if (Sense)
		{
			CalcSpeedTime();
			MProtect.Stall_Cnt6 = 0;
		}
	}
	break;
	default:
	{
		MotorStop();
	}
	break;
	}
}
/*****************************************************************************
 �� �� ��  : JudgeErrorCommutation
 ��������  : �жϻ������
 �������  : ��
 �������  : void
*****************************************************************************/
void JudgeErrorCommutation()
{
	if (Motor.Last_Phase == Motor.Current_Phase)
	{
		if (++SamePhaseCnt >= 3000) // 2000:0.125s
		{
			SamePhaseCnt = 0;
			mcState = mcStop;
			mcFault = Overtime_PhaseChange;
			printf("-----JudgeErrorCommutation!\r\n");
		}
	}
	else
	{
		SamePhaseCnt = 0;
	}

	Motor.Last_Phase = Motor.Current_Phase;
}

/************************************************************************************
 �� �� ��  : Fault_OverUnderVoltage
 ��������  : ��ѹ��Ƿѹ��������
 �������  : ��
 �������  : void
************************************************************************************/
void Fault_OverUnderVoltage(void)
{
	ADCSampPare.BUS_Voltage = RegularConvData_Tab[1]; // DCĸ�ߵ�ѹ
	if ((ADCSampPare.BUS_Voltage < Under_Protect_Voltage) || (ADCSampPare.BUS_Voltage > Over_Protect_Voltage))
	{
		MProtect.OverUnderVoltage_Cnt++;
		if (MProtect.OverUnderVoltage_Cnt >= Under_OverProtectTime)
		{
			MProtect.OverUnderVoltage_Cnt = 0;
			mcState = mcStop;
			mcFault = OverUnderVoltage;
			printf("-----Fault_OverUnderVoltage\r\n");
		}
	}
	else
	{
		MProtect.OverUnderVoltage_Cnt = 0;
	}
}
/************************************************************************************
 �� �� ��  : Motor_Stall
 ��������  : �����ת����
 �������  : ��
 �������  : void
************************************************************************************/
void FaultMotor_Stall(void)
{
	if ((mcState == mcAlignment) || (mcState == mcDrag) || (mcState == mcRun))
	{
		if ((Motor.ActualSpeed > Motor_Stall_Speed) && (Motor.Current > 2000)) // ���϶�ת����
		{
			MProtect.Stall_Cnt1++;
			if (MProtect.Stall_Cnt1 > Motor_Stall_Count)
			{
				MProtect.Stall_Cnt1 = 0;
				mcState = mcStop;
				mcFault = Motor_Stall_1;
				printf("-----MProtect.Stall_Cnt1 > Motor_Stall_Count\r\n");
			}
		}
		else
		{
			MProtect.Stall_Cnt1 = 0;
		}
	}
	if (mcState == mcRun)
	{
		MProtect.Stall_Cnt1++;
		MProtect.Stall_Cnt2++;
		MProtect.Stall_Cnt3++;
		MProtect.Stall_Cnt4++;
		MProtect.Stall_Cnt5++;
		MProtect.Stall_Cnt6++;
		if ((MProtect.Stall_Cnt1 > Motor_Stall_Count) || (MProtect.Stall_Cnt2 > Motor_Stall_Count) || (MProtect.Stall_Cnt3 > Motor_Stall_Count) || (MProtect.Stall_Cnt4 > Motor_Stall_Count) || (MProtect.Stall_Cnt5 > Motor_Stall_Count) || (MProtect.Stall_Cnt6 > Motor_Stall_Count))
		{
			mcState = mcStop;
			mcFault = Motor_Stall_1;
			printf("-----MProtect.Stall_Cnt1 > Motor_Stall_Count222\r\n");
		}
	}
}
/************************************************************************************
 �� �� ��  : Cal_AverCurrent
 ��������  : ����ƽ����������
 �������  : ��
 �������  : void
************************************************************************************/
void Cal_AverCurrent(void)
{
	static uint32_t ADTotalCurrent = 0;
	static int8_t Current_AverTimes = 0;
	uint16_t Aver_Current = 0;
	if (mcState == mcRun)
	{
		Instant_Current_Cale();

		if (Filter_Mode == Filter_Average) // ƽ��ֵ�˲�
		{
			StorageDataCyclic(&ADCSampPare.busCur, Global_Current);
			Motor.Current = Filter_AverageCalc(ADCSampPare.busCur.array, BUFFER_SIZE) * CURRENTFACTOR; //  ����ϵ��  0.82  22.3*0.82
		}
		else if (Filter_Mode == Filter_Function) // �����˲�
		{
			ADTotalCurrent += Global_Current;
			Current_AverTimes++;
			if (Current_AverTimes >= Filter_Const1) // ƽ����������
			{
				Current_AverTimes = 0;
				Aver_Current = CURRENTFACTOR * ADTotalCurrent / Filter_Const1; // MA
				ADTotalCurrent = Global_Current;
				FirstOrder_LPF_Cacl(Aver_Current, Motor.Current, 0.1);
			}
		}
		else
		{
			Motor.Current = 0;
		}
	}
	else
	{
		Motor.Current = 0;
		ADTotalCurrent = 0;
		Global_Current = 0;
	}
}

/************************************************************************************
 �� �� ��  : Cal_AverCurrent
 ��������  : ����ƽ���������� 62.5usִ��һ�� �ж��ﲻ���� ����ʱ��  �����AD ֵ
 �������  : ��
 �������  : void
************************************************************************************/

void Instant_Current_Cale(void)
{
	if ((mcState == mcRun) || (mcState == mcDrag))
	{

		Global_Current = RegularConvData_Tab[0]; //*CURRENT_COFF; ADֵ
	}
}
/************************************************************************************
 �� �� ��  : Fault_Soft_Overcurrent
 ��������  : �������
 �������  : ��
 �������  : void
************************************************************************************/
void Fault_Soft_Overcurrent(void)
{

	if (Motor.Current > OverSoftCurrent) // 35A/0.12=291
	{
		MProtect.OverCurr_Cnt++;
	}
	else
	{
		MProtect.OverCurr_Cnt = 0;
	}
	if (MProtect.OverCurr_Cnt > OverSoftCurrentTime) // 16000*62.5=1s  ���Ƴ�ʱ�䳬��������
	{
		MProtect.OverCurr_Cnt = 0;
		mcState = mcStop;
		mcFault = OverCurrent;
		printf("-----MProtect.OverCurr_Cnt > OverSoftCurrentTime\r\n");
	}
}
/************************************************************************************
 �� �� ��  : Fault_Detection
 ��������  :���ϼ�⺯��
 �������  : ��
 �������  : void
************************************************************************************/
void Fault_Detection(void)
{
	if (OverSoftCurrentProtectEnable == Enable)
	{
		Fault_Soft_Overcurrent();
	}
	if (VoltageProtectEnable == Enable)
	{
		Fault_OverUnderVoltage();
	}
	if (StallProtectEnable == Enable)
	{
		FaultMotor_Stall();
	}
}
#endif
