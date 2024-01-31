
#define _BLDC_FILE

#include "Analog.h"
#include "BLDC.h"
#include "SysConfig.h"
#include "Timer.h"
#include "GPIO_Init.h"
#include "PI_Cale.h"
#include "Timer_ISR.h"
#include "User_Config.h"

uint16_t SamePhaseCnt = 0; // ����������
uint16_t ADCTimeCnt = 0;
uint16_t User_Speed_AD = 0; // ��λ����ADֵ
uint16_t DragTime = 0;
uint8_t pos_check_stage = 0;
uint16_t ADC_check_buf[6];
uint8_t Initial_stage = 0;
uint8_t Flag_adc = 0;
uint8_t Flag_Charger = 0;
uint8_t pos_idx = 0;
uint8_t IPD_Times = 0;
uint8_t Flag_HardOverCurr = 0;
uint16_t ADC_check_buf[6] = {0};
// ������ʼ��

Protect MProtect;
BLDC Motor = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/*****************************************************************************
 �� �� ��  : MotorInit
 ��������  : ��ʼ��
 �������  : ��
 �������  : void
*****************************************************************************/
void MotorInit(void)
{
	Motor.Current_Phase = 0;
	Motor.ActualSpeed = 0;
	flag_SpeedTime = 0;
	SpeedTimeCnt = 0;
	SpeedTime = 0;
	SpeedTimeTemp = 0;
	SpeedTimeSum = 0;
	UserRequireSpeed = 0;
	pos_check_stage = 0;
	Flag_adc = 0;
	pos_idx = 0;
	IPD_Times = 0;
	Sysflags.Run_One = 0;
	MProtect.Stall_Cnt1 = 0;
	MProtect.Stall_Cnt2 = 0;
	MProtect.Stall_Cnt3 = 0;
	MProtect.Stall_Cnt4 = 0;
	MProtect.Stall_Cnt5 = 0;
	MProtect.Stall_Cnt6 = 0;

	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable); // ��ʼ״̬ �Ϲ����0-��ֹ
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);

	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable); // ��ʼ״̬  �¹����1-����
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

	EXTI->IMR &= ~(EXTI_Line4); // ��ֹ�ж�
	EXTI->IMR &= ~(EXTI_Line5); // ��ֹ�ж�
	EXTI->IMR &= ~(EXTI_Line6); // ��ֹ�ж�
	mcState = mcAlignment;
}
/*****************************************************************************
 �� �� ��  : MotorAlignment
 ��������  : ���붨λ
 �������  : ��
 �������  : void
*****************************************************************************/
void MotorAlignment(void)
{
	Motor.Duty = ALIGNMENTDUTY; // ����ռ�ձ�
	if (Motor.StartMode == Alig_Drag)
	{
		Motor.Current_Phase++;
		if (Motor.Current_Phase > 6)
		{
			Motor.Current_Phase = 1;
		}
		if (Motor.Current_Phase < 1)
		{
			Motor.Current_Phase = 6;
		}
	}
	ManulStartup_Turn();	// ǿ�ƻ���
	Delay_ms(ALIGNMENTNMS); // ����ʱ��
	EnterDragInit();		// ���ٳ�ʼ��
}
/*****************************************************************************
 �� �� ��  : EnterDragInit
 ��������  : ����ǿ�ϳ�ʼ��
 �������  : ��
 �������  : void
*****************************************************************************/
void EnterDragInit(void)
{
	ADCTimeCnt = 0;
	DragTime = RAMP_TIM_STA;
	mcState = mcDrag; // ��ʼ�����   �������
}
/*****************************************************************************
 �� �� ��  : EnterRunInit
 ��������  : ����Run�ĳ�ʼ��
 �������  : ��
 �������  : void
*****************************************************************************/
void EnterRunInit(void)
{
	ADCTimeCnt = 0;
	Sysflags.ChangePhase = 0;
	TuneDutyRatioCnt = 0;
	Init_EXT();
	mcState = mcRun;
	TIM_Cmd(TIM3, ENABLE);
	TIM_SetCounter(TIM3, 0); // ���¼���
}
/*****************************************************************************
 �� �� ��  : StartupDrag
 ��������  : ��������
 �������  : ��
 �������  : void
*****************************************************************************/
void StartupDrag(void)
{
	ADCTimeCnt++;
	if (ADCTimeCnt >= DragTime)
	{
		ADCTimeCnt = 0;
		Motor.Duty += RAMP_DUTY_INC;
		DragTime -= (DragTime / RAMP_TIM_STEP) + 1;
		if (DragTime < RAMP_TIM_END)
		{
			DragTime = RAMP_TIM_END;
			EnterRunInit();
			return;
		}
		Motor.Current_Phase++;
		if (Motor.Current_Phase > 6)
		{
			Motor.Current_Phase = 1;
		}
		if (Motor.Current_Phase < 1)
		{
			Motor.Current_Phase = 6;
		}
		ManulStartup_Turn(); // ����
	}
#if 1
	if (Motor.Duty < RAMP_DUTY_STA) // �������ռ�ձȵ������Сֵ
	{
		Motor.Duty = RAMP_DUTY_STA;
	}
	else if (Motor.Duty > RAMP_DUTY_END)
	{
		Motor.Duty = RAMP_DUTY_END;
	}
#endif
}
/*****************************************************************************
 �� �� ��  : Init_EXT
 ��������  : ��ʼ���ж�
 �������  : ��
 �������  : void
*****************************************************************************/
void Init_EXT(void)
{
	switch (Motor.Current_Phase)
	{
	case 1: // UV
	{
		EXTI->IMR &= ~(EXTI_Line5); // ��ֹ�ж�
		EXTI->IMR &= ~(EXTI_Line6); // ��ֹ�ж�
		EXTI->IMR |= (EXTI_Line4);	// ʹ���ж�   W-4,V-5,U-6
	}
	break;
	case 2: // UW
	{

		EXTI->IMR &= ~(EXTI_Line4); // ��ֹ�ж�
		EXTI->IMR &= ~(EXTI_Line6); // ��ֹ�ж�
		EXTI->IMR |= (EXTI_Line5);	// ʹ���ж�   W-4,V-5,U-6
	}
	break;
	case 3: // VW
	{

		EXTI->IMR &= ~(EXTI_Line4); // ��ֹ�ж�
		EXTI->IMR &= ~(EXTI_Line5); // ��ֹ�ж�
		EXTI->IMR |= (EXTI_Line6);	// ʹ���ж�   W-4,V-5,U-6
	}
	break;
	case 4: // VU
	{

		EXTI->IMR &= ~(EXTI_Line5); // ��ֹ�ж�
		EXTI->IMR &= ~(EXTI_Line6); // ��ֹ�ж�
		EXTI->IMR |= (EXTI_Line4);	// ʹ���ж�   W-4,V-5,U-6
	}
	break;
	case 5: // WU
	{

		EXTI->IMR &= ~(EXTI_Line4); // ��ֹ�ж�
		EXTI->IMR &= ~(EXTI_Line6); // ��ֹ�ж�
		EXTI->IMR |= (EXTI_Line5);	// ʹ���ж�   W-4,V-5,U-6
	}
	break;
	case 6: // WV
	{

		EXTI->IMR &= ~(EXTI_Line4); // ��ֹ�ж�
		EXTI->IMR &= ~(EXTI_Line5); // ��ֹ�ж�
		EXTI->IMR |= (EXTI_Line6);	// ʹ���ж�   W-4,V-5,U-6
	}
	break;
	default:
	{
	}
	break;
	}
}
/*****************************************************************************
 �� �� ��  : GetComparatorState
 ��������  : ��ȡ�Ƚ���״̬
 �������  : ��
 �������  : void
*****************************************************************************/
uint8_t GetComparatorState(void)
{
	static uint8_t temp_Level;
	if (Motor.Current_Phase == 1)
	{
		if (NULL_W)
		{
			temp_Level = 1;
		}
		else
		{
			temp_Level = 0;
		}
	}
	else if (Motor.Current_Phase == 2)
	{
		if (NULL_V)
		{
			temp_Level = 1;
		}
		else
		{
			temp_Level = 0;
		}
	}
	else if (Motor.Current_Phase == 3)
	{
		if (NULL_U)
		{
			temp_Level = 1;
		}
		else
		{
			temp_Level = 0;
		}
	}
	else if (Motor.Current_Phase == 4)
	{
		if (NULL_W)
		{
			temp_Level = 1;
		}
		else
		{
			temp_Level = 0;
		}
	}
	else if (Motor.Current_Phase == 5)
	{
		if (NULL_V)
		{
			temp_Level = 1;
		}
		else
		{
			temp_Level = 0;
		}
	}
	else if (Motor.Current_Phase == 6)
	{
		if (NULL_U)
		{
			temp_Level = 1;
		}
		else
		{
			temp_Level = 0;
		}
	}
	else
	{
		;
	}
	return temp_Level;
}
/*****************************************************************************
 �� �� ��  : Startup_Turn
 ��������  : �Զ�����
 �������  : ��
 �������  : void
*****************************************************************************/
void Startup_Turn(void)
{
	switch (Motor.Current_Phase)
	{
	case 1: // UV
	{
		MOS_Q16PWM();				// UW
		EXTI->IMR &= ~(EXTI_Line4); // ��ֹ�ж�
		EXTI->IMR &= ~(EXTI_Line6); // ��ֹ�ж�
		EXTI->IMR |= (EXTI_Line5);	// ʹ���ж�   W-4,V-5,U-6��

		Motor.Current_Phase = 2;
	}
	break;
	case 2:
	{
		MOS_Q26PWM();				// VW
		EXTI->IMR &= ~(EXTI_Line5); // ��ֹ�ж�
		EXTI->IMR &= ~(EXTI_Line4); // ��ֹ�ж�
		EXTI->IMR |= (EXTI_Line6);	// ʹ���ж�   W-4,V-5,U-6

		Motor.Current_Phase = 3;
	}
	break;

	case 3:
	{
		MOS_Q24PWM();				// VU
		EXTI->IMR &= ~(EXTI_Line6); // ��ֹ�ж�
		EXTI->IMR &= ~(EXTI_Line5); // ��ֹ�ж�
		EXTI->IMR |= (EXTI_Line4);	// ʹ���ж�   W-4,V-5,U-6

		Motor.Current_Phase = 4;
	}
	break;

	case 4:
	{
		MOS_Q34PWM();				// WU
		EXTI->IMR &= ~(EXTI_Line4); // ��ֹ�ж�
		EXTI->IMR &= ~(EXTI_Line6); // ��ֹ�ж�
		EXTI->IMR |= (EXTI_Line5);	// ʹ���ж�   W-4,V-5,U-6

		Motor.Current_Phase = 5;
	}
	break;

	case 5:
	{
		MOS_Q35PWM();				// WV
		EXTI->IMR &= ~(EXTI_Line5); // ��ֹ�ж�
		EXTI->IMR &= ~(EXTI_Line4); // ��ֹ�ж�
		EXTI->IMR |= (EXTI_Line6);	// ʹ���ж�   W-4,V-5,U-6

		Motor.Current_Phase = 6;
	}
	break;

	case 6:
	{
		MOS_Q15PWM();				// UV
		EXTI->IMR &= ~(EXTI_Line6); // ��ֹ�ж�
		EXTI->IMR &= ~(EXTI_Line5); // ��ֹ�ж�
		EXTI->IMR |= (EXTI_Line4);	// ʹ���ж�   W-4,V-5,U-6

		Motor.Current_Phase = 1;
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
 �� �� ��  : ManulStartup_Turn
 ��������  : ��Ϊ����
 �������  : ��
 �������  : void
*****************************************************************************/
void ManulStartup_Turn(void)
{

	switch (Motor.Current_Phase)
	{

	case 1:
	{
		MOS_Q15PWM(); // UV
	}
	break;

	case 2:
	{
		MOS_Q16PWM(); // UW
	}
	break;

	case 3:
	{
		MOS_Q26PWM(); // VW
	}
	break;

	case 4:
	{
		MOS_Q24PWM(); // VU
	}
	break;

	case 5:
	{
		MOS_Q34PWM(); // WU
	}
	break;

	case 6:
	{
		MOS_Q35PWM(); // WV
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
 �� �� ��  : Charger
 ��������  : ���
 �������  : ��
 �������  : void
*****************************************************************************/
void Charger(void)
{
	TIM1->CCR1 = 0; // ������
	TIM1->CCR2 = 0; //
	TIM1->CCR3 = 0; //
	// U
	TIM_OC2PolarityConfig(TIM1, TIM_OCPolarity_High);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable); // �Ϲܽ�ֹ  ��� �͵�ƽ
	TIM_OC2NPolarityConfig(TIM1, TIM_OCNPolarity_Low);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable); // �¹ܺ㵼ͨ ����͵�ƽ
	// V
	TIM_OC2PolarityConfig(TIM1, TIM_OCPolarity_High);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable); // �Ϲܽ�ֹ  ��� �͵�ƽ
	TIM_OC2NPolarityConfig(TIM1, TIM_OCNPolarity_Low);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable); // �¹ܺ㵼ͨ ����͵�ƽ
	// V
	TIM_OC2PolarityConfig(TIM1, TIM_OCPolarity_High);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable); // �Ϲܽ�ֹ  ��� �͵�ƽ
	TIM_OC2NPolarityConfig(TIM1, TIM_OCNPolarity_Low);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable); // �¹ܺ㵼ͨ ����͵�ƽ

	Flag_Charger = 1; // ����־λ
}
/*****************************************************************************
 �� �� ��  : All_Discharg
 ��������  : ȫ���ر�
 �������  : ��
 �������  : void
*****************************************************************************/
void All_Discharg(void)
{
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	// U
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);	// �Ϲ���� 0��ֹ
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable); //  �¹����1-����
														// V
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);	// �Ϲ���� 0��ֹ
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable); //  �¹����1-����
														// W
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);	// �Ϲ���� 0��ֹ
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable); //  �¹����1-����
}
/*****************************************************************************
 �� �� ��  : UV_W_phase_inject
 ��������  : UV_Wע��
 �������  : ��
 �������  : void
*****************************************************************************/
void UV_W_phase_inject(void)
{

	TIM1->CCR1 = 0;
	TIM1->CCR3 = Lock_Duty; //	 U��
	TIM1->CCR2 = Lock_Duty; //    V��

	// U
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);   // �Ϲ����pwm
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable); // �¹�ͬ����� -������ʱ��
	// V
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);   // �Ϲ����pwm
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable); // �¹�ͬ����� -������ʱ��

	// W
	TIM_OC1PolarityConfig(TIM1, TIM_OCPolarity_High);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable); // �Ϲܽ�ֹ  ��� �͵�ƽ
	TIM_OC1NPolarityConfig(TIM1, TIM_OCNPolarity_Low);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable); // �¹ܺ㵼ͨ ����͵�ƽ

	Flag_adc = 1;
	pos_idx = 0;
}
/*****************************************************************************
 �� �� ��  : W_UV_phase_inject
 ��������  : W_UVע��
 �������  : ��
 �������  : void
*****************************************************************************/
void W_UV_phase_inject(void)
{

	TIM1->CCR3 = 0;			//	 U��
	TIM1->CCR2 = 0;			//    V��
	TIM1->CCR1 = Lock_Duty; // W��
	// W
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);   // �Ϲ����pwm
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable); // �¹�ͬ����� -������ʱ��
	// U
	TIM_OC1PolarityConfig(TIM1, TIM_OCPolarity_High);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable); // �Ϲܽ�ֹ  ��� �͵�ƽ
	TIM_OC1NPolarityConfig(TIM1, TIM_OCNPolarity_Low);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable); // �¹ܺ㵼ͨ ����͵�ƽ
	// V
	TIM_OC1PolarityConfig(TIM1, TIM_OCPolarity_High);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable); // �Ϲܽ�ֹ  ��� �͵�ƽ
	TIM_OC1NPolarityConfig(TIM1, TIM_OCNPolarity_Low);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable); // �¹ܺ㵼ͨ ����͵�ƽ

	Flag_adc = 1;
	pos_idx = 1;
}
/*****************************************************************************
 �� �� ��  : WU_V_phase_inject
 ��������  : WU  Vע��
 �������  : ��
 �������  : void
*****************************************************************************/
void WU_V_phase_inject(void)
{

	TIM1->CCR2 = 0; // W��

	TIM1->CCR1 = Lock_Duty; // W��
	TIM1->CCR3 = Lock_Duty; //	U��
	// W
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);   // �Ϲ����pwm
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable); // �¹�ͬ����� -������ʱ��
	// U
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);   // �Ϲ����pwm
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable); // �¹�ͬ����� -������ʱ��
													   // V
	TIM_OC1PolarityConfig(TIM1, TIM_OCPolarity_High);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable); // �Ϲܽ�ֹ  ��� �͵�ƽ
	TIM_OC1NPolarityConfig(TIM1, TIM_OCNPolarity_Low);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable); // �¹ܺ㵼ͨ ����͵�ƽ
	Flag_adc = 1;
	pos_idx = 2;
}
/*****************************************************************************
 �� �� ��  : V_WU_phase_inject
 ��������  : V_WU ע��
 �������  : ��
 �������  : void
*****************************************************************************/
void V_WU_phase_inject(void)
{
	TIM1->CCR1 = 0;			// W��
	TIM1->CCR3 = 0;			//	U��
	TIM1->CCR2 = Lock_Duty; //    V��

	// V
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);   // �Ϲ����pwm
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable); // �¹�ͬ����� -������ʱ��
	// W
	TIM_OC1PolarityConfig(TIM1, TIM_OCPolarity_High);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable); // �Ϲܽ�ֹ  ��� �͵�ƽ
	TIM_OC1NPolarityConfig(TIM1, TIM_OCNPolarity_Low);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable); // �¹ܺ㵼ͨ ����͵�ƽ
	// U
	TIM_OC1PolarityConfig(TIM1, TIM_OCPolarity_High);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable); // �Ϲܽ�ֹ  ��� �͵�ƽ
	TIM_OC1NPolarityConfig(TIM1, TIM_OCNPolarity_Low);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable); // �¹ܺ㵼ͨ ����͵�ƽ

	Flag_adc = 1;
	pos_idx = 3;
}
/*****************************************************************************
 �� �� ��  : VW_U_phase_inject
 ��������  : VW_U ע��
 �������  : ��
 �������  : void
*****************************************************************************/
void VW_U_phase_inject(void)
{

	TIM1->CCR3 = 0;
	TIM1->CCR1 = Lock_Duty; // W��
	TIM1->CCR2 = Lock_Duty; //	V��
	// V
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);   // �Ϲ����pwm
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable); // �¹�ͬ����� -������ʱ��
	// W
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);   // �Ϲ����pwm
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable); // �¹�ͬ����� -������ʱ��
	// U
	TIM_OC1PolarityConfig(TIM1, TIM_OCPolarity_High);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable); // �Ϲܽ�ֹ  ��� �͵�ƽ
	TIM_OC1NPolarityConfig(TIM1, TIM_OCNPolarity_Low);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable); // �¹ܺ㵼ͨ ����͵�ƽ
	Flag_adc = 1;
	pos_idx = 4;
}
/*****************************************************************************
 �� �� ��  : U_VW_phase_inject
 ��������  : U_VW ע��
 �������  : ��
 �������  : void
*****************************************************************************/
void U_VW_phase_inject(void)
{

	TIM1->CCR1 = 0;									   // W��
	TIM1->CCR2 = 0;									   //	V��
	TIM1->CCR3 = Lock_Duty;							   //	U��
													   // U
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);   // �Ϲ����pwm
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Enable); // �¹�ͬ����� -������ʱ��
	// V
	TIM_OC1PolarityConfig(TIM1, TIM_OCPolarity_High);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable); // �Ϲܽ�ֹ  ��� �͵�ƽ
	TIM_OC1NPolarityConfig(TIM1, TIM_OCNPolarity_Low);
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Enable); // �¹ܺ㵼ͨ ����͵�ƽ
	// W
	TIM_OC1PolarityConfig(TIM1, TIM_OCPolarity_High);
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable); // �Ϲܽ�ֹ  ��� �͵�ƽ
	TIM_OC1NPolarityConfig(TIM1, TIM_OCNPolarity_Low);
	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Enable); // �¹ܺ㵼ͨ ����͵�ƽ
	Flag_adc = 1;
	pos_idx = 5;
}

/*****************************************************************************
 �� �� ��  : Align_pos_check_proc
 ��������  : ��ʼλ�ü��
 �������  : ��
 �������  : void
*****************************************************************************/
void Align_pos_check_proc(void)
{
	if ((Flag_adc == 0) && (Flag_Charger == 0))
	{
		switch (pos_check_stage)
		{
		case 0:
			Charger();
			pos_check_stage = 10;
			break;
		case 10:
			UV_W_phase_inject();
			pos_check_stage = 1;
			break;

		case 1:
			Charger();
			pos_check_stage = 20;
			break;
		case 20:
			W_UV_phase_inject();
			pos_check_stage = 3;
			break;

		case 3:
			Charger();
			pos_check_stage = 30;
			break;
		case 30:
			WU_V_phase_inject();
			pos_check_stage = 4;
			break;
		case 4:
			Charger();
			pos_check_stage = 40;
			break;

		case 40:
			V_WU_phase_inject();
			pos_check_stage = 5;
			break;
		case 5:
			Charger();
			pos_check_stage = 50;
			break;
		case 50:
			VW_U_phase_inject();
			pos_check_stage = 6;
			break;
		case 6:
			Charger();
			pos_check_stage = 60;
			break;
		case 60:
			U_VW_phase_inject();
			pos_check_stage = 7;
			break;
		case 7:
			Motor.Current_Phase = 0;
			if (ADC_check_buf[0] <= ADC_check_buf[1])
				Motor.Current_Phase |= 0x04;
			if (ADC_check_buf[2] <= ADC_check_buf[3])
				Motor.Current_Phase |= 0x02;
			if (ADC_check_buf[4] <= ADC_check_buf[5])
				Motor.Current_Phase |= 0x01;
			Initial_stage = Motor.Current_Phase;
			Flag_HardOverCurr = 1;
#if 1 // ֻ����λ��  ��
			All_Discharg();

#endif
#if 1 // ֻ����λ��   �ر�
			switch (Initial_stage)
			{

			case 0x05: // ��ǰ��5
			{
				Motor.Current_Phase = 1; // ����һ��2
				MotorAlignment();
			}
			break;

			case 0x01:
			{
				Motor.Current_Phase = 2;
				MotorAlignment();
			}
			break;

			case 0x03:
			{
				Motor.Current_Phase = 3;
				MotorAlignment();
			}
			break;
			case 0x02:
			{
				Motor.Current_Phase = 4;
				MotorAlignment();
			}
			break;

			case 0x06:
			{
				Motor.Current_Phase = 5;
				MotorAlignment();
			}
			break;

			case 0x04:
			{
				Motor.Current_Phase = 6;
				MotorAlignment();
			}
			break;
			default:
			{
				// ���´�һ��
				All_Discharg();
			}
			break;
			}
#endif
			pos_check_stage = 70;
			if ((Initial_stage == 0) || (Initial_stage == 7))
			{

				IPD_Times++;
				if (IPD_Times >= 3)
				{
					IPD_Times = 3;
					mcState = mcStop;
					All_Discharg();
					printf("-----(Initial_stage == 0) || (Initial_stage == 7)\r\n");
				}
				else
				{
					pos_check_stage = 0;
					Flag_adc = 0;
					pos_idx = 0;
					Flag_Charger = 0;
					mcState = mcAlignment;
					Flag_HardOverCurr = 0;
				}
			}
			break;
		default:
			break;
		}
	}
}

/*****************************************************************************
 �� �� ��  : MotorControl
 ��������  : �������
 �������  : ��
 �������  : void
*****************************************************************************/
void MotorControl(void) // 500us
{
	switch (mcState)
	{
	case mcReady:
		if ((Sysflags.Systerm_On == 1) && (mcFault == RunNormal))
		{
			mcState = mcInit;
			TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); // ��ֹ�ж�
		}
		break;
	case mcInit: // ��ʼ��
		if (Sysflags.Systerm_On == 1)
			MotorInit();
#ifdef CLOSED_SPEEDLOOP_Halless // �ջ�����
		PIDParament_Init();
#endif
		break;
	case mcCharge:
		break;
	case mcAlignment: // ��λ
		if (Motor.StartMode == Alig_Drag)
		{
			MotorAlignment();
			Flag_HardOverCurr = 1;
		}
		if (Sysflags.Systerm_On == 0)
		{
			mcState = mcStop;
			MotorStop();
			printf("-----mcAlignment_fail!\r\n");
		}
		break;
	case mcDrag: // ǿ��	`
		if (Sysflags.Systerm_On == 0)
		{
			mcState = mcStop;
			MotorStop();
			printf("-----mcDrag_fail!\r\n");
		}
		break;
	case mcRun: // ADC���綯��
		SpeedController();
		if (Sysflags.Run_One == 0)
		{
			Sysflags.Run_One = 1;
			TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE); // ��ֹ�ж�
		}

		if (Sysflags.Systerm_On == 0)
		{
			mcState = mcStop;
			MotorStop();
			printf("-----Sysflags.Systerm_On == 0\r\n");
		}
		break;
	case mcStop: // ���ֹͣ�������ϵ�
		MotorStop();
		if (((mcFault != RunNormal) && (Sysflags.Systerm_On == 0)) || (mcFault == RunNormal))
		{
			mcState = mcBrake;
		}
		break;
	case mcBrake:
		if (Sysflags.Systerm_On == 1)
		{
			mcState = mcReady;
			mcFault = RunNormal;
		}
		break;

	default:
		break;
	}
}

/*****************************************************************************
 �� �� ��  : MotorStop
 ��������  : ���ͣ��
 �������  : ��
 �������  : void
*****************************************************************************/
void MotorStop(void)
{
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable); // ��ʼ״̬ �Ϲ����0-��ֹ
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);

	TIM_CCxNCmd(TIM1, TIM_Channel_1, TIM_CCxN_Disable); // ��ʼ״̬  �¹����1-����
	TIM_CCxNCmd(TIM1, TIM_Channel_2, TIM_CCxN_Disable);
	TIM_CCxNCmd(TIM1, TIM_Channel_3, TIM_CCxN_Disable);

	EXTI_ClearITPendingBit(EXTI_Line4);
	EXTI_ClearITPendingBit(EXTI_Line5);
	EXTI_ClearITPendingBit(EXTI_Line6);

	//	TIM_SetCompare1(TIM1, 0);
	//	TIM_SetCompare2(TIM1, 0);
	//	TIM_SetCompare3(TIM1, 0);
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;

	TIM_Cmd(TIM14, DISABLE);  // ʧ�ܶ�ʱ��
	TIM_Cmd(TIM3, DISABLE);	  // ʧ�ܶ�ʱ��
	TIM_SetCounter(TIM3, 0);  // ���¼���
	TIM_SetCounter(TIM14, 0); // ���¼���

	Flag_HardOverCurr = 0;
	Motor.Duty = 0;
	Motor.Current_Phase = 0;
	Motor.Current = 0;
	Motor.ActualSpeed = 0;
	TuneDutyRatioCnt = 0;
	Sysflags.ChangePhase = 0;
	Sysflags.Angle_Mask = 0;
	// LIN �������������  ��ֹͣʱ Ӧ�����������ƽΪ�� �������Ϊ��--ɲ��
	if (STOPMODE == BRAKEDOWN)
	{
	}
	else if (STOPMODE == FREEDOWN)
	{
	}

	EXTI->IMR &= ~(EXTI_Line4); // ��ֹ�ж�
	EXTI->IMR &= ~(EXTI_Line5); // ��ֹ�ж�
	EXTI->IMR &= ~(EXTI_Line6); // ��ֹ�ж�
}

float KQ = 0.01; // ������������
float KR = 0.1;	 // ������������
float Get_KRM_ADJ_Actual(float InputValue)
{
	static float ADJ_Kx_Out = 0; // ���Ƶ�״ֵ̬
	static float ADJ_KP = 1;	 // ��������Э����
	// �������˲�����Ԥ�ⲽ��
	// ���¹�������Э����
	ADJ_KP = ADJ_KP + KQ;
	// ���㿨��������
	float ADJ_KK = ADJ_KP / (ADJ_KP + KR);
	// ##########################################
	// ����״̬����
	float adj_value = InputValue - ADJ_Kx_Out;
	ADJ_Kx_Out = ADJ_Kx_Out + ADJ_KK * adj_value;
	// ���¹�������Э����
	ADJ_KP = (1 - ADJ_KK) * ADJ_KP;
	// ##########################################
	// #ifdef Debug_EN
	// 	static unsigned char BB_Cnt;
	// 	BB_Cnt++;
	// 	if (BB_Cnt >= 2)
	// 	{
	// 		BB_Cnt = 0; // 20*10ms
	// 		UART_SendString("->ADJ_Kx_Out:");
	// 		print_number(ADJ_Kx_Out);
	// 	}
	// #endif
	// ##########################################

	return ADJ_Kx_Out;
}

/*****************************************************************************
 �� �� ��  : UserSpeedControl
 ��������  : ���ٿ���
 �������  : ��
 �������  : void
*****************************************************************************/
void UserSpeedControl(void) // 500usִ��һ��
{
	static uint8_t AD_Cnt = 0;		 // AD�������ֲ�����
	static uint8_t RheostatCnt0 = 0; // ���پֲ�����
	static uint8_t RheostatCnt1 = 0; // ���پֲ�����
	static float Temp_UserRequireSpeed = 0;

#ifdef CLOSED_SPEEDLOOP_Halless // �ջ�����

	// if (Motor.ControlMode == CLOSED_SPEEDLOOP_Halless)
	{
		if (ADJ_MODE == STEPLESS_SPEED)
		{
			AD_Cnt++;
			if (AD_Cnt < 200) // 500us*100=100 000us=100ms
			{
				AD_Cnt = 0;
				User_Speed_AD = RegularConvData_Tab[2]; // ��ȡ��λ��ADֵ0-4095
			}

			if (User_Speed_AD < 50) // RHEOSTATMIN
			{
				RheostatCnt0++;
				RheostatCnt1 = 0;
				if (RheostatCnt0 >= 20)
				{
					RheostatCnt0 = 0;
					Sysflags.Systerm_On = 0;
				}
			}
			else
			{
				RheostatCnt1++;
				RheostatCnt0 = 0;
				if (RheostatCnt1 >= 20)
				{
					RheostatCnt1 = 0;
					Sysflags.Systerm_On = 1;
				}
			}

			if (Sysflags.Systerm_On == 1)
			{
				FirstOrder_LPF_Cacl((User_Speed_AD * 5), Temp_UserRequireSpeed, 0.001);
				// 6000~22000
				UserRequireSpeed = Get_KRM_ADJ_Actual(Temp_UserRequireSpeed);
			}
			else
			{
				UserRequireSpeed = 0;
			}
		}
		if (ADJ_MODE == DIRECT_GIVE)
		{
			if (GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_1) == 0)
			{
				RheostatCnt0++;
				if (RheostatCnt0 > 100)
				{
					RheostatCnt0 = 0;
					RheostatCnt1 = 0;
					Sysflags.Systerm_On = 1;
				}
			}
			else
			{
				RheostatCnt1++;
				if (RheostatCnt1 > 100)
				{
					RheostatCnt1 = 0;
					RheostatCnt0 = 0;
					Sysflags.Systerm_On = 0;
				}
			}
			
			if (Sysflags.Systerm_On == 1)
				UserRequireSpeed = Motor_UserSpeed;
			else
				UserRequireSpeed = 0;
		}
	}
#endif

#ifdef OPEN_LOOP_Halless // ��������
	// else if (Motor.ControlMode == OPEN_LOOP_Halless)
	{
		if (ADJ_MODE == STEPLESS_SPEED)
		{
			AD_Cnt++;
			if (AD_Cnt < 200) // 500us*100=100 000us=100ms
			{
				AD_Cnt = 0;
				User_Speed_AD = RegularConvData_Tab[2]; // ��ȡ��λ��ADֵ0-4095
			}
			if (User_Speed_AD < 50)
			{
				RheostatCnt0++;
				RheostatCnt1 = 0;
				if (RheostatCnt0 >= 20)
				{
					RheostatCnt0 = 0;
					Sysflags.Systerm_On = 0;
				}
			}
			else
			{
				RheostatCnt1++;
				RheostatCnt0 = 0;
				if (RheostatCnt1 >= 20)
				{
					RheostatCnt1 = 0;
					Sysflags.Systerm_On = 1;
				}
			}
			if (Sysflags.Systerm_On == 1)
			{
				// UserRequireSpeed = MIN_DUTY + User_Speed_AD * 0.5;
				// FirstOrder_LPF_Cacl((User_Speed_AD * 5), Temp_UserRequireSpeed, 0.001);
				// 4096 / 1500 = 2.73
				FirstOrder_LPF_Cacl((User_Speed_AD / 2.73), Temp_UserRequireSpeed, 0.001);
				// 6000~22000
				UserRequireSpeed = Get_KRM_ADJ_Actual(Temp_UserRequireSpeed);
			}
			else
			{
				UserRequireSpeed = 0;
			}
		}
		if (ADJ_MODE == DIRECT_GIVE)
		{
			if (GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_1) == 0)
			{
				RheostatCnt0++;
				if (RheostatCnt0 > 100)
				{
					RheostatCnt0 = 0;
					RheostatCnt1 = 0;
					Sysflags.Systerm_On = 1;
				}
			}
			else
			{
				RheostatCnt1++;
				if (RheostatCnt1 > 100)
				{
					RheostatCnt1 = 0;
					RheostatCnt0 = 0;
					Sysflags.Systerm_On = 0;
				}
			}
			if (Sysflags.Systerm_On == 1)
				UserRequireSpeed = Motor_UserSpeed;
			else
				UserRequireSpeed = 0;
		}
	}
#endif
	// if (Debug_cnt == 0) // 500us*500 = 250ms
	// {
	// 	Debug_cnt = 500;
	// 	printf("Temp_UserRequireSpeed: %f->UserRequireSpeed: %f->\r\n",Temp_UserRequireSpeed, UserRequireSpeed);
	// }
}
