#include "dspin.h"

dSPIN_RegsStruct_TypeDef dSPIN_RegsStruct;

unsigned char CS = 0x00;
unsigned char FLAG = 0x03;
unsigned char BUSY = 0x03;

/********************************************************************
 * 函数说明：初始化STM32外设，GPIO,时钟
 * 输    入：无
 * 返    回：无
 ******************************************************************/
void dSPIN_Peripherals_Init(void) {
	// 在图形界面里面配置你的引脚
	// 包括 SPI 3Pins, SS, BUSY, FLAG
	// 后两个是上拉输入
}

/********************************************************************
 * 函数说明：初始化L6470各寄存器初值
 * 输    入：无
 * 返    回：无
 ******************************************************************/
void dSPIN_Regs_Struct_Reset(dSPIN_RegsStruct_TypeDef* dSPIN_RegsStruct) {
	dSPIN_RegsStruct->ABS_POS = 0;		//该ABC_POS寄存器包含当前电机的绝对位置与选定选定的步模式
	dSPIN_RegsStruct->EL_POS = 0;		//EL_POS寄存器的包含电机的直流电位置。
	dSPIN_RegsStruct->MARK = 0;	//MARK寄存器包含称为MARK的绝对位置，根据所选择的步模式，选择单步模式（全，半，季度等）的存储值的单位是平等的。
	dSPIN_RegsStruct->SPEED = 0;//速度寄存器包含当前电机的转速，0-15625步/秒，参照dSPIN_Run（方向，速度）	命令
	dSPIN_RegsStruct->ACC = 0x08A;		//加速率，范围14.55至59590 steps/s2
	dSPIN_RegsStruct->DEC = 0x08A;		//减速率，范围14.55至59590 steps/s2
	dSPIN_RegsStruct->MAX_SPEED = 0x041;		//最大速度，取值范围为15.25至15610步/秒
	dSPIN_RegsStruct->MIN_SPEED = 0;	//最小速度，取值范围为    0至976.3步/秒
	dSPIN_RegsStruct->FS_SPD = 0x027;	//全步进切换阀值，取值范围为7.63到15625步/秒
	dSPIN_RegsStruct->KVAL_HOLD = 0x29;	//保持时的占空比（转矩）设定为10％，范围在0到99.6％
	dSPIN_RegsStruct->KVAL_RUN = 0x29;	//运行时的占空比（转矩）设定为10％，范围在0到99.6％
	dSPIN_RegsStruct->KVAL_ACC = 0x29;	//加速时的占空比（转矩）设定为10％，范围在0到99.6％
	dSPIN_RegsStruct->KVAL_DEC = 0x29;	//减速时的占空比（转矩）设定为10％，范围在0到99.6％ 
	dSPIN_RegsStruct->INT_SPD = 0x0408;	//加速/减速(反电动势)BEMF补偿曲线斜率的速度值。范围在0到3906 步/秒
	dSPIN_RegsStruct->ST_SLP = 0x19;	//加减速——开始斜率（反电动势）BEMF补偿设置 范围在0到0.4% s/step 
	dSPIN_RegsStruct->FN_SLP_ACC = 0x29;//加速度——结束斜率（反电动势）BEMF补偿设置 范围在0到0.4% s/step 
	dSPIN_RegsStruct->FN_SLP_DEC = 0x29;//减速度——结束斜率（反电动势）BEMF补偿设置 范围在0到0.4% s/step 
	dSPIN_RegsStruct->K_THERM = 0;		//热补偿参数设置为1，范围为1-1.46875
	dSPIN_RegsStruct->OCD_TH = 0x8;		//过流阈值设置1500毫安 
	dSPIN_RegsStruct->STALL_TH = 0x6;	//失速阈值设置至1000mA，范围：31.25 to 4000mA  //31.25 *6
	dSPIN_RegsStruct->STEP_MODE = 0x7;	//设置到128微步模式 1-7
	dSPIN_RegsStruct->ALARM_EN = 0x00;	//报警全能 
	//7：错误命令
	//6：开关打开事件
	//5：B桥失速
	//4：A桥失速
	//3：欠压
	//2：热警告
	//1：热关机
	//0：过流
	dSPIN_RegsStruct->CONFIG = 0x2E88;//状态寄存器  2E88		  	   001	       011		10	  1 	0	 0  	0	 1000
	//							15/14/13	12/11/10	9/8	  7 	6	 5  	4	 3/2/1/0
	//15/14/13：PWM分频				2分频
	//12/11/10：PWM倍频				1倍
	//9/8：		压摆率				290V		00：180V	01：180V	10：290V	11：530V
	//7：		过流关电源桥		关闭		0：不关闭电源桥	1：关闭电源桥
	//6
	//5：		电机电压补偿		无效		0：无效	1：有效
	//4：		外部开关硬停止		硬停止		0：硬停止	1：用户处理
	//3/2/1/0：	L6470时钟源设置		内部16MHz振荡器，2MHz的的OSCOUT时钟
}
/**********************************************************************
 * 名    称：L6470_Configuration1
 * 功    能：配置1号电机各参数
 * 入口参数：
 * 出口参数：
 * 说    明：步进电机初始化设置
 ***********************************************************************/
void L6470_Configuration1(void) {
	/* 初始化L6470各寄存器初值 */
	dSPIN_Regs_Struct_Reset(&dSPIN_RegsStruct);
	/* 加速率的设置为1000 steps/s2，范围14.55至59590 steps/s2*/
	dSPIN_RegsStruct.ACC = AccDec_Steps_to_Par(1000);										//466);
	/* 减速率的设置为1000 steps/s2，范围14.55至59590 steps/s2 */
	dSPIN_RegsStruct.DEC = AccDec_Steps_to_Par(1000);										//466);
	/* 最大速度设置为3000步/秒，最大速度设置范围为15.25至15610步/秒*/
	dSPIN_RegsStruct.MAX_SPEED = MaxSpd_Steps_to_Par(3000);
	/* 最小速度设置为0步/秒，取值范围为0至976.3，步骤/秒*/
	dSPIN_RegsStruct.MIN_SPEED = MinSpd_Steps_to_Par(0);
	/* 全步进速度设置2500步/秒，范围为7.63到15625步/秒*/
	dSPIN_RegsStruct.FS_SPD = FSSpd_Steps_to_Par(2500);										//252);
	/* 这里要注意不同型号的电机需要的占空比很不一样，需要调整下面的四个数字 */
	/*保持占空比（转矩）设定为10％，范围在0到99.6％*/
	dSPIN_RegsStruct.KVAL_HOLD = Kval_Perc_to_Par(10);
	/* 运行占空比（转矩）设定为10％，范围在0到99.6％*/
	dSPIN_RegsStruct.KVAL_RUN = Kval_Perc_to_Par(10);
	/* 加速的占空比（转矩）设定为15％，范围在0到99.6％*/
	dSPIN_RegsStruct.KVAL_ACC = Kval_Perc_to_Par(15);
	/* 减速的占空比（转矩）设定为15％，范围在0到99.6％ */
	dSPIN_RegsStruct.KVAL_DEC = Kval_Perc_to_Par(15);
	/* 加速/减速曲线斜率的速度值。 range 0 to 3906 steps/s */
	dSPIN_RegsStruct.INT_SPD = IntSpd_Steps_to_Par(500);									//200);
	/* 加减速——开始斜率(反电动势)BEMF补偿设置 0 to 0.4% s/step */
	dSPIN_RegsStruct.ST_SLP = BEMF_Slope_Perc_to_Par(0.038);
	/* 加速度——结束斜率（反电动势）BEMF补偿设置 0 to 0.4% s/step */
	dSPIN_RegsStruct.FN_SLP_ACC = BEMF_Slope_Perc_to_Par(0.063);
	/* 减速度——结束斜率（反电动势）BEMF补偿设置 0 to 0.4% s/step */
	dSPIN_RegsStruct.FN_SLP_DEC = BEMF_Slope_Perc_to_Par(0.063);
	/* 热补偿参数设置为1，范围为1-1.46875*/
	dSPIN_RegsStruct.K_THERM = KTherm_to_Par(1);
	/* 过流阈值设置1500毫安 */
	dSPIN_RegsStruct.OCD_TH = dSPIN_OCD_TH_2250mA; //375mA//750mA//1125mA//1500mA//1875mA//2250mA//2625mA//3000mA//3375mA//3750mA//4125mA//4500mA//4875mA//5250mA//5625mA//6000mA
	/* 失速阈值设置至1000mA，范围：31.25 to 4000mA */
	dSPIN_RegsStruct.STALL_TH = StallTh_to_Par(1000); //1000
	/* 设置到128微步模式 */
	dSPIN_RegsStruct.STEP_MODE = dSPIN_STEP_SEL_1_128;
	/* 报警设置 - 启用所有警报 */
	dSPIN_RegsStruct.ALARM_EN = dSPIN_ALARM_EN_ALL_ENABLE;
	//dSPIN_ALARM_EN_OVERCURRENT
	//|dSPIN_ALARM_EN_THERMAL_SHUTDOWN
	//|dSPIN_ALARM_EN_THERMAL_WARNING
	//|dSPIN_ALARM_EN_UNDER_VOLTAGE
	//|dSPIN_ALARM_EN_STALL_DET_A
	//|dSPIN_ALARM_EN_STALL_DET_B;
	//|dSPIN_ALARM_EN_SW_TURN_ON
	//|dSPIN_ALARM_EN_WRONG_NPERF_CMD;
	/* 内部振荡器，2MHz的的OSCOUT时钟，电源电压补偿禁用，启用过流关断，压摆率= 290 V /us，PWM频率为15.6kHz *///ENABLE// //过流关断
	dSPIN_RegsStruct.CONFIG = dSPIN_CONFIG_INT_16MHZ_OSCOUT_2MHZ
			| dSPIN_CONFIG_SW_HARD_STOP | dSPIN_CONFIG_VS_COMP_DISABLE
			| dSPIN_CONFIG_OC_SD_DISABLE | dSPIN_CONFIG_SR_290V_us
			| dSPIN_CONFIG_PWM_DIV_2 | dSPIN_CONFIG_PWM_MUL_1;
	// Program all dSPIN registers 
	dSPIN_Registers_Set(&dSPIN_RegsStruct);
}
/********************************************************************
 * 函数说明：1号步进电机测忙函数
 * 输    入：无
 * 返    回：繁忙返回1，空闲返回0无
 ******************************************************************/
uint8_t L6470_BUSY1(void) {
	BUSY = HAL_GPIO_ReadPin(dSPIN_BUSY_Port, dSPIN_BUSY_Pin);
	if (BUSY == 0)
		return 0x01;
	else
		return 0x00;
}

/********************************************************************
 * 函数说明：配置L6470各寄存器初值
 * 输    入：寄存器地址，对应的值
 * 返    回：无
 ******************************************************************/
void dSPIN_Registers_Set(dSPIN_RegsStruct_TypeDef* dSPIN_RegsStruct) {
	dSPIN_Set_Param(dSPIN_ABS_POS, dSPIN_RegsStruct->ABS_POS);
	dSPIN_Set_Param(dSPIN_EL_POS, dSPIN_RegsStruct->EL_POS);
	dSPIN_Set_Param(dSPIN_MARK, dSPIN_RegsStruct->MARK);
	dSPIN_Set_Param(dSPIN_SPEED, dSPIN_RegsStruct->SPEED);
	dSPIN_Set_Param(dSPIN_ACC, dSPIN_RegsStruct->ACC);
	dSPIN_Set_Param(dSPIN_DEC, dSPIN_RegsStruct->DEC);
	dSPIN_Set_Param(dSPIN_MAX_SPEED, dSPIN_RegsStruct->MAX_SPEED);
	dSPIN_Set_Param(dSPIN_MIN_SPEED, dSPIN_RegsStruct->MIN_SPEED);
	dSPIN_Set_Param(dSPIN_FS_SPD, dSPIN_RegsStruct->FS_SPD);
	dSPIN_Set_Param(dSPIN_KVAL_HOLD, dSPIN_RegsStruct->KVAL_HOLD);
	dSPIN_Set_Param(dSPIN_KVAL_RUN, dSPIN_RegsStruct->KVAL_RUN);
	dSPIN_Set_Param(dSPIN_KVAL_ACC, dSPIN_RegsStruct->KVAL_ACC);
	dSPIN_Set_Param(dSPIN_KVAL_DEC, dSPIN_RegsStruct->KVAL_DEC);
	dSPIN_Set_Param(dSPIN_INT_SPD, dSPIN_RegsStruct->INT_SPD);
	dSPIN_Set_Param(dSPIN_ST_SLP, dSPIN_RegsStruct->ST_SLP);
	dSPIN_Set_Param(dSPIN_FN_SLP_ACC, dSPIN_RegsStruct->FN_SLP_ACC);
	dSPIN_Set_Param(dSPIN_FN_SLP_DEC, dSPIN_RegsStruct->FN_SLP_DEC);
	dSPIN_Set_Param(dSPIN_K_THERM, dSPIN_RegsStruct->K_THERM);
	dSPIN_Set_Param(dSPIN_OCD_TH, dSPIN_RegsStruct->OCD_TH);
	dSPIN_Set_Param(dSPIN_STALL_TH, dSPIN_RegsStruct->STALL_TH);
	dSPIN_Set_Param(dSPIN_STEP_MODE, dSPIN_RegsStruct->STEP_MODE);
	dSPIN_Set_Param(dSPIN_ALARM_EN, dSPIN_RegsStruct->ALARM_EN);
	dSPIN_Set_Param(dSPIN_CONFIG, dSPIN_RegsStruct->CONFIG);
}

/********************************************************************
 * 函数说明：向L6470发送一个空指令。（不执行任何动作）
 * 输    入：无
 * 返    回：无
 ******************************************************************/
void dSPIN_Nop(void) {
	/* Send NOP operation code to dSPIN */
	dSPIN_Communicate_Byte(dSPIN_NOP);
}

/********************************************************************
 * 函数说明：Set_Param(寄存器地址,写入的值)命令，设定指定寄存器的值，
 有些寄存器要条件满足才可写（详见数据手册）
 错误寄存器名或错误值会导致命令忽略；WRONG_CMD 标志位高电平
 * 输    入：寄存器名称（地址），寄存器值。1-3字节不等
 * 返    回：无
 ******************************************************************/
void dSPIN_Set_Param(dSPIN_Registers_TypeDef param, uint32_t value) {
	/* Send SetParam operation code to dSPIN */
//	while(dSPIN_Busy_HW());
	dSPIN_Communicate_Byte(dSPIN_SET_PARAM | param);
	switch (param) {
	case dSPIN_ABS_POS:
		;
	case dSPIN_MARK:
		;
	case dSPIN_SPEED:
		/* Send parameter - byte 2 to dSPIN */
		dSPIN_Communicate_Byte((uint8_t) (value >> 16));
	case dSPIN_ACC:
		;
	case dSPIN_DEC:
		;
	case dSPIN_MAX_SPEED:
		;
	case dSPIN_MIN_SPEED:
		;
	case dSPIN_FS_SPD:
		;
	case dSPIN_INT_SPD:
		;
	case dSPIN_CONFIG:
		;
	case dSPIN_STATUS:
		/* Send parameter - byte 1 to dSPIN */
		dSPIN_Communicate_Byte((uint8_t) (value >> 8));
	default:
		/* Send parameter - byte 0 to dSPIN */
		dSPIN_Communicate_Byte((uint8_t) (value));
	}
}

/********************************************************************
 * 函数说明：Get_Param(寄存器地址)命令，读取指定寄存器的值，
 可以随时读取
 错误寄存器地址会导致命令忽略；WRONG_CMD 标志位高电平
 * 输    入：寄存器地址
 * 返    回：返回相应寄存器值，1-3字节不等
 ******************************************************************/
uint32_t dSPIN_Get_Param(dSPIN_Registers_TypeDef param) {
	uint32_t temp = 0;
	uint32_t rx = 0;

	/* Send GetParam operation code to dSPIN */
	temp = dSPIN_Communicate_Byte(dSPIN_GET_PARAM | param);
	/* MSB which should be 0 */
	temp = temp << 24;
	rx |= temp;
	switch (param) {
	case dSPIN_ABS_POS:
		rx |= dSPIN_Communicate_Byte(0) << 16;
		rx |= dSPIN_Communicate_Byte(0) << 8;
		rx |= dSPIN_Communicate_Byte(0);
		break;
	case dSPIN_MARK:
		break;
	case dSPIN_SPEED:
		rx |= dSPIN_Communicate_Byte(0) << 16;
		rx |= dSPIN_Communicate_Byte(0) << 8;
		rx |= dSPIN_Communicate_Byte(0);
		break;
	case dSPIN_ACC:
		break;
	case dSPIN_DEC:
		break;
	case dSPIN_MAX_SPEED:
		break;
	case dSPIN_MIN_SPEED:
		break;
	case dSPIN_FS_SPD:
		break;
	case dSPIN_INT_SPD:
		break;
	case dSPIN_CONFIG:
		break;
	case dSPIN_STATUS:
		temp = dSPIN_Communicate_Byte((uint8_t) (0x00));
		temp = temp << 8;
		rx |= temp;
		break;
	default:
		temp = dSPIN_Communicate_Byte((uint8_t) (0x00));
		rx |= temp;
		break;
	}
	return rx;
}

// 把L6470各种长度的寄存器原始补码数据符号位左移成正常的32位数字。
int32_t dSPIN_RegToInt(uint32_t raw, uint32_t len) {
	int negative = raw >> (len - 1);
	if(negative) {
		// make an binary 1111100000 mask
		int mask = (int32_t)(1u << 31);
		mask >>= 32 - len;
		return (int32_t)(raw | mask);
	} else {
		return (int32_t)raw;
	}
}

/*******
 * 获取目前的电机绝对位置
*******/
int32_t dSPIN_Get_Pos() {
	uint32_t raw = dSPIN_Get_Param(dSPIN_ABS_POS);
	return dSPIN_RegToInt(raw, 22);
}

/********************************************************************
 * 函数说明：RUN(FWD/REV,speed)命令 驱动电机以speed/S速度运行，
 此命令可随时执行，执行时BUSY低电平，直至达到目标速度
 speed应小于最大速度，大于最小速度
 Go_HOME函数无法返回绝对位置
 * 输    入：FWD:反转	REV:正转	speed:速度（0-15625步/秒）
 * 返    回：无
 ******************************************************************/
void dSPIN_Run(dSPIN_Direction_TypeDef direction, uint32_t speed) {
	/* Send RUN operation code to dSPIN */
	dSPIN_Communicate_Byte(dSPIN_RUN | direction);
	/* Send speed - byte 2 data dSPIN */
	dSPIN_Communicate_Byte((uint8_t) (speed >> 16));
	/* Send speed - byte 1 data dSPIN */
	dSPIN_Communicate_Byte((uint8_t) (speed >> 8));
	/* Send speed - byte 0 data dSPIN */
	dSPIN_Communicate_Byte((uint8_t) (speed));
}

/********************************************************************
 * 函数说明：Move(FWD/REV,steps)命令，驱动电机运行指定步数。
 Go_HOME函数返回绝对位置
 电机停止状态才能用此命令，否则命令忽略，NOTPERF_CMD标志位高电平
 执行时BUSY低电平，直至命令完成
 绝对位置电机锁死
 * 输    入：0000000-2097152	   Go_HOME函数返回绝对位置
 2097153-4194303	   Go_HOME函数返回下一HOME位置
 * 返    回：返回状态
 ******************************************************************/
void dSPIN_Move(dSPIN_Direction_TypeDef direction, uint32_t n_step) {
	/* Send Move operation code to dSPIN */
	dSPIN_Communicate_Byte(dSPIN_MOVE | direction);
	/* Send n_step - byte 2 data dSPIN */
	dSPIN_Communicate_Byte((uint8_t) (n_step >> 16));
	/* Send n_step - byte 1 data dSPIN */
	dSPIN_Communicate_Byte((uint8_t) (n_step >> 8));
	/* Send n_step - byte 0 data dSPIN */
	dSPIN_Communicate_Byte((uint8_t) (n_step));
}
/********************************************************************
 * 函数说明：Reset Device 命令会复位L6470重新上电。
 重新上电时，电源桥禁止状态。
 * 输    入：无
 * 返    回：无
 ******************************************************************/
void dSPIN_Reset_Device(void) {
	/* Send ResetDevice operation code to dSPIN */
	dSPIN_Communicate_Byte(dSPIN_RESET_DEVICE);
}
/********************************************************************
 * 函数说明：Reset_Pos命令会重置ABS_POS绝对位置为0，也是HOME位置
 * 输    入：无
 * 返    回：无
 ******************************************************************/
void dSPIN_Reset_Pos(void) {
	/* Send ResetPos operation code to dSPIN */
	dSPIN_Communicate_Byte(dSPIN_RESET_POS);
}
/********************************************************************
 * 函数说明：SoftStop命令会导致电机立即减速到零速，
 电机停止使用的减速值存储在DEC寄存器中
 当电机处于高阻抗状态，一个SoftStop的命令强制退出电源桥
 此命令可以随时随地立即执行。
 此命令使BUSY标志低电平，直到电机停止。
 * 输    入：无
 * 返    回：返回状态
 ******************************************************************/
// 这个函数不会漏电
void dSPIN_Soft_Stop(void) {
	/* Send SoftStop operation code to dSPIN */
	dSPIN_Communicate_Byte(dSPIN_SOFT_STOP);
}
/********************************************************************
 * 函数说明：Hard_Stop命令会导致任意速度的电机立即停止。（无减速停止）
 此命令可以随时立即执行。此命令使BUSY标志电平，直到电机停止。
 当电机处于高阻抗状态，硬停止命令会强制退出电源桥
 * 输    入：无
 * 返    回：无
 ******************************************************************/
void dSPIN_Hard_Stop(void) {
	/* Send HardStop operation code to dSPIN */
	dSPIN_Communicate_Byte(dSPIN_HARD_STOP);
}
/********************************************************************
 * 函数说明：SoftHiZ命令禁用电源桥（高阻抗状态）后减速到零,
 所用的减速度值存储在DEC寄存器中。
 当电源桥被禁用时，HiZ标志位是高电平。
 当电机停止时，SoftHiZ的命令强制的电源桥进入高阻抗状态。
 此命令可以随时随地立即执行。此命令使BUSY标志低，直到电机停止。
 * 输    入：无
 * 返    回：返回状态
 ******************************************************************/
void dSPIN_Soft_HiZ(void) {
	/* Send SoftHiZ operation code to dSPIN */
	dSPIN_Communicate_Byte(dSPIN_SOFT_HIZ);
}

/********************************************************************
 * 函数说明：HardHiZ命令立即关闭电源桥（高阻抗状态）。（不减速）
 当电源桥被禁用时，HiZ标志位是高电平。
 当电机停止时，HardHiZ的命令强制的电源桥进入高阻抗状态。
 此命令可以随时随地立即执行。此命令使BUSY标志低，直到电机停止。
 * 输    入：无
 * 返    回：返回状态
 ******************************************************************/
void dSPIN_Hard_HiZ(void) {
	/* Send HardHiZ operation code to dSPIN */
	dSPIN_Communicate_Byte(dSPIN_HARD_HIZ);
}
/********************************************************************
 * 函数说明：getStatus命令读取Status状态寄存器的值。
 getStatus命令重设的状态寄存器警告标志。
 forcesthe命令使系统的任何错误状态退出
 The GetStatus command DOES NOT reset the HiZ flag.
 * 输    入：无
 * 返    回：2字节状态
 ******************************************************************/
uint16_t dSPIN_Get_Status(void) {
	uint16_t temp = 0;
	uint16_t rx = 0;

	/* Send GetStatus operation code to dSPIN */
	dSPIN_Communicate_Byte(dSPIN_GET_STATUS);
	/* Send zero byte / receive MSByte from dSPIN */
	temp = dSPIN_Communicate_Byte((uint8_t) (0x00));
	temp = temp << 8;
	rx |= temp;
	/* Send zero byte / receive LSByte from dSPIN */
	temp = dSPIN_Communicate_Byte((uint8_t) (0x00));
	rx |= temp;
	return rx;
}
/********************************************************************
 * 函数说明：GoMark命令将以最短路径运行到标志位置
 相当于Go_To(MARK)命令
 如果要确定电机方向，必须使用GoTo_DIR命令
 此命令使BUSY标志低电平，直到电机停止。
 此命令运行时不能使用其它命令，否则命令忽略，NOTPERF_CMD标志位高电平
 * 输    入：无
 * 返    回：无
 ******************************************************************/
void dSPIN_Go_Mark(void) {
	/* Send GoMark operation code to dSPIN */
	dSPIN_Communicate_Byte(dSPIN_GO_MARK);
}
/********************************************************************
 * 函数说明：GoHome命令将以最短路径运行到零位置
 相当于Go_To(000)命令
 如果要确定电机方向，必须使用GoTo_DIR命令
 此命令使BUSY标志低电平，直到电机停止。
 此命令运行时不能使用其它命令，否则命令忽略，NOTPERF_CMD标志位高电平
 * 输    入：无
 * 返    回：无
 ******************************************************************/
void dSPIN_Go_Home(void) {
	/* Send GoHome operation code to dSPIN */
	dSPIN_Communicate_Byte(dSPIN_GO_HOME);
}

/********************************************************************
 * 函数说明：Go_To(ABC_POS)命令，驱动电机到绝对位置。Go_HOME函数返回绝对位置
 绝对位置电机锁死
 * 输    入：顺：0x000000-0x1FFFFF	   0 step to 2097151 step
 逆：0x3FFFFE-0x200000	   0 step to 2097150 step
 * 返    回：返回状态
 ******************************************************************/
void dSPIN_Go_To(uint32_t abs_pos) {
	/* 发送 GoTo 操作码到L6470 */
	dSPIN_Communicate_Byte(dSPIN_GO_TO);
	/* 发送绝对位置第2字节 - byte 2 data to L6470 */
	dSPIN_Communicate_Byte((uint8_t) (abs_pos >> 16));
	/* 发送绝对位置第1字节 - byte 2 data to L6470  */
	dSPIN_Communicate_Byte((uint8_t) (abs_pos >> 8));
	/* 发送绝对位置第0字节 - byte 2 data to L6470  */
	dSPIN_Communicate_Byte((uint8_t) (abs_pos));
}
/********************************************************************
 * 函数说明：Go_To_Dir(FWD/REV,ABC_POS)命令，驱动电机到绝对位置。Go_HOME函数返回绝对位置
 绝对位置电机锁死
 * 输    入：顺：0x000000-0x200000	   0 step to 2097151 step Go_HOME函数返回绝对位置
 逆：0x200001-3FFFFF	   0 step to 2097150 step Go_HOME函数返回下一HOME位置 ******************
 * 返    回：返回状态
 ******************************************************************/
void dSPIN_Go_To_Dir(dSPIN_Direction_TypeDef direction, uint32_t abs_pos) {
	/* Send GoTo_DIR operation code to dSPIN */
	dSPIN_Communicate_Byte(dSPIN_GO_TO_DIR | direction);
	/* Send absolute position parameter - byte 2 data to dSPIN */
	dSPIN_Communicate_Byte((uint8_t) (abs_pos >> 16));
	/* Send absolute position parameter - byte 1 data to dSPIN */
	dSPIN_Communicate_Byte((uint8_t) (abs_pos >> 8));
	/* Send absolute position parameter - byte 0 data to dSPIN */
	dSPIN_Communicate_Byte((uint8_t) (abs_pos));
}

/**
 * @brief  Issues dSPIN Step Clock command.设置步模式
 * @param  Movement direction (FWD, REV)
 * @retval None
 */
void dSPIN_Step_Clock(dSPIN_Direction_TypeDef direction) {
	/* Send StepClock operation code to dSPIN */
	dSPIN_Communicate_Byte(dSPIN_STEP_CLOCK | direction);
}
/**
 * @brief  Issues dSPIN Go Until command.
 * @param  Action, Movement direction, Speed
 * @retval None
 */
void dSPIN_Go_Until(dSPIN_Action_TypeDef action,
		dSPIN_Direction_TypeDef direction, uint32_t speed) {
	/* Send GoUntil operation code to dSPIN */
	dSPIN_Communicate_Byte(dSPIN_GO_UNTIL | action | direction);
	/* Send speed parameter - byte 2 data to dSPIN */
	dSPIN_Communicate_Byte((uint8_t) (speed >> 16));
	/* Send speed parameter - byte 1 data to dSPIN */
	dSPIN_Communicate_Byte((uint8_t) (speed >> 8));
	/* Send speed parameter - byte 0 data to dSPIN */
	dSPIN_Communicate_Byte((uint8_t) (speed));
}

/**
 * @brief  Issues dSPIN Release SW command.
 * @param  Action, Movement direction
 * @retval None
 */
void dSPIN_Release_SW(dSPIN_Action_TypeDef action,
		dSPIN_Direction_TypeDef direction) {
	/* Send ReleaseSW operation code to dSPIN */
	dSPIN_Communicate_Byte(dSPIN_RELEASE_SW | action | direction);
}
/********************************************************************
 * 函数说明：检测L6470硬件忙寄存器
 * 输    入：无
 * 返    回：返回1忙，返回0闲
 *****************************************************************
 uint8_t dSPIN_Busy_HW(void)
 {
 if(!(GPIO_ReadInputDataBit(dSPIN_BUSY_Port, dSPIN_BUSY_Pin))) return 0x01;
 else return 0x00;
 }  */
/********************************************************************
 * 函数说明：检测L6470软件忙寄存器
 * 输    入：无
 * 返    回：返回1忙，返回0闲
 ******************************************************************/
uint8_t dSPIN_Busy_SW(void) {
	if (!(dSPIN_Get_Status() & dSPIN_STATUS_BUSY))
		return 0x01;
	else
		return 0x00;
}
/********************************************************************
 * 函数说明：发送/接收一个字节到SPI
 * 输    入：发送字节
 * 返    回：接收字节
 ******************************************************************/
uint8_t dSPIN_Communicate_Byte(uint8_t byte) {
	uint8_t rec;
	// 片选信号 - 低
	HAL_GPIO_WritePin(dSPIN_nSS_Port, dSPIN_nSS_Pin, GPIO_PIN_RESET);
	// 发送一个字节 一毫秒超时
	HAL_SPI_TransmitReceive(dSPIN_SPI, &byte, &rec, 1, 1);
	// 片选信号 - 高
	HAL_GPIO_WritePin(dSPIN_nSS_Port, dSPIN_nSS_Pin, GPIO_PIN_SET);
	return rec;
}
