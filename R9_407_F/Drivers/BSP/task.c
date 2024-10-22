/*
 * @Author: lisir lisir@rehand.com
 * @Date: 2024-06-07 16:01:18
 * @LastEditors: lisir lisir@rehand.com
 * @LastEditTime: 2024-08-13 10:44:11
 * @FilePath: \R9_407F\R9_407F\R9_407_F\Drivers\BSP\task.c
 * @Description: 主任务列表
 */
#include "./BSP/task.h"

extern ADCDATA adcdata;

/**
 * @description: 
 * @return {*}
 */

/**
 * @description: 所有任务初始化代码
 * @return {*}
 */
void Hard_devInit(void)
{
		HAL_Init();                                 //* 初始化HAl库 */
		MoterdriveInit();
		sys_stm32_clock_init(336, 8, 2, 7);     /* 初始化时钟频率,168Mhz 主时钟*/
		delay_init(168);                        /*初始化延时时钟频率*/
		usart_init(115200);                     /* 串口通讯波特率 115200 */
		led_init();                             /* 转向灯初始化 */
		// key_init();								/*按键初始化*/
		btim_timx_int_init(10 - 1, 8400 - 1);   /*定时器中断初始化 产生固定 1ms 的定时器中断 */
		brake_init(2000-1,84-1);                           /*抱闸初始化*/   
		getadcDataInit();                      /*ADC数据采集初始化*/
		MPU_Init();                            /*陀螺仪初始化*/
		mpu_dmp_init();
		Host_ModbusDap21_Init();              /*与DYPA21通讯*/
		SlaveModbus_Init();                  /*与RK3588作为从机通讯*/
		can_init(CAN_SJW_1TQ, CAN_BS2_6TQ, CAN_BS1_7TQ, 6, CAN_MODE_NORMAL);  /* CAN初始化, 正常模式, 波特率500Kbps */
		iwdg_init(IWDG_PRESCALER_64, 1500);      /* 预分频数为64,重载值为1500,溢出时间约为3s */
		filterInit();                    /*初始化滤波器*/
		vSetUpMlx90393();
		g_slaveReg[0] = 0x68;//本机设备作为Modbus从机时的设备ID
		printf("ERROR");
}

void Task_GetMlx90393(void)
{

	vInMeasurementNormal();

}

/**
 * @description: 灯控程序
 * @return {*}
 */
void Task_led_control(void)
{
	led_beepControlRK3588();
//	led_beepControl();
	LED0_TOGGLE();

}

/**
 * @description: 
 *  针对 R9系统的所有ADC 数据采集 ，
 *  一 、ADC1 采集7通道数据 包含   
 * (1)  摇杆数据采集         PA2 PA3
 * (2)  抱闸 数据监测        PA4 PA5
 * (3)  底盘电机电流检测     PA6 PA7
 * (4)  电池电压            PC7 pc5
/
 * 二、   ADC3 数据采集  包含
 * (1) 推杆1~6  位置检测  PF5 PF3 PF4 PF6 PF8 PF7
 * (2) 推杆 1~6 的电流检测 PC2 PC3 PC0 PC1 PF9 PF10
 * @return {*}
 */
void Task_GetADC_AllData(void)
{

	getadcData();
	// vInMeasurementNormal();
    /*数据采集及测试*/
	// printf("lift_pos:%d,pedestal_pos:%d,backboard_pos:%d,legangle_pos:%d,leglength_pos:%d,support_pos:%d\n",adcdata.lift_pos,adcdata.pedestal_pos,adcdata.backboard_pos,adcdata.legangle_pos,adcdata.leglength_pos,adcdata.support_pos);
	// printf("lift_current:%d,pedestal_current:%d,backboard_current:%d,legangle_current:%d,leglength_current:%d,support_current:%d\n",adcdata.lift_current,adcdata.pedestal_current,adcdata.backboard_current,adcdata.legangle_current,adcdata.leglength_current,adcdata.support_current);
	// printf("adcdata.l_current :%d, adcdata.r_current %d\n",adcdata.l_current,adcdata.r_current);
	// printf("Xbase:%d,Ybase:%d,xdata:%d,ydata:%d\t\n",adcdata.adc_xbase,adcdata.adc_ybase,adcdata.adc_x,adcdata.adc_y);
	// printf("adcdata.A1V:%f,adcdata.A2V:%f,adcdata.B1V:%f,adcdata.B2V:%f\n",adcdata.A1V,adcdata.A2V,adcdata.B1V,adcdata.B2V);
	legKinematics();
	// printf("legkinematicspra.L4:%f,theta1:%f,theta2:%f,theta3:%f,theta4:%f\n",legkinematicspra.L4,legkinematicspra.theta1*180/PI,legkinematicspra.theta2*180/PI,legkinematicspra.theta3*180/PI,legkinematicspra.theta4*180/PI);
	// printf("x_o:%f,y_o:%f,z_o:%f\n",legkinematicspra.x_o,legkinematicspra.y_o,legkinematicspra.z_o);
}	

/**
 * @description: 底盘控制既驱动执行
 * @return {*}
 */
void Task_UnderpanDrive(void)
{
	underpanExcute();
}

/**
 * @description: 推杆控制及驱动
 * @return {*}
 */
void Task_linearactuatorDrive(void)
{	
	// printf("BACKSET_CMD:%d\n",g_slaveReg[68]);
	linearactuatorTest(); 

}

/**
 * @description: MPU6050 执行
 * @return {*}
 */
void Task_gyroscopeData(void)
{
	MPU6050Excute();
}

/**
 * @description: 与RK3588 通讯程序执行
 * @return {*}
 */
void Task_ModbusSlaveExecute (void)
{	
	SlaveModbus_Event();//Modbus事件处理函数(执行读或者写的判断)--从机地址0x01	
}

/**
 * @description: 超声波测距程序，目前单从没问题
 * @return {*}
 */
void Task_ultrasonicreadExecute1 (void)
{
			HostDap21_Read03_slave(0x01,0x0101,0x0001);//参数1从机地址，参数2起始地址，参数3寄存器个数
			
			if(modbus_dap21.Host_send_flag)
			{

				modbus_dap21.Host_send_flag=0;//清空发送结束数据标志位

				HOST_ModbusDap21RX();//接收数据进行处理
			}

}

void Task_ultrasonicreadExecute2 (void)
{
			HostDap21_Read03_slave(0x02,0x0100,0x0001);//参数1从机地址，参数2起始地址，参数3寄存器个数			
			if(modbus_dap21.Host_send_flag)
			{
				modbus_dap21.Host_send_flag=0;//清空发送结束数据标志位
				HOST_ModbusDap21RX();//接收数据进行处理
			}
			// 仅在驾驶过程或未在充电中传输有效数据
			if((g_slaveReg[10] || g_slaveReg[11]) && g_slaveReg[2]==0)
			{
				g_slaveReg[7] = dap21Data.dyplength2 ; /*超声波数据通过MOdbus上传至上位机进行显示/应用*/   
			}	
			else
			{

				g_slaveReg[7] = 2000 ; /*超声波数据通过MOdbus上传至上位机进行显示/应用*/   
			}
			// printf("distence2: %d\n",dap21Data.dyplength2);
	
}


/**
 * @description: 按键板Can通讯实现
 * @return {*}
 */
void Task_CanKeyRun(void)
{
      CanReadexcute();
}
void Task_ex_handl(void)
{

	ex_handl_excute();

}

void Task_Comsdetect(void)
{	

	if (comheartstate.detect_falge)
	{
		ComheartReset();
		comheartstate.detect_falge =0;	
	}
	ComheartDetect(100);
	// printf("g_slave[87]:%d,comheartstate.com_state:%d\n",g_slaveReg[87] ,comheartstate.com_state)	;
}
