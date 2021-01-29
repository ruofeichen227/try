/**
  ******************************************************************************
  * 文件名          : pc_communicate.c
  * 创建时间        : 2020.01.01
  * 作者            : 张大明
  *-----------------------------------------------------------------------------
  * 最近修改时间    : 2021.01.25
  * 修改人          : 丁冰
  ******************************************************************************
  * 1.本代码基于STMF427IIT6开发，编译环境为Keil 5，基于FreeRTOS进行开发。
  * 2.本代码只适用于RoboMaster机器人，不建议用于其他用途
	* 3.本代码包含大量中文注释，请以UTF-8编码格式打开
	* 4.本代码最终解释权归哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT所有
	* 
	* Copyright (c) 哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT 版权所有
  ******************************************************************************
**/

#include "pc_communicate.h"
#include "AttitudeResolve.h"
#include "robot.h"
#include "supervise.h"
#include "datatypes.h"
#include "string.h"

#define RX_LEN 16
#define TX_LEN 11
	
SendData data_rx;
McuData data_tx;

uint8_t pc_tx[TX_LEN] = {0};  
uint8_t pc_rx[RX_LEN] = {0}; 
uint8_t pc_rx_buf = 0; 

void PC_communicate_init(void)
{
	HAL_UART_Receive_IT(&PC_huart, pc_rx, RX_LEN);
}

void PC_RxCpltCallback(UART_HandleTypeDef *huart)
{	
	HAL_UART_Receive_IT(&PC_huart,&pc_rx_buf,1);
	PcDataReceiveHandle();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
	if(huart == &PC_huart)
	{
    PC_RxCpltCallback(huart);
	}
}

/**
	* @brief 接收上位机数据
	* @param none
	* @retval none
	*/
void PcDataReceiveHandle(void)
{
	static int data_count = 0;
	pc_rx[data_count] = pc_rx_buf;
	data_count++;
	if(pc_rx[0] != 's')
	{
		data_count = 0;
		return;
	}
	
	if(data_count < RX_LEN)		return;
	if(pc_rx[RX_LEN-1] != 'e')
	{
		data_count = 0;
		return;
	}
	memcpy(&data_rx,pc_rx,RX_LEN); 
	data_count = 0;
	LostCounterFeed(PC_INDEX);
}

/**
	* @brief 发送接口
	* @param none
	* @retval none
	*/
void PcDataTramsmit(enum Data_Type_t data_type)
{
	if(data_type==Speed)
		data_tx = generateSpeedMcuData(0);      //传入正确数据
	else if(data_type==Pan)
		data_tx = generatePanMcuData(0, 0);
	else if(data_type==Energy)
		data_tx = generateEnergyMcuData(0, 0);
	else if(data_type==Config)
		data_tx = generateConfigMcuData('a', 0, 0);
	memcpy(pc_tx, &data_tx, TX_LEN);
	HAL_UART_Transmit_IT(&PC_huart,(uint8_t *)pc_tx,TX_LEN);
}
