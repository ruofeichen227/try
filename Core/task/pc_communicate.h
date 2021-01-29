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


#ifndef PC_COMMUNICATE_H
#define PC_COMMUNICATE_H

#include "usart.h"


#define PC_huart huart1
#define USART_pc USART1

/*
u8——1个字节，无符号型（不能表达负数，如果用来当作负数的话，就出错了）；
u16 ——2个字节，无符号型
u32——4个字节，无符号型
int——4个字节,有符号型，可以表达负整数；
float ——4个字节，有符号型，可以表达负数/小数；
double——8个字节，有符号弄，可以表达负数/小数；
char——1个字节
*/

enum Data_Type_t
{
	Pan,
	Mcu,
  Config,
  Energy,
  Speed	
};

void PcDataReceiveHandle(void);
void PC_RxCpltCallback(UART_HandleTypeDef *huart);
void PC_communicate_init(void);
void PcDataTramsmit(enum Data_Type_t data_type);

#endif
