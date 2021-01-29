#include "ramp.h"

/**
	* @brief 斜坡初始化函数
	* @param None
	* @retval None
	*/
void RampInit(struct ramp_t *ramp, int32_t scale)
{
  ramp->count = 0;
  ramp->scale = scale;
  ramp->out = 0;
}

/**
	* @brief 斜坡计算函数
	* @param 斜坡结构体
	* @retval 0~1之间的浮点数
	*/
float RampCalc(struct ramp_t *ramp)
{
  if (ramp->scale <= 0)
    return 0;
  ramp->count++;
  if (ramp->count >= ramp->scale)
    ramp->count = ramp->scale;
  
  ramp->out = ramp->count / ((float)ramp->scale);
  return ramp->out;
}
