#include "beep.h"

int music1[8] = {M_Re,M_Mi,M_So,H_Do,0,M_La,H_Do,H_Do};
int music2[14] = {H_Do,0,M_So,0,M_Mi,0,M_Xi,M_Xi,M_Xi,M_Xi,M_La,0,M_Xi,H_Do};
int music3[23] = {L_So,0,M_Do,0,0,M_Mi,M_Re,0,M_Do,0,0,0,M_Mi,0,M_So,0,0,M_La,M_Xi,0,H_Do,H_Do,H_Do};
int music4[33] = {M_Do,0,M_Do,M_Mi,M_Mi,M_Mi,M_Mi,0,
									M_Do,0,M_Do,M_Mi,M_Mi,M_Mi,M_Mi,0,
									M_La,0,M_La,0,M_So,0,M_La,0,
									M_So,M_Do,0,M_Mi,M_Mi,0,0//,
//									H_Do,0,M_La,M_La,M_So,0,M_La,0,0,
//									M_So,0,M_Do,M_Re,M_Re,0,0,
//									M_Xi,M_Xi,M_Xi,M_Xi,M_Xi,0,M_Xi,0,M_So,M_Mi,0,M_So,M_So,M_So
									};
int music5[62] = {H_Do,0,M_Xi,0,M_So,0,
									M_So,0,M_La,1145,M_La,0,
									M_So,0,M_So,0,M_La,1145,M_La,0,
									M_So,0,M_So,0,M_La,1145,M_La,0,
									M_La,0,M_La,M_La,M_Xi,M_Xi,H_Do,H_Do,H_Re,H_Re,
									M_Xi,M_Xi,M_Xi,M_Xi,M_So,M_So,M_So,M_So,
	                H_Fa,H_Fa,H_Fa,H_Fa,H_Re,H_Re,H_Re,0,
									H_Re,H_Re,0,H_Re,H_Re,H_Re,H_Mi,H_Mi};//Only My Railgun
int music6[56] = {H_Do,H_Do,H_Do,0,H_Do,0,M_Xi,H_Do,H_Do,0,
									H_Re,0,H_Mi,0,H_Fa,0,
									H_Mi,H_Mi,0,H_Mi,H_Mi,0,M_Xi,0,M_Xi,M_Xi,M_Xi,0,0,
									M_La,M_La,M_La,0,M_La,0,M_So,M_La,M_La,0,
									H_Fa,0,H_Mi,0,H_Re,0,
									H_Re,H_Re,0,H_Do,H_Do,0,H_Re,0,H_Mi,H_Mi,H_Mi
									};
int music_nxt[40] = {M_Do, M_Do,M_Do,M_Do,M_Do,M_Do,M_Do,M_Do,
										M_La,M_La,M_La,M_La,M_La,M_La,M_La,
										M_Fa,M_Fa,M_Fa,M_Fa,M_Fa,
										M_Do,M_Do,M_Do,M_Do,M_Do,M_Do,M_Do,M_Do,
										H_Do,H_Do,H_Do,H_Do,H_Do,H_Do,H_Do,H_Do,H_Do,H_Do,H_Do,H_Do};

void Beep(void)
{
	static int beep_count = 0;
	if(beep_count < sizeof(MUSIC)/sizeof(int))
	{
		if(MUSIC[beep_count] != 0)
		{
			TIM4->CCR3 = MUSIC[beep_count]/2;
		}
		else
		{
			TIM4->CCR3 = MUSIC[beep_count];
		}
		TIM4->ARR = MUSIC[beep_count];
		beep_count++;
	}
	else
	{
		HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
	}
}
