#include <stdio.h>
#include <stdlib.h>
#include "../../BBBio_lib/BBBiolib.h"
#include <string.h>
#define checkpoint 5

const float kp = 0.35;
const float ki = 0.2;
//const float kd = 0.2;

typedef struct PID{
	float target;     //目標
	float Proportion; //kp
	float Integral;   //ki
	//float Derivative; //kd
	float pre2Err;    //pre two step err
	float preErr;     //pre err
}PID;
float PIDCale(PID *p,float input)
{
	float Err,pErr,dErr,sum,dU;
	Err = p->target - input;  //當前誤差
	pErr = Err - p->preErr;   //比例項增量式誤差
	/*kp * delta e(k) + ki * e(k) + kd[delta e(k) - delta e(k-1)]; "delta e(k) = e(k) - e(k-1)"*/
	sum = p->Proportion * pErr + p->Integral * Err;// + p->Derivative * (pErr * (p->preErr - p->pre2Err));
	//dErr = Err - 2 * p->preErr + p->pre2Err;  //微分項增量式誤差
	//dU = p->Proportion * pErr + p->Derivative * dErr+p->Integral * Err;  //控制量增量
	p->pre2Err = p->preErr;
	p->preErr = Err;
	return sum;
}
void PIDInit(PID *p)
{
	memset(p,0,sizeof(PID));  //初始化
}


void WheeledA_init()
{
	char now_valueA = 0, tmpA = 0;

	BBBIO_ehrPWM_Enable(1);
	while(1)
	{
		if(is_high(8,11))
		{
			tmpA = now_valueA;
			now_valueA = 1;
		}
		else
			if(is_low(8,11))
			{
				tmpA = now_valueA;
				now_valueA = 0;
			}
		if(now_valueA == 1 && tmpA + now_valueA == 1)
		{
			BBBIO_ehrPWM_Disable(BBBIO_PWMSS1);	
			break;	
		}
	}
}

void WheeledB_init()
{
	char now_valueB = 0, tmpB = 0;

	BBBIO_ehrPWM_Enable(0);
	while(1)
	{
		if (is_high(8,12))
		{
			tmpB = now_valueB;
			now_valueB = 1;
		}	
		else
			if (is_low(8,12))
			{
				tmpB = now_valueB;
				now_valueB = 0;
			}
		if(now_valueB == 1 && tmpB + now_valueB == 1)
		{
			BBBIO_ehrPWM_Disable(BBBIO_PWMSS0);
			break;
		}
	}
}

int main(void)
{
	//int del;
	iolib_init();
	iolib_setdir(8,11, BBBIO_DIR_IN);
	iolib_setdir(8,12, BBBIO_DIR_IN);

	const float PWM_HZ = 100.0f;    /* 100 Hz */
	const float duty_A = 15.0f;
	float duty_B = 15.0f;
	/*
	   FILE *pFileA,*pFileB;

	   char out_A[101];
	   char out_B[101];
	   pFileA = fopen("outputA.txt","w");
	   pFileB = fopen("outputB.txt","w");

	   const float PWM_HZ = 100.0f;	//100 Hz 

	   if( NULL == pFileA )
	   {
	   printf( "open failure" );
	   return 1;
	   }
	   if( NULL == pFileB )
	   {
	   printf( "open failure" );
	   return 1;
	   }
	 */
	int countA = 0,countB = 0;
	char now_valueA = 0, tmpA = 1;
	char now_valueB = 0, tmpB = 1;
	float err = 0,Pout = 0;
	PID *p;
	
	BBBIO_PWMSS_Setting(1, PWM_HZ ,0.1f , duty_A);
	BBBIO_PWMSS_Setting(0, PWM_HZ ,duty_B , 0.1f);

	PIDInit(p);
	p->Proportion = kp;
	p->Integral = ki;
	//p->Derivative = kd;

	//WheeledA_init();
	WheeledB_init();
	WheeledA_init();

	BBBIO_ehrPWM_Enable(1);
	BBBIO_ehrPWM_Enable(0);
	while(1)
	{

		countA = 0;
		countB = 0;

		BBBIO_PWMSS_Setting(0, PWM_HZ ,duty_B , 0.1f);

		//BBBIO_ehrPWM_Enable(1);
		BBBIO_ehrPWM_Enable(0);


		do{
			if (is_high(8,11))
			{
				//del=100; // fast speed
				//printf("1\n");
				//out_A[count] = '1';
				tmpA = now_valueA;
				now_valueA = 1;
			}
			else
				if (is_low(8,11))
				{
					//del=500; // slow speed
					//printf("0\n");
					//out_A[count] = '0';
					tmpA = now_valueA;
					now_valueA = 0;
				}
			if(tmpA == 0 && now_valueA == 1)
			{
				countA++;
			}

			//iolib_delay_ms(10);

			//pin_high(8,12);
			if (is_high(8,12))
			{
				//printf("1\n");
				//out_B[count] = '1';
				tmpB = now_valueB;
				now_valueB = 1;

			}
			//iolib_delay_ms(del);
			//pin_low(8,12);
			else
				if (is_low(8,12))
				{           
					//printf("0\n");
					//out_B[count] = '0';
					tmpB = now_valueB;
					now_valueB = 0;
				}
			if(tmpB == 0 && now_valueB == 1)
			{
				countB++;
			}		
			//iolib_delay_ms(10);
		}while(countA < checkpoint && countB < checkpoint);

		printf("%d\n%d\n",countA,countB);
		p->target = (float)countA;
		//err = (float)(countA - countB);
		Pout = PIDCale(p,countB);	
		duty_B += Pout;

		if(duty_B < 5.0)
			duty_B = 5.0;

		printf("%lf\n",duty_B);	
		//iolib_delay_ms(2);

	}

	//BBBIO_ehrPWM_Disable(BBBIO_PWMSS1);
	//BBBIO_ehrPWM_Disable(BBBIO_PWMSS0);

	//fwrite(out_A,1,sizeof(out_A),pFileA);
	//fwrite(out_B,1,sizeof(out_B),pFileB);
	//fclose(pFileA);
	//fclose(pFileB);
	printf("%d\n",countA);
	printf("%d\n",countB);

	iolib_free();
	return(0);
}


