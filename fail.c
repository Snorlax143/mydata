#include <stdio.h>
#include <stdlib.h>
#include "../../BBBio_lib/BBBiolib.h"
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>

#define checkpoint 5
#define distance 50
#define PWM 25.0f
//---------------global value---------------
const float kp = 1;
const float ki = 0;
const float kd = 1;
struct timespec start;
struct timespec end;
struct timespec diff;
//------------------------------------------
typedef struct PID
{
	float target;     //目標
	float Proportion; //kp
	float Integral;   //ki
	float Derivative; //kd
	float pre2Err;    //pre two step err
	float preErr;     //pre err
}PID;
float PIDCale(PID *p,float input)
{
	float Err,pErr,dErr,sum,acc_err;

	Err = p->target - input;  //當前誤差
	/*kp * delta e(k) + ki * e(k) + kd[delta e(k) - delta e(k-1)]; "delta e(k) = e(k) - e(k-1)" "Incremental_PID method"*/
	pErr = Err - p->preErr;   //比例項增量式誤差
	//sum = p->Proportion * pErr + p->Integral * Err;// + p->Derivative * (pErr * (p->preErr - p->pre2Err));

	/*位置式PID控制*/
	acc_err = (acc_err + Err) / checkpoint;//accumulated error
	//sum = p->Proportion * Err + p->Integral * acc_err;//PI controll
	sum = p->Proportion * Err + p->Derivative * (Err + p->preErr) / checkpoint + p->Integral * acc_err;//PD controll

	//dErr = Err - 2 * p->preErr + p->pre2Err;  //微分項增量式誤差
	//sum = p->Proportion * pErr + p->Derivative * dErr;// + p->Integral * Err;  //控制量增量
	p->pre2Err = p->preErr;
	p->preErr = Err;
	return sum;
}
void PIDInit(PID p)
{
	//printf("pidinit");
	memset(&p,0,sizeof(PID));  //初始化
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
	//printf("main\n");
	//int del;
	iolib_init();
	iolib_setdir(8,11, BBBIO_DIR_IN);
	iolib_setdir(8,12, BBBIO_DIR_IN);
	
	const float PWM_HZ = 100.0f;    /* 100 Hz */
	const float duty_A = PWM;
	float duty_B = PWM;
	float out_A[distance];
	float out_B[distance];
	float PWM_B[distance];
	
	float countA = 0,countB = 0;
	char now_valueA = 0, tmpA = 1;
	char now_valueB = 0, tmpB = 1;
	float err = 0,Pout = 0;
	float time,used_time;
	int i = 0,count = 0;
	
	struct PID p;
	BBBIO_PWMSS_Setting(1, PWM_HZ ,0.1f , duty_A);
	BBBIO_PWMSS_Setting(0, PWM_HZ ,duty_B , 0.1f);
		
	PIDInit(p);
	p.Proportion = kp;
	p.Integral = ki;
	p.Derivative = kd;
	//printf("log1.5\n");
	WheeledB_init();
	WheeledA_init();
	//printf("log2\n");
	BBBIO_ehrPWM_Enable(1);
	BBBIO_ehrPWM_Enable(0);
	//printf("log3\n");

	while(count < distance)
	{
		countA = 0;
		countB = 0;
		//clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
		
		do{
			if (is_high(8,11))
			{
				tmpA = now_valueA;
				now_valueA = 1;
			}
			else
			{
				tmpA = now_valueA;
				now_valueA = 0;
			}
			if (is_high(8,12))
			{
				tmpB = now_valueB;
				now_valueB = 1;

			}
			else
			{           
				tmpB = now_valueB;
				now_valueB = 0;
			}

			if(tmpA == 0 && now_valueA == 1)
			{
				countA++;
			}

			if(tmpB == 0 && now_valueB == 1)
			{
				countB++;
			}		
			iolib_delay_ms(10);
		}while(countA < checkpoint);
		/*
		clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end);
		diff.tv_sec = ( end.tv_sec - start.tv_sec );
		diff.tv_nsec = ( end.tv_nsec - start.tv_nsec );
		if (diff.tv_nsec < 0) 
		{
			diff.tv_sec--;
			diff.tv_nsec += 1000000000;
		}
		int usec = diff.tv_nsec + diff.tv_sec * 100000000;
		double resultTime = (double)usec / 1000000;
		printf("TimeMeasure: used time %6.1lf[ms]\n", resultTime);
		*/
		printf("%f\n%f\n",countA,countB);
		/*	
		if(count > 0)	
			if(countB - out_B[count-1] < -5)
				countB = out_B[count];
		*/
				
		p.target = (float)countA;
		Pout = PIDCale(&p,countB);	
		duty_B += Pout;

		if(duty_B < 0)
			duty_B = 0;
		if(duty_B > PWM)
			duty_B = PWM;
		BBBIO_PWMSS_Setting(0, PWM_HZ ,duty_B , 0.1f);
		BBBIO_ehrPWM_Enable(0);printf("PWMB set to be %lf%\n",duty_B);
		
		//out_A[count] = countA;
		out_B[count] = countB;
		PWM_B[count] = duty_B;
		count++;
	}

	BBBIO_ehrPWM_Disable(BBBIO_PWMSS1);
	BBBIO_ehrPWM_Disable(BBBIO_PWMSS0);
	/*
	if(freopen("/BBBIOlib/Demo/Demo_LED/outputA.txt","w",stdout) == NULL)
		printf("重定向出错");
	
	for(i = 0;i < distance;i++)
	{
		printf("%f\n",out_A[i]);
	}
	fclose(stdout);
	*/
	if(freopen("/BBBIOlib/Demo/Demo_LED/outputB.txt","w",stdout) == NULL)
		printf("重定向出错");
	for(i = 0;i < distance;i++)
	{
		printf("%f\n",out_B[i]);
	}
	fclose(stdout);
	
	if(freopen("/BBBIOlib/Demo/Demo_LED/PWM_B.txt","w",stdout) == NULL)
		printf("重定向出错");
	for(i = 0;i < distance;i++)
	{
		printf("%f\n",PWM_B[i]);
	}
	fclose(stdout);
	
	/*
	   fwrite(out_A,4,sizeof(out_A),pFileA);
	   fwrite(out_B,4,sizeof(out_B),pFileB);
	   fclose(pFileA);
	   fclose(pFileB);
	 */

	iolib_free();
	return(0);
}
