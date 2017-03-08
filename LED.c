#include <stdio.h>
#include <stdlib.h>
#include "../../BBBio_lib/BBBiolib.h"
#define threshold 10

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
	int del;
	iolib_init();
	iolib_setdir(8,11, BBBIO_DIR_IN);
	iolib_setdir(8,12, BBBIO_DIR_IN);

	const float PWM_HZ = 100.0f;    /* 100 Hz */
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
	BBBIO_PWMSS_Setting(1, PWM_HZ ,0.1f , 15.0f);
	BBBIO_PWMSS_Setting(0, PWM_HZ ,15.0f , 0.1f);

	//WheeledA_init();
	WheeledB_init();
	WheeledA_init();

	BBBIO_ehrPWM_Enable(1);
	BBBIO_ehrPWM_Enable(0);
	while(1)
	{
		//BBBIO_PWMSS_Setting(BBBIO_PWMSS0, PWM_HZ ,duty_A , duty_B);
		//BBBIO_ehrPWM_Enable(1);
		//BBBIO_ehrPWM_Enable(BBBIO_PWMSS0);
		//BBBIO_ehrPWM_Enable(0);
		countA = 0;
		countB = 0;

		BBBIO_ehrPWM_Enable(1);
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
		}while(countA < threshold && countB < threshold);

		printf("%d\n%d\n",countA,countB);
		
		BBBIO_ehrPWM_Disable(BBBIO_PWMSS1);
		BBBIO_ehrPWM_Disable(BBBIO_PWMSS0);
		//A輪比B輪快
		if(countA > countB)
		{
			BBBIO_ehrPWM_Enable(BBBIO_PWMSS0);
			do{
				if (is_high(8,12))
				{
					tmpB = now_valueB;
					now_valueB = 1;
				}
				else
					if(is_low(8,12))
					{
						tmpB = now_valueB;
						now_valueB = 0;
					}
				if(tmpB == 0 && now_valueB == 1)
				{
					countB++;
				}
			}while(countB < threshold);
		}
		//B輪比A輪快
		else
			if(countB > countA)
			{
			BBBIO_ehrPWM_Enable(BBBIO_PWMSS1);
			do{
				if (is_high(8,11))
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
			if(tmpA == 0 && now_valueA == 1)
			{
				countA++;
			}
			}while(countA < threshold);
		}	
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


