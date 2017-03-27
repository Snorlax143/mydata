#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>

//======== L3G4200D =========
#define L3G_ADDR	0x69	//i2c
#define L3G_CTL_REG1	0x20
#define L3G_CTL_REG2	0x21
#define L3G_CTL_REG4	0x23	//DPS
#define L3G_STATUS_REG	0x27
#define L3G_X_L		0x28
#define L3G_X_H		0x29
#define L3G_Y_L		0x2A
#define L3G_Y_H		0x2B
#define L3G_Z_L		0x2C
#define L3G_Z_H		0x2D
#define FIX_TIME	100

//陀螺儀的degree per seconed範圍
/*
 * +- 200, 8.75mdps/digit	
 * +- 500, 17.5
 * +- 2000, 70
 */
#define GYRO_POS_LIMIT	500
#define GYRO_NEG_LIMIT	-500
#define dpsPerDigit	.0175f

#define GYRO_DIV	100	//加速度區間
short div_gyro_buf[3] = {0,0,0};		// (QS用)	gryo角速度 / GYRO_DIV = state區間
short goal_gyro[3] = {0,0,0};	// 目標加速度/GYRO_DIV 
short l3g_regulate_buf[3] = {0, 0, 0}; //靜態基準
float l3g_angle[3] = {0, 0, 0};	//目前傾斜角度	//comp	YZ面，XZ面，水平旋轉		
float l3g_angle2[3] = {0, 0, 0};	//目前傾斜角度2	//純陀螺儀累積，會有誤差，畫圖用
short acc_buf[3] = {0,0,0};	//目前的三軸數值	x,y,z
short gyro_buf[3] = {0,0,0};	//目前的加速度值	繞X軸,繞Y軸,z

float compAngle[3] = {0,0,0};	//互補濾波角度	{Axr,Ayr,Azr}

const float kp = 0.35;
const float ki = 0.2;
//const float kd = 0.2;
struct timeval last_tv;

float getTimeDif ()
{
	static unsigned long diff;
	struct timeval tv;
	float d;

	gettimeofday(&tv, NULL);


	diff = (1000000L * (tv.tv_sec - last_tv.tv_sec) + tv.tv_usec - last_tv.tv_usec) / 1000L;	
	d = diff/1000.0;


	printf("diff time: %.4f\n", d);
	last_tv = tv;

	return d;
}

void l3g_init (int file)
{
	char wbuf[2] =  {L3G_CTL_REG1,0x0f};	//100hz
	char wbuf2[2] =  {L3G_CTL_REG4,0x90};

	if (ioctl(file, I2C_SLAVE, L3G_ADDR) <0) {
		printf("i2c open error\n");
		return ;
	}
	//ODR:200hz, cut-off:12.5
	write(file, wbuf, 2);
	//block data update
	write(file, wbuf2, 2);
}

void l3g_regulate (int file)
{
	char read_start_buf[6] = {L3G_X_H, L3G_Y_H, L3G_Z_H, L3G_X_L, L3G_Y_L, L3G_Z_L};
	char read_buf[6] = {0,0,0,0,0,0};		//x_h, y_h, z_h, x_l, y_l, z_l;
	char hi_buf, lo_buf, i, j;
	short	status;
	short	l3g_last_dps[3] = {0,0,0};	//前一次的角速度值
	short	deltaGyro[3] = {0,0,0};	
	short	gyroDPS[3] = {0,0,0};	//瞬時角速度

	if (ioctl(file, I2C_SLAVE, L3G_ADDR) <0) 
	{
		printf("i2c open error\n");
		return ;
	}

	for (i=0;i<FIX_TIME;i++) 
	{
		//陀螺儀基準
		for (j=0;j<3;j++) 
		{
			write(file, &read_start_buf[j], 1);
			read(file,&hi_buf,1);
			write(file, &read_start_buf[j+3], 1);
			read(file,&lo_buf,1);
			status = (hi_buf << 8) | lo_buf;

			deltaGyro[j] = status - l3g_last_dps[j];
			gyroDPS[j] = (short)(deltaGyro[j] * dpsPerDigit);
			//printf("\nt1:%d,  t2:%f\n",gyroDPS[j],deltaGyro[j] * dpsPerDigit);
			gyroDPS[j] = abs(gyroDPS[j]);	//因為計算誤差，+-一視同仁
			if (gyroDPS[j] > GYRO_POS_LIMIT ) gyroDPS[j] = GYRO_POS_LIMIT;
			//else if (gyroDPS[j] < GYRO_NEG_LIMIT ) gyroDPS[j] = GYRO_NEG_LIMIT;
			l3g_regulate_buf[j] += gyroDPS[j];
			l3g_last_dps[j] = status;
		}
		usleep(10000);
	}

	for (i=0;i<3;i++) 
	{
		l3g_regulate_buf[i] /= FIX_TIME;
		goal_gyro[i] = l3g_regulate_buf[i]/GYRO_DIV;
	}
	//printf("目標狀態: %d, %d, %d \n",goal_gyro[0],goal_gyro[1],goal_gyro[2]);
}

void l3g_ctl (int file, float timeDiff)
{
	char read_start_buf[6] = {L3G_X_H, L3G_Y_H, L3G_Z_H, L3G_X_L, L3G_Y_L, L3G_Z_L};
	char hi_buf, lo_buf;
	char i;
	short	x,y,z;
	short	status;
	short	gyroDPS[3] = {0,0,0};		//瞬時角速度
	short	deltaGyro[3] = {0,0,0};		//扣除前一次，本次檢測時間內的角速度差
	float	gyroDPSDelta[3] = {0,0,0};	//時間內移動的角度
	short	l3g_last_dps[3] = {0,0,0};	//前一次的角速度值

	if (ioctl(file, I2C_SLAVE, L3G_ADDR) <0) 
	{
		printf("i2c open error\n");
		return ;
	}
	/*
	//應該要檢查狀態才撈數值，不過不理他 >.0
	while (1) {
	read_start_buf = L3G_STATUS_REG;
	write(file, &read_start_buf, 1);
	read(file,&status,1);
	printf("123: %x\n",status);
	if ( status == 0x08) break;
	}
	 */

	//
	// 0:圍繞X軸(YZ平面傾角)，1:圍繞Y軸(XZ平面傾角)，2:圍繞Z軸(水平旋轉角度)

	for (i=0;i<3;i++) 
	{
		write(file, &read_start_buf[i], 1);
		read(file,&hi_buf,1);
		write(file, &read_start_buf[i+3], 1);
		read(file,&lo_buf,1);
		status = (hi_buf << 8) | lo_buf;
		deltaGyro[i] = status - l3g_last_dps[i];	//與前次相減
		gyroDPS[i] = (short)(deltaGyro[i] * dpsPerDigit);	//帶上精度
		if (gyroDPS[i] > GYRO_POS_LIMIT ) gyroDPS[i] = GYRO_POS_LIMIT;		//極值過濾
		else if (gyroDPS[i] < GYRO_NEG_LIMIT) gyroDPS[i] = GYRO_NEG_LIMIT;

		if (abs(gyroDPS[i]) <= l3g_regulate_buf[i]) gyroDPS[i] = 0; //低於平均誤差，視為沒變

		//偷懶拉
		if (i==2) gyroDPS[i] = 0;


		gyroDPSDelta[i] = (float)(gyroDPS[i] * timeDiff);
		l3g_angle[i] +=gyroDPSDelta[i];	//累積移動角度，互補濾波
		l3g_angle2[i] +=gyroDPSDelta[i];	//累積移動角度

		l3g_last_dps[i] = status;	//記錄本次，供下次算差距

		gyro_buf[i] = gyroDPS[i];


		//區間處理
		if (gyroDPS[i] == 0) div_gyro_buf[i] = 0;
		else if ((gyroDPS[i] > 0) && (gyroDPS[i] < GYRO_DIV)) div_gyro_buf[i] = (gyroDPS[i]/GYRO_DIV) +1;
		else if ((gyroDPS[i] < 0) && ( abs(gyroDPS[i]) < GYRO_DIV)) div_gyro_buf[i] = (gyroDPS[i]/GYRO_DIV) -1;
		else  div_gyro_buf[i] = gyroDPS[i]/GYRO_DIV;
	}

	//	printf("角速度 [0]:%d, [1]:%d,[2]:%d\n",gyro_buf[0],gyro_buf[1],gyro_buf[2]);

		printf("\n=====\n瞬時Y角速度: %d, angle:%4lf \n",gyro_buf[1], l3g_angle2[0]);//div_gyro_buf[1]);	//gyro只看Y  so far 03/30

}

typedef struct PID
{
	float target;     //目標
	float Proportion; //kp
	float Integral;   //ki
	//float Derivative; //kd
	float pre2Err;    //pre two step err
	float preErr;     //pre err
}PID;
float PIDCale(PID *pid,float input)
{
	float Err,pErr,dErr,sum,dU;
	Err = pid->target - input;  //當前誤差
	pErr = Err - pid->preErr;   //比例項增量式誤差
	/*kp * delta e(k) + ki * e(k) + kd[delta e(k) - delta e(k-1)]; "delta e(k) = e(k) - e(k-1)"*/
	sum = pid->Proportion * pErr + pid->Integral * Err;// + p->Derivative * (pErr * (p->preErr - p->pre2Err));
	//dErr = Err - 2 * p->preErr + p->pre2Err;  //微分項增量式誤差
	//dU = p->Proportion * pErr + p->Derivative * dErr+p->Integral * Err;  //控制量增量
	pid->pre2Err = pid->preErr;
	pid->preErr = Err;
	return sum;
}

void PIDInit(PID pid)
{
	memset(&pid,0,sizeof(PID));  //初始化
}

int kbhit(void)  
{  
	struct termios oldt, newt;  
	int ch;  
	int oldf;  
	tcgetattr(STDIN_FILENO, &oldt);  
	newt = oldt;  
	newt.c_lflag &= ~(ICANON | ECHO);  
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);  
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);  
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);  
	ch = getchar();  
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  
	fcntl(STDIN_FILENO, F_SETFL, oldf);  
	if(ch != EOF)  
	{  
		ungetc(ch, stdin);  
		return ch;  
	}  
	return 0;  
}  

void main()
{
	//int outA[100];
	int i = 0;
	PID *pid;
	struct PID p;
	PIDInit(p);
	p.Proportion = kp;
	p.Integral = ki;
	char ch;
	int file;
	float timeDiff;
	if ((file = open("/dev/i2c-1",O_RDWR)) <0 ) 
	{
		printf("file open error\n");
		return ;
	}

	l3g_init(file);


	while(1)
	{
		while(!kbhit())
		{
			//陀螺儀取靜態基準值
			//l3g_regulate(file);
			timeDiff = getTimeDif();
			l3g_ctl(file, timeDiff);
		}
		//printf("!!!");
		//puts("Press a key!");  
		ch = kbhit();
		ch = getchar();
		//printf("You pressed '%c'!\n", getchar());	
		switch(ch)
		{
			case 'w':
				printf("is work\n");
				break;

			case 's':
				printf("still work\n");
				break;

		}
		//sleep(5); 
	} 
}
