//以state為主，皆以 角度/ADX_DIV 為主的區間為主，再無單一角度概念
//加入互補濾波修正
//修改reward規則

//互補濾波取樣時間待調整
//將PWM delay 與 sensor delay錯開

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h> 
#include "../BBBIOlib/BBBio_lib/BBBiolib.h"

//======== L3G4200D =========
#define L3G_ADDR		0x69	//i2c
#define L3G_CTL_REG1	0x20
#define L3G_CTL_REG2	0x21
#define L3G_CTL_REG4	0x23	//DPS
#define L3G_STATUS_REG	0x27
#define L3G_X_L			0x28
#define L3G_X_H			0x29
#define L3G_Y_L			0x2A
#define L3G_Y_H			0x2B
#define L3G_Z_L			0x2C
#define L3G_Z_H			0x2D

//======== ADXL345 =========
#define ADX_ADDR		0x53	//i2c
#define	ADX_POW			0x2D
#define ADX_DATA_F		0x31
#define ADX_FIFO_CTL	0x38
#define	ADX_BW			0x2C
#define	ADX_X_L			0x32
#define	ADX_X_H			0x33
#define	ADX_Y_L			0x34
#define	ADX_Y_H			0x35
#define	ADX_Z_L			0x36
#define	ADX_Z_H			0x37
#define ADX_SOURCE		0x30
#define FIX_TIME		100

//陀螺儀的degree per seconed範圍
/*
 * +- 200, 8.75mdps/digit	
 * +- 500, 17.5
 * +- 2000, 70
 */
#define GYRO_POS_LIMIT	500
#define GYRO_NEG_LIMIT	-500
#define dpsPerDigit		.0175f

//======= Qlearning ========
#define NO_POLICY	0	//是否不執行policy
#define ADX_DIV		3	//計算目標差的block單位，正常以1度計算，為放寬區間將數值放大
#define ANGLE_GOOD	1	//!!誤差值!!	(單位為 ADX_DIV)
#define ANGLE_BAD	1	//兩次誤差的差  (單位為 ADX_DIV)
#define ANGLE_WORSE	5
#define ADX_MAX		180	//角度輸出範圍 0~180
#define ADX_BUF_SIZE ((ADX_MAX*2/ADX_DIV)+1)	//一個軸向的切割數
#define ADX_ALL_SIZE ADX_BUF_SIZE
#define ADX_SCALE	0.0039f

#define GYRO_DIV	100	//加速度區間
#define GOOD_GY		1
#define GYRO_BUF_SIZE ((GYRO_POS_LIMIT*2/GYRO_DIV)+1)
#define GYRO_ALL_SIZE (((GYRO_POS_LIMIT*2/GYRO_DIV)+1)*((GYRO_POS_LIMIT*2/GYRO_DIV)+1))
#define G1_MEM	(GYRO_BUF_SIZE*ADX_ALL_SIZE*PWM_SET)
#define MEM_SIZE	(GYRO_ALL_SIZE*ADX_ALL_SIZE*PWM_SET)

#define PWM_SET		10	//PWM輸出設定 正反轉10%~100%  +0%
#define PWM_BASE	3.0f

//#define ALPHA	0.8f
#define	GAMMA	0.95f
//避免陷入邊緣狀態，使用e-greedy跳脫
#define EGREEDY	5	//100%, 10 = 10%

//======== PID ====================
#define checkpoint 10
#define distance 200
#define turn_degree 45
#define PWM 10.0f

//========average filter===========
#define lpt 8
short lo_buf[lpt][3] = {{0,0,0},
						{0,0,0},
						{0,0,0},
						{0,0,0},
						{0,0,0}};

//=======Global value==============
float ALPHA=1.0;

void do_action (int act);
void stop_wheele();
void save();
void load();
void acc_degree_count (short *buf , short *r_buf);
int choose_act (short *arg_a, short *arg_g, int greedy_flag, int print_flag);

short l3g_regulate_buf[3] = {0, 0, 0}; //靜態基準
float l3g_angle[3] = {0, 0, 0};	//目前傾斜角度	//comp	YZ面，XZ面，水平旋轉 		
float l3g_angle2[3] = {0, 0, 0};	//目前傾斜角度2	//純陀螺儀累積，會有誤差，畫圖用
short acc_buf[3] = {0,0,0};	//目前的三軸數值	x,y,z
short gyro_buf[3] = {0,0,0};	//目前的加速度值	繞X軸,繞Y軸,z

float compAngle[3] = {0,0,0};	//互補濾波角度	{Axr,Ayr,Azr}

//state區間
short div_angle_buf[3] = {0,0,0};	//  (QS用)  將三軸轉換成實際夾角 / ADX_DIV = state區間 {X,Y,傾角}
short angle_buf[3] = {0,0,0};	//三軸回傳出夾角
short div_gyro_buf[3] = {0,0,0};		// (QS用)	gryo角速度 / GYRO_DIV = state區間
short draw_buf[3] = {0,0,0};

short acc_target[3] = {0,0,0};
short goal_acc[3] = {0,0,0};		// 目標三軸角度/ADX_DIV = 目標state
short goal_gyro[3] = {0,0,0};	// 目標加速度/GYRO_DIV 

struct timeval last_tv;
float *action_pair;
float reward_list[] = {100.0, 1.0, -1.0, 0.0,-10}; // good, bad

int start_flag = 0;

const float kp = 0.2;
const float ki = 0.0;
const float kd = 0.5;
struct timespec start;
struct timespec end;
struct timespec diff;

struct timeval last_tv;
//====================================

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

int average_filter(int i)
{
	int j = lpt - 1;
	short result = 0;
	//lo_buf[j][i] = acc_buf[i];
	while(j--)
	{
		result += lo_buf[j][i];
		lo_buf[j][i] = lo_buf[j-1][i];
	}
	lo_buf[0][i] = acc_buf[i];
	return result / lpt;
}

void complementary_filter()
{
	short tmp;
	int i=0;

	// angle_buf : 已轉換為實際角度
	//互補濾波 XZ相傾角(整合為翻仰角)
	//以三軸角度區分方向，修正翻轉角度
	//三軸回傳的角度沒有正負之分，而陀螺儀累積的有
	//不以90度為中置基準，以初始靜置的X軸為主
	if (angle_buf[0]>acc_target[0]) tmp = (float)angle_buf[2];	//後仰
	else tmp = -1.0 * (float)angle_buf[2];	//前傾

	compAngle[2] = (0.95 *l3g_angle[1] + 0.05*tmp );
	l3g_angle[1] = compAngle[2];	//儲存回去，l3g_ctl下一次用
	//互補濾波 XR相傾角(整合為翻仰角)
	//	compAngle[0] = compAngle[2] + acc_target[0] ;
	compAngle[0] = 90.0 - compAngle[2] ;
	if ( compAngle[0] < 0) compAngle[0] = 0;
	else if (compAngle[0] > 180) compAngle[0] = 180;
	//互補濾波 YR相傾角(整合為翻仰角)
	tmp = (float)angle_buf[1];	//右翻
	compAngle[1] = (0.28 *(l3g_angle[0]) + 0.72*tmp );
	l3g_angle[0] = compAngle[1];	//儲存回去，l3g_ctl下一次用

	//互補後
	//	printf("target_buf:  %d, %d, %d\n",acc_target[0],acc_target[1],acc_target[2]);	//要跟未互補一樣 
	//	printf("(互補後角度)Axr:%.2f, Ayr:%.2f, Azr:%.2f\n",compAngle[0],compAngle[1],compAngle[2]);

	//區間化
	//	printf("acc_div(div_angle_buf)\n");
	for(i=0;i<3;i++) 
	{
		div_angle_buf[i] = (short)(compAngle[i]/ADX_DIV);
		//		printf("[%d]:%d  ",i,div_angle_buf[i]);
	}
}

//timeval.tv_sec 秒, .tv_usec微秒
float getTimeDif ()
{
	static unsigned long diff;
	struct timeval tv;
	float d;

	gettimeofday(&tv, NULL);


	diff = (1000000L * (tv.tv_sec - last_tv.tv_sec) + tv.tv_usec - last_tv.tv_usec) / 1000L;	
	d = diff/1000.0;


	//printf("diff time: %.4f\n", d);
	last_tv = tv;

	return d;
}

//計算三個軸向的夾角
void acc_degree_count (short *buf , short *r_buf)
{
	double R,Axr,Ayr,Azr,Rx,Ry,Rz;


	Rx = buf[0] * ADX_SCALE;
	Ry = buf[1] * ADX_SCALE;
	Rz = buf[2] * ADX_SCALE;

	//測試，無視水平
	Ry = 0;


	R = sqrt(pow(Rx,2)+pow(Ry,2)+pow(Rz,2));

	Axr = acos(Rx/R) * (180.0/3.14);
	Ayr = acos(Ry/R) * (180.0/3.14);
	Azr = acos(Rz/R) * (180.0/3.14);

	r_buf[0] = (short)Axr;
	r_buf[1] = (short)Ayr;
	r_buf[2] = (short)Azr;

	//	printf("(未互補角度)Axr:%d, Ayr:%d, Azr:%d\n",r_buf[0],r_buf[1],r_buf[2]);

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

//抓5次平均誤差為修正量
void l3g_regulate (int file)
{
	char read_start_buf[6] = {L3G_X_H, L3G_Y_H, L3G_Z_H, L3G_X_L, L3G_Y_L, L3G_Z_L};
	char read_buf[6] = {0,0,0,0,0,0};		//x_h, y_h, z_h, x_l, y_l, z_l;
	char hi_buf, lo_buf, i, j;
	short	status;
	short	l3g_last_dps[3] = {0,0,0};	//前一次的角速度值
	short	deltaGyro[3] = {0,0,0};	
	short	gyroDPS[3] = {0,0,0};	//瞬時角速度

	if (ioctl(file, I2C_SLAVE, L3G_ADDR) <0) {
		printf("i2c open error\n");
		return ;
	}



	for (i=0;i<FIX_TIME;i++) {
		//陀螺儀基準
		for (j=0;j<3;j++) {
			write(file, &read_start_buf[j], 1);
			read(file,&hi_buf,1);
			write(file, &read_start_buf[j+3], 1);
			read(file,&lo_buf,1);
			status = (hi_buf << 8) | lo_buf;

			deltaGyro[j] = status - l3g_last_dps[j];
			gyroDPS[j] = (short)(deltaGyro[j] * dpsPerDigit);
			//			printf("\nt1:%d,  t2:%f\n",gyroDPS[j],deltaGyro[j] * dpsPerDigit);
			gyroDPS[j] = abs(gyroDPS[j]);	//因為計算誤差，+-一視同仁
			if (gyroDPS[j] > GYRO_POS_LIMIT ) gyroDPS[j] = GYRO_POS_LIMIT;
			//else if (gyroDPS[j] < GYRO_NEG_LIMIT ) gyroDPS[j] = GYRO_NEG_LIMIT;
			l3g_regulate_buf[j] += gyroDPS[j];
			l3g_last_dps[j] = status;
		}
		usleep(20000);
	}

	for (i=0;i<3;i++) {
		l3g_regulate_buf[i] /= FIX_TIME;
		goal_gyro[i] = l3g_regulate_buf[i]/GYRO_DIV;
	}
	printf("目標狀態: %d, %d, %d \n",goal_gyro[0],goal_gyro[1],goal_gyro[2]);
}

void l3g_ctl (int file, float timeDiff)
{
	char read_start_buf[6] = {L3G_X_H, L3G_Y_H, L3G_Z_H, L3G_X_L, L3G_Y_L, L3G_Z_L};
	char hi_buf, lo_buf;
	char i,j;
	short	x,y,z;
	short	status;
	short	gyroDPS[3] = {0,0,0};	//瞬時角速度
	short	deltaGyro[3] = {0,0,0};		//扣除前一次，本次檢測時間內的角速度差
	float	gyroDPSDelta[3] = {0,0,0};	//時間內移動的角度
	short	l3g_last_dps[3] = {0,0,0};	//前一次的角速度值

	if (ioctl(file, I2C_SLAVE, L3G_ADDR) <0) {
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

	for (i=0;i<3;i++) {
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
		//if (i==2) gyroDPS[i] = 0;


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

	//	printf("\n=====\n瞬時Y角速度: %d, div:%d \n",gyro_buf[2], div_gyro_buf[2]);	//gyro只看Z  so far 03/30

}

void adx_init (int file)
{
	char wbuf[2] =  {ADX_POW,0x00};
	char wbuf2[2] =  {ADX_DATA_F,0x0B};
	char wbuf3[2] =  {ADX_FIFO_CTL,0x00};
	char wbuf4[2] =  {ADX_BW,0x0A};
	char wbuf5[2] =  {ADX_POW,0x08};

	if (ioctl(file, I2C_SLAVE, ADX_ADDR) <0) {
		printf("i2c open error\n");
		return ;
	}

	//init
	write(file, wbuf, 2);
	write(file, wbuf2, 2);
	write(file, wbuf3, 2);
	write(file, wbuf4, 2);
	write(file, wbuf5, 2);

}

void adx_regulate (int file)
{
	char read_start_buf[6] = {ADX_X_H, ADX_Y_H, ADX_Z_H, ADX_X_L, ADX_Y_L, ADX_Z_L};
	char i,j, hi_buf, lo_buf;
	short status,tmp_buf[3]={0,0,0};
	//	short acc_target[3] = {0,0,0};
	short tmp_angle[3]={0,0,0}, sum_buf[3]={0,0,0};
	float tmp_comp[3]={0,0,0};
	short tmp;

	if (ioctl(file, I2C_SLAVE, ADX_ADDR) <0) 
	{
		printf("i2c open error\n");
		return ;
	}

	for(j=0;j<FIX_TIME;j++){
		for(i=0;i<3;i++) {
			write(file, &read_start_buf[i], 1);
			read(file,&hi_buf,1);
			write(file, &read_start_buf[i+3], 1);
			read(file,&lo_buf,1);
			status = (hi_buf << 8) | lo_buf;
			tmp_buf[i] = status;
		}
		//角度化		
		acc_degree_count(tmp_buf,tmp_angle);
		//互補濾波
		tmp_comp[2] = (0.99*tmp_comp[2] + 0.01* (float)tmp_angle[2]);
		//		tmp_comp[2] = (0.98*tmp_comp[2] + 0.02* (float)tmp_angle[2]);
		tmp_comp[1] = (0.38*tmp_comp[1] + 0.62*(float)tmp_angle[1]);
		tmp_comp[0] = 90.0 - tmp_comp[2];

		usleep(10000);
	}

	//以互補濾波取樣一段時候，取最後的穩定值為靜置基準


	printf("目標三軸: \n");
	for(j=0;j<3;j++) {
		acc_target[j] = (short)tmp_comp[j];
		printf("acc_target[%d]:%d  ",j,acc_target[j]);
		goal_acc[j] = acc_target[j]/ADX_DIV;
	}

	printf("\n");
}



void adx_ctl (int file)
{
	char read_start_buf[6] = {ADX_X_H, ADX_Y_H, ADX_Z_H, ADX_X_L, ADX_Y_L, ADX_Z_L};
	char i, ok_buf;
	char hi_buf,lo_buf;
	short x,y,z,ready;
	short status;
	int j = 0;	
	short tmp;
	//	float x2,y2,z2;

	//float scale = 0.0039;	//全解析度模式 輸出值的比例為 3.9mg/LSB
	//為了加速計算，就不額外作正確數值調整，以原有輸出對照即可。
	int scale = 1;

	if (ioctl(file, I2C_SLAVE, ADX_ADDR) <0) {
		printf("i2c open error\n");
		return ;
	}
	//for(j = 0;j <= 300;j++)
	//{
	ready = ADX_SOURCE;
	write(file,&ready,1);
	read(file,&ok_buf,1);
	if ( (ok_buf>>7)== 1 ) 
	{
		for(i = 0; i < 3;i++) 
		{
			write(file, &read_start_buf[i], 1);
			read(file,&hi_buf,1);
			write(file, &read_start_buf[i+3], 1);
			read(file,&lo_buf,1);
			status = (hi_buf << 8) | lo_buf;

			acc_buf[i] = status;
			acc_buf[i]	= average_filter(i);
			draw_buf[i] = status;
		}
	}
	write(file,&ready,1);
	read(file,&ok_buf,1);

	//角度化
	acc_degree_count(acc_buf,angle_buf);
}

//delay = 10^-6 sec
void right_backward (const float duty, const float pwm_hz)
{
	BBBIO_PWMSS_Setting(BBBIO_PWMSS0, pwm_hz, 0.1f, duty);
	BBBIO_ehrPWM_Enable(BBBIO_PWMSS0);
}

void right_forward (const float duty, const float pwm_hz)
{
	BBBIO_PWMSS_Setting(BBBIO_PWMSS0, pwm_hz, duty, 0.1f);
	BBBIO_ehrPWM_Enable(BBBIO_PWMSS0);
}

//s1
void left_backward (const float duty, const float pwm_hz)
{
	BBBIO_PWMSS_Setting(BBBIO_PWMSS1, pwm_hz, duty, 0.1f);
	BBBIO_ehrPWM_Enable(BBBIO_PWMSS1);
}

void left_forward (const float duty, const float pwm_hz)
{
	BBBIO_PWMSS_Setting(BBBIO_PWMSS1, pwm_hz, 0.1f, duty);
	BBBIO_ehrPWM_Enable(BBBIO_PWMSS1);
}

void state_init ()
{
	action_pair = (float*)malloc(sizeof(float)*MEM_SIZE);
	printf("size: %d\n",MEM_SIZE);
	memset(action_pair,0.0,sizeof(float)*MEM_SIZE);
}

//傳入 div_angle_buf, div_gyro_buf。
void update_qs (short *last_acc, short *last_gyro, short *this_acc, short *this_gyro, int last_act)
{
	float qsa, old_qsa, reward=0.0;
	int big_act,i;
	int old_gyrofix_z, gyrofix_z, old_gyrofix_x, gyrofix_x, accfix_z, old_accfix_z;
	short old_acc_x,  old_acc_z, old_gyro_x, old_gyro_z, acc_x, acc_z, gyro_x, gyro_z;
	short *et_acc_old, *et_acc_now, *et_gyro_old, *et_gyro_now;


	et_acc_old = malloc(sizeof(short)*3);
	memset(et_acc_old, 0, sizeof(short)*3);
	et_acc_now = malloc(sizeof(short)*3);
	memset(et_acc_now, 0, sizeof(short)*3);
	et_gyro_old = malloc(sizeof(short)*3);
	memset(et_gyro_old, 0, sizeof(short)*3);
	et_gyro_now = malloc(sizeof(short)*3);
	memset(et_gyro_now, 0, sizeof(short)*3);

	//把舊qsa抓出來
	last_act = abs(last_act);
	old_acc_x = last_acc[0];
	old_acc_z = last_acc[2];

	old_gyro_x = last_gyro[1];
	old_gyro_z = last_gyro[2];

	//RxGyro offset
	if (old_gyro_x < 0) {
		old_gyrofix_x = GYRO_BUF_SIZE/2;
		old_gyro_x = abs(old_gyro_x);
	} else old_gyrofix_x = 0;
	if (old_gyro_z < 0) {
		old_gyrofix_z = GYRO_BUF_SIZE/2;
		old_gyro_z = abs(old_gyro_z);
	} else old_gyrofix_z = 0;
	//old acc offset負值補正
	if (old_acc_z < 0) {
		old_accfix_z = ADX_BUF_SIZE/2;
		old_acc_z = abs(old_acc_z);
	} else old_accfix_z = 0;


	acc_x = this_acc[0];	//不屬於state，但用來判斷區域
	acc_z = this_acc[2];
	gyro_x = this_gyro[1];
	gyro_z = this_gyro[2];
	big_act = choose_act(this_acc, this_gyro,0,0);	//不考慮greedy
	big_act = abs(big_act);

	//	printf("State: { %d, %d, %d }\n",acc_z,gyro_x,gyro_z);

	//傾角加速度負值offset補正
	if (gyro_x < 0) {
		gyrofix_x = GYRO_BUF_SIZE/2;
		gyro_x = abs(gyro_x);
	} else gyrofix_x = 0;

	if (gyro_z < 0) {
		gyrofix_z = GYRO_BUF_SIZE/2;
		gyro_z = abs(gyro_z);
	} else gyrofix_z = 0;
	//acc offset負值補正
	if (acc_z < 0) {
		accfix_z = ADX_BUF_SIZE/2;
		acc_z = abs(acc_z);
	} else accfix_z = 0;



	//	printf("ABS State: { %d, %d, %d }\n",acc_z,gyro_x,gyro_z);

	//算誤差區間
	for (i=0;i<3;i++) {
		et_acc_old[i] = goal_acc[i] - abs(last_acc[i]);
		et_gyro_old[i] = goal_gyro[i] - abs(last_gyro[i]);
		et_acc_now[i] = goal_acc[i] - abs(this_acc[i]);
		et_gyro_now[i] = goal_gyro[i] - abs(this_gyro[i]);
		//		printf("[%d]ETacc:%d, et_gyro:%d\n",i,et_acc_now[i],et_gyro_now[i]);
	}

	/*
	   if ( (abs(et_acc_now[2]) < ANGLE_GOOD) &&  (abs(et_gyro_now[1]) <= GOOD_GY ) ) {
	   stop_wheele();
	   start_flag = 0; //結束這次的epsiode
	   }
	   if ( abs(et_acc_now[2]) < ANGLE_WORSE ) reward  = reward_list[3]; 
	   else reward  = reward_list[2];
	 */


	//給獎勵
	if (( abs(et_acc_now[2]) == 0 ) &&  (abs(et_gyro_now[1]) <= GOOD_GY ) ) {
		//完美狀態
		reward = reward_list[0];
		//		printf("\n         GOOOOOOOD \n");
		stop_wheele();
		start_flag = 0;	//結束這次的epsiode
	} else if ( abs(et_acc_now[2]) >= abs(et_acc_old[2]) ) {
		//	} else if ( (abs(et_acc_now[2]) > abs(et_acc_old[2])) && (abs(et_acc_now[2] - et_acc_old[2]) >= ANGLE_BAD) ) {
		//誤差比前次大
		//根據是否不同區進行後處理
		if ( ((acc_x > goal_acc[0]) && (old_acc_x > goal_acc[0])) || ((acc_x < goal_acc[0]) && (old_acc_x < goal_acc[0]))  ) {
			reward = reward_list[2];
			//同一側,不應該
			//			printf("                        N >Old  same side\n");

			//s(t) 所有小於last_act的都給bad
			qsa = *(action_pair+(int)((gyro_x+gyrofix_x)*G1_MEM + (gyro_z+gyrofix_z)*ADX_ALL_SIZE*PWM_SET + (acc_z+accfix_z)*PWM_SET + big_act));
			for(i=0;i<last_act;i++) {
				old_qsa = *(action_pair+(int)((old_gyro_x+old_gyrofix_x)*G1_MEM +(old_gyro_z+old_gyrofix_z)*ADX_ALL_SIZE*PWM_SET + (old_acc_z+old_accfix_z)*PWM_SET + i));
				old_qsa = old_qsa + ALPHA*(reward + GAMMA*qsa - old_qsa);
				*(action_pair+(int)((old_gyro_x+old_gyrofix_x)*G1_MEM +(old_gyro_z+old_gyrofix_z)*ADX_ALL_SIZE*PWM_SET + (old_acc_z+old_accfix_z)*PWM_SET + i)) = old_qsa;
			}

		} else {
			//不同側,翻過頭 不應該
			//			printf("                        N >O  other side\n");
			reward = reward_list[4];
			//			printf("cross side\n");
			//s(t) 所有大於last_act的都給bad
			qsa = *(action_pair+(int)((gyro_x+gyrofix_x)*G1_MEM +(gyro_z+gyrofix_z)*ADX_ALL_SIZE*PWM_SET + (acc_z+accfix_z)*PWM_SET + big_act));
			for(i=(last_act+1);i<PWM_SET;i++) {
				old_qsa = *(action_pair+(int)((old_gyro_x+old_gyrofix_x)*G1_MEM +(old_gyro_z+old_gyrofix_z)*ADX_ALL_SIZE*PWM_SET + (old_acc_z+old_accfix_z)*PWM_SET + i));
				old_qsa = old_qsa + ALPHA*(reward + GAMMA*qsa - old_qsa);
				*(action_pair+(int)((old_gyro_x+old_gyrofix_x)*G1_MEM +(old_gyro_z+old_gyrofix_z)*ADX_ALL_SIZE*PWM_SET + (old_acc_z+old_accfix_z)*PWM_SET + i)) = old_qsa;
			}

		}
} else if ( abs(et_acc_now[2]) < abs(et_acc_old[2])  ) {
	//	} else if ( (abs(et_acc_now[2]) < abs(et_acc_old[2])) && (abs(et_acc_now[2] - et_acc_old[2]) >= ANGLE_BAD)  ) {
	//誤差比前次少
if ( ((acc_x > goal_acc[0]) && (old_acc_x > goal_acc[0])) || ((acc_x < goal_acc[0]) && (old_acc_x < goal_acc[0]))  ) {
	//			printf("                        N <O  same side\n");
	//do nothing
	reward = reward_list[3];
} else {
	//不同側
	//			printf("                        N <O  other side\n");
	reward = reward_list[3];
}
} else {
	reward = reward_list[2];
	//		printf("\n[0]\n\n");
}	

old_qsa = *(action_pair+(int)((old_gyro_x+old_gyrofix_x)*G1_MEM +(old_gyro_z+old_gyrofix_z)*ADX_ALL_SIZE*PWM_SET +  (old_acc_z+old_accfix_z)*PWM_SET + last_act));
//qsa'  找新狀態的最大QSA值，更新前一動的qsa
qsa = *(action_pair+(int)((gyro_x+gyrofix_x)*G1_MEM +(gyro_z+gyrofix_z)*ADX_ALL_SIZE*PWM_SET + (acc_z+accfix_z)*PWM_SET + big_act));
//更新
old_qsa = old_qsa + ALPHA*(reward + GAMMA*qsa - old_qsa);
*(action_pair+(int)((old_gyro_x+old_gyrofix_x)*G1_MEM +(old_gyro_z+old_gyrofix_z)*ADX_ALL_SIZE*PWM_SET + (old_acc_z+old_accfix_z)*PWM_SET + last_act)) = old_qsa;
//	printf("new_qsa :%.2f\n ",old_qsa);


}

//選擇動作
//return : (回傳+1)*10% = pwm output
// 傳入arg: div_angle_buf, div_gyro_buf
int choose_act (short *arg_a, short *arg_g, int greedy_flag, int print_flag)
{

	float max_action,tmp;
	int act,i,propor;
	int gyrofix_z,gyrofix_x,accfix_z;
	short gyro_x,gyro_z,acc_x,acc_y,acc_z;
	short *et_acc_now, *et_gyro_now;


	et_acc_now = malloc(sizeof(short)*3);
	memset(et_acc_now, 0, sizeof(short)*3);
	et_gyro_now = malloc(sizeof(short)*3);
	memset(et_gyro_now, 0, sizeof(short)*3);

	acc_x = arg_a[0];
	acc_z = arg_a[2];
	gyro_x = arg_g[1];
	gyro_z = arg_g[2];


	for (i=0;i<3;i++) {
		et_acc_now[i] = goal_acc[i] - arg_a[i];
		et_gyro_now[i] = goal_gyro[i] - arg_g[i];
	}


	//傾角加速度負值offset補正
	if (gyro_x < 0) {
		gyrofix_x = GYRO_BUF_SIZE/2;
		gyro_x = abs(gyro_x);
	} else gyrofix_x = 0;	

	if (gyro_z < 0) {
		gyrofix_z = GYRO_BUF_SIZE/2;
		gyro_z = abs(gyro_z);
	} else gyrofix_z = 0;
	//acc offset負值補正
	if (acc_z < 0) {
		accfix_z = ADX_BUF_SIZE/2;
		acc_z = abs(acc_z);
	} else accfix_z = 0;


	//給予比例初始
	propor = (abs(et_acc_now[2]) * 1.5 );		//    zone / 20 acts

	//	printf("[prop: %d]\n",propor);

	if (propor >= PWM_SET) propor = PWM_SET-1;
	max_action = *(action_pair+(int)((gyro_x+gyrofix_x)*G1_MEM +(gyro_z+gyrofix_z)*ADX_ALL_SIZE*PWM_SET + (acc_z+accfix_z)*PWM_SET +propor));
	act = propor;
	//	printf("Pact:%d\n",propor);

	if ( greedy_flag ) {
		//e-greedy
		if ((rand()%100+1) < greedy_flag ) 
		{
			act = rand()%PWM_SET;
			//			printf("greedy, act:%d\n",act);

		}
		else 
		{
			//MaxQs
			for (i=0;i<PWM_SET;i++) 
			{
				tmp = *(action_pair+(int)((gyro_x+gyrofix_x)*G1_MEM +(gyro_z+gyrofix_z)*ADX_ALL_SIZE*PWM_SET + (acc_z+accfix_z)*PWM_SET + i));
				if( NO_POLICY && (i==0) ) 
				{
					max_action = tmp; 
					act=0;
				} 
				else 
				{
					if (tmp > max_action) 
					{
						act = i;
						max_action = tmp;
					}
				}
				if(print_flag)	
				{
					printf("[%d] ",i);
					printf("\n<< max_action_qs: %f, this_qs: %f, act:%d >>\n",max_action, tmp, act);
				}
			}
		}
	} 
	else 
	{
		//MaxQs
		for (i=0;i<PWM_SET;i++) 
		{
			tmp = *(action_pair+(int)((gyro_x+gyrofix_x)*G1_MEM +(gyro_z+gyrofix_z)*ADX_ALL_SIZE*PWM_SET + (acc_z+accfix_z)*PWM_SET + i));
			if(NO_POLICY && (i==0)) 
			{
				max_action = tmp; 
				act=0;
			}
			else 
			{
				if (tmp > max_action) 
				{
					act = i;
					max_action = tmp;
				}
			}
			if(print_flag)	
			{
				printf("[%d] ",i);
				printf("\n<< max_action_qs: %f, this_qs: %f, act:%d >>\n",max_action, tmp, act);
			}

		}
	}
	//後仰
	if (acc_x < (acc_target[0]/ADX_DIV) ) act *= -1;	//單方向
	if (print_flag) printf("choose act: %d\n",act);

	return act;
}

void do_action (int act)
{
	float duty;

	printf("\n>>> act : %d<<<\n",act);
	if (act == 0) 
	{
		stop_wheele();
	} else if (act < 0) 
	{	//單方向
		//	if (act > 10) {
		act = abs(act);	//單方向
		//		duty = (act-10) * PWM_BASE; 
		duty = (float)(act+1) * PWM_BASE; //單方向
		right_forward(duty, 100.0f);
		left_forward(duty, 100.0f);
		printf("=====backward %.2f%=====\n\n",duty);
	} else 
	{
		duty = (float)(act+1) * PWM_BASE;
		right_backward(duty, 100.0f);
		left_backward(duty, 100.0f);
		printf("=====forward %.2f%=====\n\n",duty);
	}
	}

	void stop_wheele ()
	{
		BBBIO_ehrPWM_Disable(BBBIO_PWMSS0);
		BBBIO_ehrPWM_Disable(BBBIO_PWMSS1);
	}

	//將訓練完的action pair儲存起來
	void save ()
	{
		FILE *pFile;
		int i,j,k,m;
		float qs;


		if ((pFile = fopen("log/qs.txt","wb"))==NULL ) 
		{
			printf("log open error\n");
			return ;
		}

		//gyx
		for(i=0;i<GYRO_BUF_SIZE;i++) 
		{	
			//gyz
			for(j=0;j<GYRO_BUF_SIZE;j++) 
			{	
				//accz
				for(k=0;k<ADX_BUF_SIZE;k++) 
				{
					for(m=0;m<PWM_SET;m++) 
					{
						qs = *(action_pair+(int)(i*G1_MEM + j*ADX_ALL_SIZE*PWM_SET + k*PWM_SET + m));
						fprintf(pFile,"%.4f\n",qs);
					}
				}
			}
		}
		fclose(pFile);

	}

	void load ()
	{
		FILE *pFile;
		int i,j,k,m;
		float qs;

		if ((pFile = fopen("log/qs.txt","r"))==NULL ) 
		{
			printf("log open error\n");
			return ;
		}

		//gyx
		for(i=0;i<GYRO_BUF_SIZE;i++) 
		{	
			//gyz
			for(j=0;j<GYRO_BUF_SIZE;j++) 
			{	
				//accz
				for(k=0;k<ADX_BUF_SIZE;k++) 
				{
					for(m=0;m<PWM_SET;m++) 
					{
						if(fscanf(pFile,"%f",&qs) != EOF) 
						{
							*(action_pair+(int)(i*G1_MEM + j*ADX_ALL_SIZE*PWM_SET + k*PWM_SET + m)) = qs ;
						}
					}
				}
			}
		}
		fclose(pFile);
	}
	//PID direction controll
	void PID_dirctl()
	{
		const float PWM_HZ = 100.0f;    /* 100 Hz */
		float duty_A = PWM;					
		float duty_B = PWM;
		float out_A[distance];				//Encoder output 繪圖用
		float out_B[distance];
		float PWM_B[distance];				//B輪PWM變化 繪圖用
		float out_l3g[distance];			//角度變化 繪圖用

		float countA = 0,countB = 0;		//Encoder counter
		char now_valueA = 0, tmpA = 1;		
		char now_valueB = 0, tmpB = 1;
		float Pout = 0,pide = 0,pidg = 0;
		float time,used_time;
		int i = 0,count = 0;
		char command;
		struct PID p;
		int file;
		float timeDiff;

		if ((file = open("/dev/i2c-1",O_RDWR)) <0 ) 
		{
			printf("file open error\n");
			return ;
		}

		BBBIO_PWMSS_Setting(1, PWM_HZ ,0.1f , duty_A);
		BBBIO_PWMSS_Setting(0, PWM_HZ ,duty_B , 0.1f);

		PIDInit(p);
		p.Proportion = kp;
		p.Integral = ki;
		p.Derivative = kd;
		//printf("log1.5\n");
		WheeledB_init();
		WheeledA_init();
		l3g_init(file);
		//printf("log2\n");
		//printf("log3\n");
		//while(1)
		//{
		//printf("log4\n");	
		while(!kbhit())
		{
			//l3g_angle2[2] = 0;
			//timeDiff = getTimeDif();
			//l3g_ctl(file, timeDiff);
			return;
		}
		command = kbhit();
		command = getchar();

		switch(command)
		{
			case 'w':
				//printf("%c\n",command);
				BBBIO_PWMSS_Setting(1, PWM_HZ ,0.1f , duty_A);
				BBBIO_PWMSS_Setting(0, PWM_HZ ,duty_B , 0.1f);
				BBBIO_ehrPWM_Enable(1);
				BBBIO_ehrPWM_Enable(0);

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
						//iolib_delay_ms(10);
					}while(countA < checkpoint);// && countB < checkpoint);
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
					//printf("%f\n%f\n",countA,countB);	
					/*	
						p.target = (float)countA;
						Pout = PIDCale(&p,countB);
						duty_B += Pout;

						timeDiff = getTimeDif();
						l3g_ctl(file, timeDiff);
					 */

					p.target = (float)countA;
					pide = PIDCale(&p,countB);

					timeDiff = getTimeDif();
					l3g_ctl(file, timeDiff);
					p.target = 0.0;
					//if(gyro_buf[2] < 5 && gyro_buf[2] > -5)
					//	pidg = 0;
					//else

					//Gyro PID
					pidg = 0.05 * (PIDCale(&p,gyro_buf[2]));
					Pout = pide + pidg;
					duty_B += Pout;
					//printf("%f\n",pidg);	
					/*very importemt*/
					if(duty_B <= 0)
						duty_B = 0.1;
					if(duty_B > 100)
						duty_B = 100.0;
					//速度需慢慢提升
					   if(count % 50 == 0)
					   {
					   duty_A += 3;
					   }
					 
					BBBIO_PWMSS_Setting(1, PWM_HZ ,0.1f , duty_A);
					BBBIO_PWMSS_Setting(0, PWM_HZ ,duty_B , 0.1f);
					BBBIO_ehrPWM_Enable(1);
					BBBIO_ehrPWM_Enable(0);
					//printf("PWMA set to be %f%\n",duty_A);
					printf("PWMB set to be %f%\n",duty_B);
					out_A[count] = countA;
					out_B[count] = countB;
					PWM_B[count] = duty_B;
					out_l3g[count] = l3g_angle2[2];
					out_l3g[count] = gyro_buf[2];
					count++;
				}//end while count
				count = 0;
				duty_A = PWM;
				duty_B = PWM * 0.7;
				l3g_angle2[2] = 0;
				BBBIO_ehrPWM_Disable(BBBIO_PWMSS1);
				BBBIO_ehrPWM_Disable(BBBIO_PWMSS0);
				
				if(freopen("/mydata/Encoder_count.txt","w",stdout) == NULL)
					printf("重定向出错");
				for(i = 0;i < distance;i++)
				{
					printf("%f,%f,%f\n",out_A[i],out_B[i],out_l3g[i]);//EncoderA & EncoderB & angular velocity

					//printf("%f\n",out_l3g[i]);
				}
				fclose(stdout);

				break;
			case 's':
				BBBIO_PWMSS_Setting(1, PWM_HZ ,duty_A , 0.1f);
				BBBIO_PWMSS_Setting(0, PWM_HZ , 0.1f , duty_B);
				BBBIO_ehrPWM_Enable(1);
				BBBIO_ehrPWM_Enable(0);

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
						//iolib_delay_ms(10);
					}while(countA < checkpoint);// && countB < checkpoint);
					/*
					   printf("%f\n%f\n",countA,countB);
					   p.target = (float)countA;
					   Pout = PIDCale(&p,countB);
					   duty_B += Pout;
					 */

					p.target = (float)countA;
					pide = PIDCale(&p,countB);
					timeDiff = getTimeDif();
					l3g_ctl(file, timeDiff);
					p.target = 0.0;
					pidg = 0.3 * (PIDCale(&p,gyro_buf[2]));

					Pout = pide - pidg;
					duty_B += Pout;
					/*very importemt*/
					if(duty_B <= 0)
						duty_B = 0.1;
					BBBIO_PWMSS_Setting(0, PWM_HZ ,0.1f , duty_B);
					BBBIO_ehrPWM_Enable(0);
					printf("PWMB set to be %lf%\n",duty_B);
					//out_A[count] = countA;
					out_B[count] = countB;
					PWM_B[count] = duty_B;
					count++;
				}//end while count
				count = 0;
				duty_B = PWM * 0.7;
				l3g_angle2[2] = 0;
				BBBIO_ehrPWM_Disable(BBBIO_PWMSS1);
				BBBIO_ehrPWM_Disable(BBBIO_PWMSS0);
				break;
			case 'a':
				BBBIO_PWMSS_Setting(0, PWM_HZ ,25.0f , 0.1f);
				BBBIO_ehrPWM_Enable(0);
				do
				{
					timeDiff = getTimeDif();
					l3g_ctl(file, timeDiff);
				}while(l3g_angle2[2] < turn_degree);
				l3g_angle2[2] = 0;
				BBBIO_ehrPWM_Disable(BBBIO_PWMSS0);
				break;
			case 'd':
				BBBIO_PWMSS_Setting(1, PWM_HZ ,0.1f , 25.0f);
				BBBIO_ehrPWM_Enable(1);
				do
				{
					timeDiff = getTimeDif();
					l3g_ctl(file, timeDiff);
				}while(l3g_angle2[2] > -turn_degree);
				l3g_angle2[2] = 0;
				BBBIO_ehrPWM_Disable(BBBIO_PWMSS1);
				break;
		}//end switch

		//}
		//BBBIO_ehrPWM_Disable(BBBIO_PWMSS1);
		//BBBIO_ehrPWM_Disable(BBBIO_PWMSS0);
	}

	void main ()
	{
		FILE *log;	//file
		int file;	//file
		int i,j,k=0,notthistime=0, last_act=0, third_act=0,notthistime2=0, dly;
		float timeDiff,time_count;
		short *last_gyro, *last_acc;
		short *third_gyro, *third_acc;
		char *filename;
		short *et_acc;	

		int delay_count;
		short episode = 0,ep_flag = 0;

		struct timespec start;
		struct timespec end;
		struct timespec diff;

		if ((file = open("/dev/i2c-1",O_RDWR)) <0 ) 
		{
			printf("file open error\n");
			return ;
		}

		last_gyro = malloc(sizeof(short)*3);
		memset(last_gyro,0,sizeof(short)*3);
		last_acc = malloc(sizeof(short)*3);
		memset(last_acc,0,sizeof(short)*3);
		third_gyro = malloc(sizeof(short)*3);
		memset(third_gyro,0,sizeof(short)*3);
		third_acc = malloc(sizeof(short)*3);
		memset(third_acc,0,sizeof(short)*3);

		et_acc = malloc(sizeof(short)*3);
		memset(et_acc, 0, sizeof(short)*3);


		filename = malloc(sizeof(char)*10);

		//init
		iolib_init();
		state_init();
		iolib_setdir(8,11, BBBIO_DIR_IN);
		iolib_setdir(8,12, BBBIO_DIR_IN);

		gettimeofday(&last_tv, NULL);
		//load
		printf("LOADING LAST DATA...\n\n");
		load();
		timeDiff = getTimeDif();
		printf("LOAD FINISH cost %.2fsec...\n\n",timeDiff);

		sleep(1);	//為了看print

		l3g_init(file);
		adx_init(file);

		sprintf(filename, "log/logfile");
		if ((log = fopen(filename, "wb") ) == NULL ) 
		{
			printf("log file open error\n");
			return ;
		}

		gettimeofday(&last_tv, NULL);

		//陀螺儀取靜態基準值
		l3g_regulate(file);
		//三軸初始值定義
		adx_regulate(file);
		srand(time(NULL));

		/*	
		k=4000;	//100 = 1s
		while(k) 
		{
			//learning rate decrease
			//if ( k%1500 == 1 ) ALPHA -= 0.1;
			if ( k%400 == 1 ) ALPHA -= 0.1;
			//		dly = 20;
			//		while(dly)
			//		{
			//			usleep(20000);
			timeDiff = getTimeDif();
			l3g_ctl(file, timeDiff);
			adx_ctl(file);
			complementary_filter();	

			//printf("(互補後角度)Axr:%.2f, Ayr:%.2f, Azr:%.2f\n",compAngle[0],compAngle[1],compAngle[2]);
			//printf("角速度 [0]:%d, [1]:%d,[2]:%d\n",gyro_buf[0],gyro_buf[1],gyro_buf[2]);
			for(i=0;i<3;i++)
				et_acc[i] = goal_acc[i] - div_angle_buf[i];
			if (start_flag == 0) 
			{	
				if ( abs(et_acc[2]) > ANGLE_GOOD ) 
				{
				start_flag=1;		//靜態校正後，有誤差才開始學習修正
				}
				if(ep_flag == 1)
				{
					episode++;
					ep_flag = 0;
				}
			}

		//更新QS
			if (notthistime)
			{
			update_qs(last_acc, last_gyro, div_angle_buf, div_gyro_buf, last_act);
			notthistime2 = 1;
			notthistime = 0;
			third_act = last_act;
			memcpy(third_gyro, last_gyro, sizeof(short)*3);
			memcpy(third_acc, last_acc, sizeof(short)*3);
			}
			if (notthistime2)
			{
				update_qs(third_acc, third_gyro, last_acc, last_gyro, third_act);
				notthistime2 = 0;
			}

			if(start_flag) 
			{
				last_act = choose_act(div_angle_buf, div_gyro_buf,EGREEDY,0);
				notthistime = 1;	//因為有了action才可以執行update qs
				memcpy(last_gyro,div_gyro_buf,sizeof(short)*3);
				memcpy(last_acc,div_angle_buf,sizeof(short)*3);

				do_action(last_act);
				k--;	//學習次數-1
				ep_flag = 1;
			}

			usleep(50000);
			printf("\nLk[%d]\n",k);
		}	

		stop_wheele();	

		printf("LEARNING END...\nSAVING DATA...\n");
		//save 學習資料
		save();

		timeDiff = getTimeDif();
		printf("SAVE FINISH...cost  %.2fsec\nREADY TO RECORD...\n",timeDiff);
		printf("//---------------------%d----------------------------//\n",episode);
		sleep(5);
		*/

		//================  停止學習  ==============================================
		//全部重新init
		ALPHA = 0.0;	
		last_act = 0;
		for (i=0;i<3;i++) l3g_angle[i] = 0;

		notthistime = 0;
		start_flag = 0;
		gettimeofday(&last_tv, NULL);

		k=60;	//記錄筆數

		while(1) 
		{
			clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
			for(i=0;i <= 10;i++){
			timeDiff = getTimeDif();
			l3g_ctl(file, timeDiff);
			adx_ctl(file);
			complementary_filter();
			}
			//printf("(互補後角度)Axr:%.2f, Ayr:%.2f, Azr:%.2f\n",compAngle[0],compAngle[1],compAngle[2]);
			//printf("角速度 [0]:%d, [1]:%d,[2]:%d\n",gyro_buf[0],gyro_buf[1],gyro_buf[2]);

			for(i=0;i<3;i++)
				et_acc[i] = goal_acc[i] - div_angle_buf[i];

			if (start_flag == 0) 
			{
				if ( abs(et_acc[2]) > ANGLE_GOOD ) 
				{
					start_flag=1;		//靜態校正後，有誤差才開始學習修正
				}
			}

			if (notthistime) 
			{
				//由reward停止
				update_qs(last_acc, last_gyro, div_angle_buf, div_gyro_buf, last_act);
				notthistime = 0;
			}

			if(start_flag) 
			{
				last_act = choose_act(div_angle_buf, div_gyro_buf,0,0);
				notthistime = 1;	//因為有了action才可以執行update qs
				memcpy(last_gyro,div_gyro_buf,sizeof(short)*3);
				memcpy(last_acc,div_angle_buf,sizeof(short)*3);
				do_action(last_act);
			}

			//log		
			if(k>0) 
			{
				/*	
				//三軸角度XYZ，陀螺儀加速度XYZ，陀螺儀累積角XYZ, 互補XYZ, 最後執行動作	
				fprintf(log,"%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d\n",\
				angle_buf[0],angle_buf[1],(angle_buf[0]>acc_target[0])?angle_buf[2]:(angle_buf[2]*-1),\
				gyro_buf[0],gyro_buf[1],gyro_buf[2],\
				l3g_angle2[0],l3g_angle2[1],l3g_angle2[2],\
				compAngle[0],compAngle[1],compAngle[2],\
				last_act);
				 */
				
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
				 
				fprintf(log,"%f\n",compAngle[2]);//平衡情況
				//fprintf(log,"%hi,%hi\n",acc_buf[0],draw_buf[0]);//加速度動平均濾波有與無
				k--;
				if (k==0) 
				{
					fclose(log);
				}
				printf("\nRECORDING.. [%d]\n",k);
				usleep(50000);
			} else usleep(50000);
			PID_dirctl();
		}

		BBBIO_ehrPWM_Disable(BBBIO_PWMSS0);
		BBBIO_ehrPWM_Disable(BBBIO_PWMSS1);
		close(file);
		//	fclose(log);

		iolib_free();
		}
