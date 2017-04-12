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

//======== ADXL345 =========
#define ADX_ADDR	0x53	//i2c
#define	ADX_POW		0x2D
#define ADX_DATA_F	0x31
#define ADX_FIFO_CTL	0x38
#define	ADX_BW		0x2C
#define	ADX_X_L		0x32
#define	ADX_X_H		0x33
#define	ADX_Y_L		0x34
#define	ADX_Y_H		0x35
#define	ADX_Z_L		0x36
#define	ADX_Z_H		0x37
#define ADX_SOURCE	0x30
#define FIX_TIME	100
#define ADX_SCALE	0.0039f
#define ADX_DIV		3	//計算目標差的block單位，正常以1度計算，為放寬區間將數值放大

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

//state區間
short div_angle_buf[3] = {0,0,0};	//  (QS用)  將三軸轉換成實際夾角 / ADX_DIV = state區間 {X,Y,傾角}
short angle_buf[3] = {0,0,0};	//三軸回傳出夾角

short acc_target[3] = {0,0,0};
short goal_acc[3] = {0,0,0};		// 目標三軸角度/ADX_DIV = 目標state

float compAngle[3] = {0,0,0};	//互補濾波角度	{Axr,Ayr,Azr}

void complementary_filter()
{
	short tmp;
	int i=0;

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

	if (ioctl(file, I2C_SLAVE, ADX_ADDR) <0) {
		printf("i2c open error\n");
		return ;
	}

	for(j=0;j<FIX_TIME;j++)
	{
		for(i=0;i<3;i++) 
		{
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

	short tmp;
	//	float x2,y2,z2;

	//float scale = 0.0039;	//全解析度模式 輸出值的比例為 3.9mg/LSB
	//為了加速計算，就不額外作正確數值調整，以原有輸出對照即可。
	int scale = 1;

	if (ioctl(file, I2C_SLAVE, ADX_ADDR) <0) 
	{
		printf("i2c open error\n");
		return ;
	}

	ready = ADX_SOURCE;
	write(file,&ready,1);
	read(file,&ok_buf,1);
	if ( (ok_buf>>7)== 1 ) 
	{
		for(i=0;i<3;i++) 
		{
			write(file, &read_start_buf[i], 1);
			read(file,&hi_buf,1);
			write(file, &read_start_buf[i+3], 1);
			read(file,&lo_buf,1);
			status = (hi_buf << 8) | lo_buf;

			acc_buf[i]	= status;
		}
	}
	//printf("z : %hi\n",acc_buf[2]);
	//printf("x : %hi\n",acc_buf[0]);
	write(file,&ready,1);
	read(file,&ok_buf,1);

	//角度化
	acc_degree_count(acc_buf,angle_buf);
	// angle_buf : 已轉換為實際角度
	//互補濾波 XZ相傾角(整合為翻仰角)
	//以三軸角度區分方向，修正翻轉角度
	//三軸回傳的角度沒有正負之分，而陀螺儀累積的有
	//不以90度為中置基準，以初始靜置的X軸為主
	/*
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
	//printf("(互補後角度)Axr:%.2f, Ayr:%.2f, Azr:%.2f\n",compAngle[0],compAngle[1],compAngle[2]);

	//區間化
	//	printf("acc_div(div_angle_buf)\n");
	for(i=0;i<3;i++) 
	{
		div_angle_buf[i] = (short)(compAngle[i]/ADX_DIV);
		//		printf("[%d]:%d  ",i,div_angle_buf[i]);
	}
	//	printf("\n");

	//	printf("target acc\n");
	//	for(i=0;i<3;i++) printf("[%d]:%d ",i,goal_acc[i]);
	//	printf("\n");
	*/
}

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

	printf("\n=====\n瞬時Y角速度: %d, angle:%f \n",gyro_buf[1], l3g_angle2[1]);//div_gyro_buf[1]);	//gyro只看Y  so far 03/30

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
	adx_init(file);

	while(1)
	{
		while(!kbhit())
		{
			//陀螺儀取靜態基準值
			//l3g_regulate(file);
			for(i = 0;i <= 40;i++)
			{
				timeDiff = getTimeDif();
				l3g_ctl(file, timeDiff);
				adx_ctl(file);
				complementary_filter();
			}
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
