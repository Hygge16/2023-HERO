#include "ahr.h"
#include "Matrix.h"

float mygetqval[9];	//用于存放传感器转换结果的数组
					 
									 
bmi088_real_data_t bmi088_real_data;
ist8310_real_data_t ist8310_real_data;
kalman_t kalman;									 
													
volatile float q0, q1, q2, q3,w1,w2,w3; // 全局四元数、角速度测量误差
volatile uint32_t lastUpdate, now; // 采样周期计数 单位 us									 


float P[49]={   0.0001,0,0,0,0,0,0,
	            0,0.0001,0,0,0,0,0,
			    0,0,0.0001,0,0,0,0,
			    0,0,0,0.0001,0,0,0,
			    0,0,0,0,0.0002,0,0,
			    0,0,0,0,0,0.0002,0,
			    0,0,0,0,0,0,0.0002  };//初始矩阵赋值，与参考的论文里赋值有所偏差

  float Q[49]={ 0.0001,0,0,0,0,0,0,
                0,0.0001,0,0,0,0,0,
				0,0,0.0001,0,0,0,0,
				0,0,0,0.0001,0,0,0,
				0,0,0,0,0.0005,0,0,		 
				0,0,0,0,0,0.0005,0,	 
				0,0,0,0,0,0,0.0005  };//如果在角度误差偏向于上一个测量误差时，这时我们就更加相信测量值，如果此时测量值与实际值误差过大，则减小Q，在参考论文里  q单位为0.00001的单位矩阵
			    
float R[36]={   0.0003,0,0,0,0,0,
                0,0.0003,0,0,0,0,
			    0,0,0.0003,0,0,0,
				0,0,0,0.0002,0,0,
				0,0,0,0,0.0002,0,
				0,0,0,0,0,0.0002    };//如果在角度误差偏向于上一个预测值时，这时我们就更加相信预测值，如果此时测量值与预测值误差过大，则减小R，在参考论文里  R在加速计校准时为2倍单位矩阵 磁力计校准时为单位矩阵，可以用作参考
			   		
float A[49],B[49],E[42],F1[36],X[49],Z[49],Ht[42],Ft[49],K[42],O[49],T[6],F[49],Y[7],P1[49],U1[36],U1t[36],D1[36],X1[36],X2[36];

float H[42]={   0,0,0,0,0,0,0,
			    0,0,0,0,0,0,0,
			    0,0,0,0,0,0,0,
                0,0,0,0,0,0,0,
                0,0,0,0,0,0,0,													   												  
			    0,0,0,0,0,0,0,  };

float I[49]={   1,0,0,0,0,0,0,
                0,1,0,0,0,0,0,
                0,0,1,0,0,0,0,
                0,0,0,1,0,0,0,
                0,0,0,0,1,0,0,
                0,0,0,0,0,1,0,
                0,0,0,0,0,0,1   };

void GetPitchYawGxGyGz(void)
{
   
}


void IMU_getYawPitchRoll(volatile float * angles) 
{
	float q[4]; //　四元数
  
    IMU_getQ(q); //更新全局四元数

    angles[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw 
    angles[1] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch
    angles[2] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI; // roll
    // if(angles[0]<0)angles[0]+=360.0f;  //将 -+180度  转成0-360度
}

void IMU_getQ(volatile float * q) 
{
    IMU_getValues(mygetqval);	 

    //将陀螺仪的测量值转成弧度每秒
    //加速度和磁力计保持 ADC值　不需要转换
    IMU_AHRSupdate(mygetqval[3] * M_PI/180, mygetqval[4] * M_PI/180, mygetqval[5] * M_PI/180,
    mygetqval[0], mygetqval[1], mygetqval[2], mygetqval[6], mygetqval[7], mygetqval[8]);
        
    q[0] = q0; //返回当前值
    q[1] = q1;
    q[2] = q2;
    q[3] = q3;
}


void IMU_getValues(volatile float * values) 
{
    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);   //循环读取加速计、陀螺仪、温度的值
    ist8310_read_mag(ist8310_real_data.mag);  //读取磁力计的值

    values[0] = bmi088_real_data.accel[0];
    values[1] = bmi088_real_data.accel[1];
    values[2] = bmi088_real_data.accel[2];
    values[3] = bmi088_real_data.gyro[0] / 16.4f;
    values[4] = bmi088_real_data.gyro[1] / 16.4f;
    values[5] = bmi088_real_data.gyro[2] / 16.4f;
    values[6] = ist8310_real_data.mag[0];
    values[7] = ist8310_real_data.mag[1];
    values[8] = ist8310_real_data.mag[2];

}


void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) 
{
    float norm;
    float bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float g=9.79973;
    float Ha1,Ha2,Ha3,Ha4,Hb1,Hb2,Hb3,Hb4;
    float e1,e2,e3,e4,e5,e6;
    float halfT;

    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;

    bx = 0.5513;  //这个地方是该地区的磁场
    bz = 0.8353; 
    /*************更正陀螺仪时间********************/	
    now = Get_Time_Micros();  //   读取单位  us
    if(now<lastUpdate)  //定时器溢出
    { 
    halfT =  ((float)(now + (0xffffffff- lastUpdate)) / 2000000.0f);
    }
    else	{
    halfT =  ((float)(now - lastUpdate) / 2000000.0f);
    }
    lastUpdate = now;	//更新时间

    /*************更正陀螺仪时间********************/		
    norm = invSqrt(ax*ax + ay*ay + az*az); //加速度平方根分之一
    ax = ax * norm*g; //加速度归一化
    ay = ay * norm*g;
    az = az * norm*g;

    norm = invSqrt(mx*mx + my*my + mz*mz);   //磁力计平方根分之一        
    mx = mx * norm; //磁场强度归一化
    my = my * norm;
    mz = mz * norm;

    gx=gx-w1;//角速度 w1 w2 w3 是角速度测量误差
    gy=gy-w2;//角速度 w1 w2 w3 是角速度测量误差
    gz=gz-w3;//角速度 w1 w2 w3 是角速度测量误差

    Ha1=2*-q2*g; /*求出旋转矩阵的一个预先赋值而已，就不用在矩阵当中直接计算*/
    Ha2=2*q3*g;
    Ha3=2*-q0*g;
    Ha4=2*q1*g;	

    Hb1=2*(bx*q0-bz*q2);
    Hb2=2*(bx*q1+bz*q3);
    Hb3=2*(-bx*q2-bz*q0);
    Hb4=2*(-bx*q3+bz*q1);
    //旋转状态矩阵的赋值
    H[0]=Ha1;	H[1]= Ha2;	H[2]= Ha3;	H[3]= Ha4;
    H[7]= Ha4;	H[8]=-Ha3;	H[9]= Ha2;	H[10]=-Ha1;
    H[14]=-Ha3;	H[15]=-Ha4;	H[16]= Ha1;	H[17]= Ha2;
    H[21]= Hb1;	H[22]= Hb2;	H[23]= Hb3;	H[24]= Hb4;
    H[28]= Hb4;	H[29]=-Hb3;	H[30]= Hb2;	H[31]=-Hb1;
    H[35]=-Hb3;	H[36]=-Hb4;	H[37]= Hb1;	H[38]= Hb2;

    //这个地方主要是用作状态更新，便于之后的计算，在有的算法里面。这一步在计算旋转矩阵的后面，都可以试一下效果，能够好用那个
    q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

    //四元数归一
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;
    //F阵，作为对dt积分后的矩阵
    F[0]=1;			F[1]=-gx*halfT;	F[2]=-gz*halfT;	F[3]=-gz*halfT;	F[4]=0; 	F[5]=0; 	F[6]=0;
    F[7]=gx*halfT;	F[8]=1;			F[9]=gz*halfT;	F[10]=-gy*halfT;F[11]=0; 	F[12]=0; 	F[13]=0;
    F[14]=gy*halfT;	F[15]=-gz*halfT;F[16]=1;		F[17]=gx*halfT;	F[18]=0; 	F[19]=0;	F[20]=0;
    F[21]=gz*halfT;	F[22]=gy*halfT;	F[23]=-gx*halfT;F[24]=1;		F[25]=0; 	F[26]=0; 	F[27]=0;
    F[28]=0;		F[29]=0;		F[30]=0;		F[31]=0;		F[32]=1;	F[33]=0;	F[34]=0;
    F[35]=0;		F[36]=0;		F[37]=0;		F[38]=0;		F[39]=0;	F[40]=1;	F[41]=0;
    F[42]=0;		F[43]=0;		F[44]=0;		F[45]=0;		F[46]=0;	F[47]=0;	F[48]=1;

    /***************在对初始矩阵H,F准备好以后，开始卡尔曼滤波*********/
    //第二步  求出先验误差协方差矩阵  Pk- = A*Pk-1*AT + Q  
    //注意，矩阵运算可以采用DSP库，但是太麻烦了，得写很多个函数，所以我使用了网上开源矩阵的运算函数，必须严格按照矩阵运算规律进行运算
    MatrixMultiply(F,7,7,P,7,7,A );	//A=F*P  这里是从公式里面将F阵对应A即状态旋转矩阵，A反而是F乘以初始协方差的一个中转矩阵，不要混乱了。 无论F还是P都是一个7X7矩阵，所以 乘积也是7x7的一个矩阵
    MatrixTranspose(F,7,7,Ft);	  //F转置F'  公式里有转置，所以先将F阵转置求解出来
    MatrixMultiply(A,7,7,Ft,7,7,B); // B=F*P*F'   B矩阵是 A*Pk-1*AT乘法的一个中转矩阵
    MatrixAdd( B,Q,P1,7,7 ); //   Pk- = A*Pk-1*AT + Q   最终运算公式  将B矩阵加上Q矩阵赋值给到P1矩阵，这里的P1是一个7x7的矩阵，在卡尔曼运算当中，一定要记得每一个步骤的矩阵行列，不然后面求逆的时候容易出问题

    //第三步卡尔曼增益的计算这个是卡尔曼当中最为重要的步骤	 K(k) = Pk-*H(k)T/(H*Pk-*HT + VK*R*VKT)   其中Vk为单位矩阵即为1
    // H*Pk-*HT + VK*R*VKT + R部分
    MatrixTranspose(H,6,7,Ht);	  //Ht = H'  H是一个6X7矩阵，为什么不像上面的F矩阵一样是7x7的矩阵呢，因为这里的H矩阵是一个雅可比矩阵，是四元数的雅可比矩阵，因此是6x7，在论文里面是一个3x4，而F是一个4x4都是一个道理
    MatrixMultiply(P1,7,7,Ht,7,6,E ); //E=P*H'  这里的p是先验误差协方差矩阵，E矩阵是P与H转置矩阵的乘积中转矩阵 一个7x7矩阵乘以6x7矩阵，最终得出的也是一个6x7矩阵
    MatrixMultiply(H,6,7,E,7,6,F1 ); //	 F1=H*P*H'  矩阵运算里(AB)C=A(BC)  所以F1矩阵就是H*Pk-*HT的乘积  6X6
    MatrixAdd(F1,R,X,6,6 );           //X=F1+R   将其加上R，这一步的除数就完成了

    //这里也是求解卡尔曼增益系数部分 卡尔曼滤波器UD分解滤波算法
    //求逆前做UD分解 防止发散  P=UD[U]T  UD分解法，这个网上随便看看 U和D分别是一个上三角矩阵和对角矩阵 将一个方阵做对角和三角分解
    UD(X,6,U1,D1);  //将X矩阵分成上三角与对角阵	   
    MatrixTranspose(U1,6,6,U1t);//上三角阵求转置
    MatrixMultiply(U1,6,6,D1,6,6,X1); //X1=U1*D1  公式计算前半部分
    MatrixMultiply(X1,6,6,U1t,6,6,X2); //X2=U1*D1*U1t  （还原矩阵）
    MatrixInverse(X2,6,0);	//对X2求逆  这里求逆是因为矩阵当中A/B就等于A*B的逆，需要注意的是只有方阵秩才满足逆，所以这里能够求逆的一定是一个方阵，如果是常规矩阵，开源按照求广义逆的方法  即ABAT=A,则B是A的逆  X2是一个6x6矩阵
    MatrixMultiply(E,7,6,X2,6,6,K ); //卡尔曼增益系数K  为一个7x6的矩阵

    /*****************************************************************/	

    //状态旋转矩阵之后的预测值
    vx = 2*(q1q3 - q0q2)*g;//ax 的预测值
    vy = 2*(q0q1 + q2q3)*g;//ay 的预测值
    vz = (q0q0 - q1q1 - q2q2 + q3q3)*g;//az 的预测值
    wx = 2*bx*(0.5f - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);//mx 的预测值
    wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);//my 的预测值
    wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5f - q1q1 - q2q2); //mz 的预测值 


    e1=ax-vx;
    e2=ay-vy;
    e3=az-vz;//加速度偏差
    e4=mx-wx;
    e5=my-wy;
    e6=mz-wz;//磁场偏差

    T[0]=e1;
    T[1]=e2;
    T[2]=e3;
    T[3]=e4;
    T[4]=e5;
    T[5]=e6; //偏差赋值给到T矩阵，准备下一步的计算

    //第四步 后验估计 Xk = Xk- + K(k)(Zk - Hxk)  
    MatrixMultiply(K,7,6,T,6,1,Y );  //Y为一个7x1矩阵

    //更新四元数
    q0= q0+Y[0];//更新四元数
    q1= q1+Y[1];//更新四元数
    q2= q2+Y[2];//更新四元数
    q3= q3+Y[3];//更新四元数
    w1= w1+Y[4];//更新角速度偏移
    w2= w2+Y[5];//更新角速度偏移
    w3= w3+Y[6];//更新角速度偏移

    //第五步，更新误差协方差矩阵 Pk = (I - K(k)*H)*Pk
    MatrixMultiply(K,7,6,H,6,7,Z); //K(k)*H赋值给Z		为7x7矩阵
    MatrixSub(I,Z,O,7,7 );//O=I-K*H O矩阵为单位矩阵I减去Z所得  7x7矩阵
    MatrixMultiply(O,7,7,P1,7,7,P);  //最后将0乘以P1得出P 更新状态 p1为前面所求误差协方差矩阵，P刚开始为自动赋值，再次等于此刻状态，之后连续更新，不在此前状态，卡尔曼滤波结束

    /*四元数归一化 保证下一次使用的四元数是单位四元数，得出四元数，可以进行数据融合*/
    norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;

}

float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

uint32_t Get_Time_Micros(void)
{
	return TIM2->CNT;
}
















