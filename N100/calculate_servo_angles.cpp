#include <cmath>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float servo1;
    float servo2;
    float servo3;
    float servo4;
} ServoAngles;

ServoAngles calculate_servo_angles(float X , float Y, float Z) {
    float a1,a2,a3,a4;                  //a1为底部圆台高度 剩下三个为三个机械臂长度 
	float j1,j4,j2,j3;                      //四个姿态角
	float L,H,P;	             //L =a2*sin(j2) + a3*sin(j2 + j3);H = a2*cos(j2) + a3*cos(j2 + j3); P为底部圆盘半径R
	float j_all;	                             //j2,j3,j4之和
	float len,high;                       //总长度,总高度
	float Cosj3,Sinj3;                   //用来存储cosj3,sinj3数值
	float Cosj2,Sinj2;
	float K1,K2;
	  	             
	int i;
	float n,m,q;
	n = 0;
	m = 0;
	q = 0;
	//目标点坐标（X，Y，Z） 
	//
	// y 3-18cm
	P = 15;     //底部圆盘半径
	a1 = 12.5; 	//底部圆盘高度	            
	a2 = 12;    //机械臂长度
	a3 = 12;
	a4 = 12;
	
	if (X == 0) 
	    j1=90;
	else 
	    j1 = atan((Y+P)/X)*(57.3);

	for(i=0;i<=180;i++)
	{	
		j_all = 3.1415927*i/180;

		len = sqrt((Y+P)*(Y+P)+X*X);
		high = Z;
			
		L = len	- a4*sin(j_all);
		H = high - a4*cos(j_all) - a1;
		
		Cosj3 = ((L*L)+(H*H)-((a2)*(a2))-((a3)*(a3)))/(2*(a2)*(a3));
		Sinj3 = (sqrt(1-(Cosj3)*(Cosj3)));
		
		j3 = atan((Sinj3)/(Cosj3))*(57.3);
		
		K2 = a3*sin(j3/57.3);
		K1 = a2+a3*cos(j3/57.3);
		
		Cosj2 = (K2*L+K1*H)/(K1*K1+K2*K2);
		Sinj2 = (sqrt(1-(Cosj2)*(Cosj2)));
		
		j2 = atan((Sinj2)/(Cosj2))*57.3;
		j4 = j_all*57.3- j2 - j3;
		
		if(j2>=0&&j3>=0&&j4>=0&&j2<=90&&j3<=90&&j4<=90)
		{
			n=n+1;
		}
    } 
   
   
   	for(i=0;i<=180;i++)
	{
		j_all = 3.1415927*i/180;
		
		len = sqrt((Y+P)*(Y+P)+X*X);
		high = Z;

		L = len	- a4*sin(j_all);
		H = high - a4*cos(j_all) - a1;
		
		Cosj3 = ((L*L)+(H*H)-((a2)*(a2))-((a3)*(a3)))/(2*(a2)*(a3));
		Sinj3 = (sqrt(1-(Cosj3)*(Cosj3)));
		
		j3 = atan((Sinj3)/(Cosj3))*(57.3);
		
		K2 = a3*sin(j3/57.3);
		K1 = a2+a3*cos(j3/57.3);
		
		Cosj2 = (K2*L+K1*H)/(K1*K1+K2*K2);
		Sinj2 = (sqrt(1-(Cosj2)*(Cosj2)));
		
		j2 = atan((Sinj2)/(Cosj2))*57.3;
		j4 = j_all*57.3- j2 - j3;
		
	    if(j2>=0&&j3>=0&&j4>=0&&j2<=90&&j3<=90&&j4<=90)
		{
			m=m+1;
			if(m==n/2||m==(n+1)/2)
			{	
				break;
			}
		}
    }
   
	//舵机设置值
    float servo1 = 0;
    float servo2 = 0;
    float servo3 = 0;
    float servo4 = 0;

	if (j1 != 90) {
	  if (j1 > 0) {
	    servo1 = j1;
	  } else if (j1 < 0) {
	    servo1 = 90 + (90 - abs(j1));
	  }
	} else {
	  servo1 = 90;
	}

	if (j2 > 0 ) {
		servo2 = j2 + 90;
	} else if (j2 < 0) {
		servo2 = 90 - abs(j2);
	}  else {
		servo2 = 90;
	}

	if (j3 > 0) {
		servo3 = 90 - j3;
	} else if (j3 < 0) {
		servo3 = 90 + abs(j3);
	} else {
		servo3 = 90;
	}
	
	if (j4 > 0 ) {
		servo4 = j4 + 90;
	} else if (j4 < 0) {
		servo4 = 90 - abs(j4);
	}  else {
		servo4 = 90;
	}		
    // 返回结构体
    ServoAngles result;
    result.servo1 = servo1;
    result.servo2 = servo2;
    result.servo3 = servo3;
    result.servo4 = servo4;
    return result;
}

#ifdef __cplusplus
}
#endif
