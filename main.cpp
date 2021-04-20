//#include "pch.h"
#include <iostream>
#include <stdio.h>
#include <math.h>

#define RAD2ANG (3.1415926535898/180.0)
#define ANG2RAD(N) ( (N) * (180.0/3.1415926535898) )
void inverseKinematics(double x, double y, double z);

int main()
{
	inverseKinematics(0, 9.5, 20);//逆解目标为（0，30，0）这个坐标
	return 0;

}

void inverseKinematics(double x, double y, double z)
{
	double a, b, c; //临时变量
	double L1 = 10, L2 = 9.5, L3 = 10;//3节手臂的长度
	double m, n, t, q, p;//临时变量
	double j1, j2, j3, j0;//4个舵机的旋转角度
	double x1, y1, z1;//逆解后正解算出来的值，看是否与逆解值相等
	char i = 0;
	j0 = atan2(y, x);
	a = x / cos(j0);
	if (x == 0) a = y; //如果x为0，需要交换x，y
	b = z;

	for (j1 = -90; j1 < 90; j1++)
	{
		j1 *= RAD2ANG;
		j3 = acos((pow(a, 2) + pow(b, 2) + pow(L1, 2) - pow(L2, 2) - pow(L3, 2) - 2 * a*L1*sin(j1) - 2 * b*L1*cos(j1)) / (2 * L2*L3));
		//if (abs(ANG2RAD(j3)) >= 135) { j1 = ANG2RAD(j1); continue; }
		m = L2 * sin(j1) + L3 * sin(j1)*cos(j3) + L3 * cos(j1)*sin(j3);
		n = L2 * cos(j1) + L3 * cos(j1)*cos(j3) - L3 * sin(j1)*sin(j3);
		t = a - L1 * sin(j1);
		p = pow(pow(n, 2) + pow(m, 2), 0.5);
		q = asin(m / p);
		j2 = asin(t / p) - q;
		//if (abs(ANG2RAD(j2)) >= 135) { j1 = ANG2RAD(j1); continue; }
		/***************计算正解然后与目标解对比，看解是否正确**************/
		x1 = (L1 * sin(j1) + L2 * sin(j1 + j2) + L3 * sin(j1 + j2 + j3))*cos(j0);
		y1 = (L1 * sin(j1) + L2 * sin(j1 + j2) + L3 * sin(j1 + j2 + j3))*sin(j0);
		z1 = L1 * cos(j1) + L2 * cos(j1 + j2) + L3 * cos(j1 + j2 + j3);
		j1 = ANG2RAD(j1);
		j2 = ANG2RAD(j2);
		j3 = ANG2RAD(j3);
		if (x1<(x + 0.1) && x1 >(x - 0.1) && y1<(y + 0.1) && y1 >(y - 0.1) && z1<(z + 0.1) && z1 >(z - 0.1))
		{
			printf("j0:%f,j1:%f,j2:%f,j3:%f,x:%f,y:%f,z:%f\r\n", ANG2RAD(j0), j1, j2, j3, x1, y1, z1);
			i = 1;
		}
	}
	for (j1 = -90; j1 < 90; j1++)//这个循环是为了求解另一组解，j2 = asin(t / p) - q;j2 = -(asin(t / p) - q);多了个负号
	{
		j1 *= RAD2ANG;
		j3 = acos((pow(a, 2) + pow(b, 2) + pow(L1, 2) - pow(L2, 2) - pow(L3, 2) - 2 * a*L1*sin(j1) - 2 * b*L1*cos(j1)) / (2 * L2*L3));
		//if (abs(ANG2RAD(j3)) >= 135) { j1 = ANG2RAD(j1); continue; }
		m = L2 * sin(j1) + L3 * sin(j1)*cos(j3) + L3 * cos(j1)*sin(j3);
		n = L2 * cos(j1) + L3 * cos(j1)*cos(j3) - L3 * sin(j1)*sin(j3);
		t = a - L1 * sin(j1);
		p = pow(pow(n, 2) + pow(m, 2), 0.5);
		q = asin(m / p);
		j2 = -(asin(t / p) - q);
		//if (abs(ANG2RAD(j2)) >= 135) { j1 = ANG2RAD(j1); continue; }
		x1 = (L1 * sin(j1) + L2 * sin(j1 + j2) + L3 * sin(j1 + j2 + j3))*cos(j0);
		y1 = (L1 * sin(j1) + L2 * sin(j1 + j2) + L3 * sin(j1 + j2 + j3))*sin(j0);
		z1 = L1 * cos(j1) + L2 * cos(j1 + j2) + L3 * cos(j1 + j2 + j3);
		j1 = ANG2RAD(j1);
		j2 = ANG2RAD(j2);
		j3 = ANG2RAD(j3);
		if (x1<(x + 0.1) && x1 >(x - 0.1) && y1<(y + 0.1) && y1 >(y - 0.1) && z1<(z + 0.1) && z1 >(z - 0.1))
		{
			printf("j0:%f,j1:%f,j2:%f,j3:%f,x:%f,y:%f,z:%f\r\n", ANG2RAD(j0), j1, j2, j3, x1, y1, z1);
			
			i = 1;
		}
	}

	if (i == 0)printf("无解");
}
