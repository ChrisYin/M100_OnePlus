package com.zthit.AHRS;

public class Mahnoy_ARHS_Nomag {
	private  double Kp;
	private  double Ki;
    private  double[] q = new double[4];         //四元素初始化
	private  double[] erro_Int = new double[3];
	int arhs_state = 0;
	
	private  void arhs_init() {
		Kp = 1.5;
		Ki = 0.005;
		q[0] = 1; q[1] = 0; q[2] = 0; q[3] = 0;
		erro_Int[0] = 0; erro_Int[1] = 0; erro_Int[2] = 0;
		arhs_state = 1;
	}  
    
    private  void arhs_update(double[] gyr,double[] acc,double SamplePeriod) {
		double norm;
		double vx,vy,vz;      //陀螺仪估计的重力分量
		double ex,ey,ez;      //重力分量的误差
		
		//归一化加速计测量值
		norm = Math.sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
		acc[0] = acc[0]/norm;
		acc[1] = acc[1]/norm;
		acc[2] = acc[2]/norm;
		
		//估测的重力和速度的方向
		vx = 2*(q[1]*q[3] - q[0]*q[2]);
		vy = 2*(q[2]*q[3] + q[0]*q[1]);
		vz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] +q[3]*q[3];
		
		//加速计测量与重力估测误差 
		//error is sum of cross product between 
		//reference direction of field and direction measured by sensor
		ex = (acc[1]*vz - acc[2]*vy);
		ey = (acc[2]*vx - acc[0]*vz);
		ez = (acc[0]*vy - acc[1]*vx);
		
		//integral error scaled integral gain
		erro_Int[0] = erro_Int[0] + ex*Ki;
		erro_Int[1] = erro_Int[1] + ey*Ki;
		erro_Int[2] = erro_Int[2] + ez*Ki;
		
		// adjusted gyroscope measurements
		gyr[0] = gyr[0] + Kp*ex + erro_Int[0];
		gyr[1] = gyr[1] + Kp*ey + erro_Int[1];
		gyr[2] = gyr[2] + Kp*ez + erro_Int[2];
		
		//integrate quaternion rate and normalize
		q[0] = q[0] - SamplePeriod*(q[1]*gyr[0] + q[2]*gyr[1] + q[3]*gyr[2]);
		q[1] = q[1] + SamplePeriod*(q[0]*gyr[0] - q[3]*gyr[1] + q[2]*gyr[2]);
		q[2] = q[2] + SamplePeriod*(q[3]*gyr[0] + q[0]*gyr[1] - q[1]*gyr[2]);
		q[3] = q[3] + SamplePeriod*(q[2]*gyr[0] - q[1]*gyr[1] + q[0]*gyr[2]);
		
		norm = Math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
		q[0] = q[0]/norm;
		q[1] = q[1]/norm;
		q[2] = q[2]/norm;
		q[3] = q[3]/norm;
		
	}
    
    public  double[] arhs(double[] gyr, double[] acc, double SamplePeriod) {
    	if(arhs_state == 0){
			arhs_init();			
		}
		else {
			arhs_update(gyr, acc, SamplePeriod);
		}
		return q;
	}

}
