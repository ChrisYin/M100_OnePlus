package com.zthit.AHRS;

public class Mahony_ARHS {
	private  double Kp;
	private  double Ki;
    private  double[] q = new double[4];         //四元素初始化
	private  double[] erro_Int = new double[3];
	public int arhs_state = 0;
	
	private  void arhs_init() {
		Kp = 1;
		Ki = 0.001;
		q[0] = 1; q[1] = 0; q[2] = 0; q[3] = 0;
		erro_Int[0] = 0; erro_Int[1] = 0; erro_Int[2] = 0;
		arhs_state = 1;
	}  
    
    private  void arhs_update(double[] gyr,double[] acc,double[] mag,double SamplePeriod) {
		double norm;
		double vx,vy,vz,wx,wy,wz;      //陀螺仪估计的重力分量
		double ex,ey,ez;      //重力分量的误差
		double hx,hy,hz;
		double bx,bz;
		
		//归一化加速计测量值
		norm = Math.sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
		acc[0] = acc[0]/norm;
		acc[1] = acc[1]/norm;
		acc[2] = acc[2]/norm;
		
		//归一化磁力计测量值
		norm = Math.sqrt(mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2]);
	    
		mag[0] = mag[0]/norm;
		mag[1] = mag[1]/norm;
		mag[2] = mag[2]/norm;
		
		//Reference direction of Earth's magnetic field
		hx = 2.0*mag[0]*(0.5-q[2]*q[2]-q[3]*q[3]) + 2.0*mag[1]*(q[1]*q[2]-q[0]*q[3]) + 2.0*mag[2]*(q[1]*q[3]+q[0]*q[2]);
		hy = 2.0*mag[0]*(q[1]*q[2]+q[0]*q[3]) + 2.0*mag[1]*(0.5-q[1]*q[1]-q[3]*q[3]) + 2.0*mag[2]*(q[2]*q[3]-q[0]*q[1]);
		hz = 2.0*mag[0]*(q[1]*q[3]-q[0]*q[2]) + 2.0*mag[1]*(q[2]*q[3]+q[0]*q[1]) + 2.0*mag[2]*(0.5-q[1]*q[1]-q[2]*q[2]);
		bx = Math.sqrt((hx*hx) + (hy*hy));
		bz = hz;
		
		//Estimated direction of gravity and magnetic field
		vx = 2*(q[1]*q[3] - q[0]*q[2]);
		vy = 2*(q[2]*q[3] + q[0]*q[1]);
		vz = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
		wx = 2*bx*(0.5-q[2]*q[2]-q[3]*q[3]) + 2*bz*(q[1]*q[3]-q[0]*q[2]);
		wy = 2*bx*(q[1]*q[2]-q[0]*q[3]) + 2*bz*(q[0]*q[1] + q[2]*q[3]);
		wz = 2*bx*(q[0]*q[2]+q[1]*q[3]) + 2*bz*(0.5-q[1]*q[1]-q[2]*q[2]);
		
		//加速计测量与重力估测误差 
		//error is sum of cross product between 
		//reference direction of field and direction measured by sensor
		ex = (acc[1]*vz - acc[2]*vy) + (mag[1]*wx - mag[2]*wy);
		ey = (acc[2]*vx - acc[0]*vz) + (mag[2]*wx - mag[0]*wz);
		ez = (acc[0]*vy - acc[1]*vx) + (mag[0]*wy - mag[1]*wx);
		
		//integral error scaled integral gain
		erro_Int[0] = erro_Int[0] + ex*SamplePeriod;
		erro_Int[1] = erro_Int[1] + ey*SamplePeriod;
		erro_Int[2] = erro_Int[2] + ez*SamplePeriod;
		
		// Apply feedback terms, adjusted gyroscope measurements
		gyr[0] = gyr[0] + Kp*ex + Ki*erro_Int[0];
		gyr[1] = gyr[1] + Kp*ey + Ki*erro_Int[1];
		gyr[2] = gyr[2] + Kp*ez + Ki*erro_Int[2];
		
		//integrate quaternion rate and normalize
		q[0] = q[0] - 0.5*SamplePeriod*(q[1]*gyr[0] + q[2]*gyr[1] + q[3]*gyr[2]);
		q[1] = q[1] + 0.5*SamplePeriod*(q[0]*gyr[0] - q[3]*gyr[1] + q[2]*gyr[2]);
		q[2] = q[2] + 0.5*SamplePeriod*(q[3]*gyr[0] + q[0]*gyr[1] - q[1]*gyr[2]);
		q[3] = q[3] + 0.5*SamplePeriod*(-q[2]*gyr[0] + q[1]*gyr[1] + q[0]*gyr[2]);
		
		norm = Math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
		q[0] = q[0]/norm;
		q[1] = q[1]/norm;
		q[2] = q[2]/norm;
		q[3] = q[3]/norm;
				
	}
    
    public  double[] arhs(double[] gyr, double[] acc, double[] mag, double SamplePeriod) {
    	if(arhs_state == 0){
			arhs_init();			
		}
		else {
			arhs_update(gyr, acc, mag, SamplePeriod);
		}
		return q;
	}

}
