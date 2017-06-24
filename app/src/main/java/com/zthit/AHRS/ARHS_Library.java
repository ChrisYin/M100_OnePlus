package com.zthit.AHRS;





public class ARHS_Library {
	
//	public void arhrs_init(double []accel,double []gyro,double []mag){
//		//姿态初始化
//		double phi,theta,psi;
//		phi = accel2phi(accel);
//		theta = accel2theta(accel);
//		psi = mag2psi(mag, phi, theta);	
//	}

	//////由传感器数据获取欧拉角的测量值/////////////
    /////////*******************************///////////////////
	public static double accel2phi(double[] accel) {
		//加速度计求roll角
		double phi;
		phi = Math.atan2(-accel[1], -accel[2]);      //加了一个负号
				return phi;
	}
	
	public static double accel2theta(double phi, double[] accel) {
		//加速度计求pitch角
		double theta,azn;
		azn = Math.cos(phi) * accel[2] + Math.sin(phi) * accel[1];
		theta = Math.atan2(accel[0], -azn);
//		theta = -Math.asin(accel[0]/(Math.sqrt(accel[0]*accel[0]+accel[1]*accel[1]+accel[2]*accel[2])));
		return theta;
	}
	 
	public static double mag2psi(double[] mag, double phi, double theta) {
		//磁力计求yaw角
//		double M_north, M_east;
//		M_north = Math.cos(theta)                 * mag[0] +
//				  Math.sin(phi) * Math.sin(theta) * mag[1] +
//				  Math.cos(phi) * Math.sin(theta) * mag[2];
//		M_east = Math.cos(phi) * mag[1] -
//				 Math.sin(phi) * mag[2];
//		double psi = -Math.atan2(M_east, M_north);
		double xn,yn,zn,psi;
		zn = Math.cos(phi) * mag[2] + Math.sin(phi) * mag[1];
		yn = Math.cos(phi)	* mag[1] - Math.sin(phi) * mag[2];
		xn = Math.cos(theta) * mag[0] + Math.sin(theta) * zn;
		psi = Math.atan2(-yn, xn);
		
		return psi;				  
	}
	
    //////由传感器数据获取欧拉角的测量值/////////////
	///////////*******************************///////////////////
	
	public static double[] eulers2quat(double[] eulers) {
		// 欧拉角转换为四元数
		double phi2 = eulers[0]/2;
		double theta2 = eulers[1]/2;
		double psi2 = eulers[2]/2;
		
		double sinphi2 = Math.sin(phi2);
		double cosphi2 = Math.cos(phi2);
		double sintheta2 = Math.sin(theta2);
		double costheta2 = Math.cos(theta2);
		double sinpsi2 = Math.sin(psi2);
		double cospsi2 = Math.cos(psi2);
		
		double[] q = new double[4];
		
		q[0] = cosphi2 * costheta2 * cospsi2 + sinphi2 * sintheta2 * sinpsi2;
		q[1] = sinphi2 * costheta2 * cospsi2 - cosphi2 * sintheta2 * sinpsi2;
		q[2] = cosphi2 * sintheta2 * cospsi2 + sinphi2 * costheta2 * sinpsi2;
		q[3] = cosphi2 * costheta2 * sinpsi2 - sinphi2 * sintheta2 * cospsi2;
		
		return q;
	}
	
	///////对应四元素获取欧拉角的估计值方程////////
	public static double[] quat2eulers(double[] q) {
		//四元数转换为欧拉角
		double[] angles = new double[3];
		//roll角phi
		angles[0] = quat2roll(q);
		//pitch角theta
		angles[1] = quat2pitch(q);
		//yaw角psi
		angles[2] = quat2yaw(q);
		//转换为角度
		angles[0] = angles[0]*180.0/Math.PI;
		angles[1] = angles[1]*180.0/Math.PI;
		angles[2] = angles[2]*180.0/Math.PI;
		
		return angles;
		
	}
	
	public static double quat2roll(double[] q) {
		double phi;
		phi = Math.atan2(2.0*(q[2]*q[3] + q[0]*q[1]), 
			    (q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3]));
		return phi;
	}
	
	public static double quat2pitch(double[] q) {
		double theta;
		theta = Math.asin(-2.0*(q[1]*q[3]-q[0]*q[2]));
		return theta;
	}
	
	public static double quat2yaw(double[] q) {
		double psi;
		psi = Math.atan2(2.0*(q[1]*q[2]+q[0]*q[3]),
			    (q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3]));
		return psi;
	}
	
	public static double[] quat_product(double[] a, double[] b) {
		//四元数的乘积
		double[] q = new double[4];
		q[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
		q[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
		q[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
		q[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
		return q;
	}
	
	public static double[] quat_conj(double[] q) {
		//求四元数的共轭
		q[0] = q[0];
		q[1] = -q[1];
		q[2] = -q[2];
		q[3] = -q[3];
		return q;
	}
	
	public static double[][] quat2Reb(double[] q) {
		//由四元素(body to reference)获取地理坐标系到载体坐标系的方向余弦矩阵 inertial to body
		//Reb左乘一个在地理坐标系中的向量可变换到载体坐标系中
		double[][] Reb = new double[3][3];
		Reb[0][0] = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
		Reb[0][1] = 2*(q[1]*q[2] + q[0]*q[3]);
		Reb[0][2] = 2*(q[1]*q[3] - q[0]*q[2]);
		Reb[1][0] = 2*(q[1]*q[2] - q[0]*q[3]);
		Reb[1][1] = q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3];
		Reb[1][2] = 2*(q[2]*q[3] + q[0]*q[1]);
		Reb[2][0] = 2*(q[1]*q[3] + q[0]*q[2]);
		Reb[2][1] = 2*(q[2]*q[3] - q[0]*q[1]);
		Reb[2][2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
		return Reb;
	}
	
	

}
