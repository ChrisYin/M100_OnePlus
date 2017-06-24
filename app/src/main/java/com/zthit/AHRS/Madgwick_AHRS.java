package com.zthit.AHRS;


//该姿态算法算出来的姿态q表示的是body frame（NED）如何旋转到参考坐标系NED frame
//对应的传感器加速度计的值D朝下（0,0,g）,N朝下（g,0,0）,E朝下（0,g,0）
//陀螺仪的值也需要按照NED的顺时针为正，逆时针为负

//以上说明废除，现在全部更正为和openpilot的坐标系定义一致，NED Frame，
//对应的传感器加速度计的值D朝下（0,0,-g）,N朝下（-g,0,0）,E朝下（0,-g,0）
//加速度计的测量值为线性加速度减去重力加速度
public class Madgwick_AHRS {
	
//	private static double[] w_bias = new double[3];
	private  double beta;
//	private static double zeta;
	public int arhs_state = 0;
//	private int instability_fix = 1;
	public  double[] q = new double[4]; 
	
	private  void arhs_init(double []accel,double []gyro,double []mag) {
		double[] angle = new double[3];
		double xn,yn,zn,azn,recipNorm;
//		double recipNorm;
//		angle[0] = ARHS_Library.accel2phi(accel);
//		angle[1] = ARHS_Library.accel2theta(accel);
//		angle[2] = ARHS_Library.mag2psi(mag, angle[0], angle[1]);

		//以下为借用的open pilot上面的初始化过程
		angle[0] = Math.atan2(-accel[1],-accel[2]);
		zn = Math.cos(angle[0])	* mag[2] + Math.sin(angle[0]) * mag[1];
		yn = Math.cos(angle[0])	* mag[1] - Math.sin(angle[0]) * mag[2];
		azn = Math.cos(angle[0]) * accel[2] + Math.sin(angle[0]) * accel[1];
		angle[1] = Math.atan2(accel[0], -azn);
		xn = Math.cos(angle[1]) * mag[0] + Math.sin(angle[1]) * zn;
		angle[2] = Math.atan2(-yn,xn);
		q = ARHS_Library.eulers2quat(angle);
//		Log.d("q: " , q[0]+ " " + q[1] + " " + q[2] + " " + q[3]);
		
		recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
		q[0] *= recipNorm;
		q[1] *= recipNorm;
		q[2] *= recipNorm;
		q[3] *= recipNorm;
		
		
//		w_bias[0] = 0; w_bias[1] = 0; w_bias[2] = 0;
		
//		q[0] = 1; q[1] = 0; q[2] = 0; q[3] = 0;
		beta = 0.1;    //由陀螺仪的静止噪声设置，噪声越大值越大，噪声越小值越小。
//		zeta = 0.5;
//		arhs_state = 1;
		
		angle = null;
	}
	
//	private static void arhs_update(double[] gyr,double[] acc,double[] mag,double SamplePeriod) {
//		
//		double norm;
//		double[][] F = new double[6][1];
//		double[][] J = new double[6][4];
//		double[] h = new double[4];
//		double[] w_erro = new double[4];    
//		double[][] step = new double[4][1];
//		double[] q_dot_omega = new double[4];
//		double[] q_dot = new double[4];
//		double bx,bz;
//		double[] s_mag = {0.0,mag[0],mag[1],mag[2]};
//		double[] s_gyr = new double[4];
//		double[] q_conj = ARHS_Library.quat_conj(q);
//		
//		//归一化加速计测量值
//		norm = Math.sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
//		acc[0] = acc[0]/norm;
//		acc[1] = acc[1]/norm;
//		acc[2] = acc[2]/norm;
//						
//		//归一化磁力计测量值
//		norm = Math.sqrt(mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2]);		   
//		mag[0] = mag[0]/norm;
//		mag[1] = mag[1]/norm;
//		mag[2] = mag[2]/norm;
//				
//		
//		h = ARHS_Library.quat_product(q, ARHS_Library.quat_product(s_mag, q_conj));
//		bx = Math.sqrt((h[1]*h[1]) + (h[2]*h[2]));
//		bz = h[3];
//		
//		F[0][0] = 2.0*(q[1]*q[3] - q[0]*q[2]) - acc[0];
//		F[1][0] = 2.0*(q[0]*q[1] + q[2]*q[3]) - acc[1];
//		F[2][0] = 2.0*(0.5-q[1]*q[1]-q[2]*q[2]) - acc[2];
//		F[3][0] = 2.0*bx*(0.5 - q[2]*q[2] - q[3]*q[3]) + 2.0*bz*(q[1]*q[3] - q[0]*q[2]) - mag[0];
//		F[4][0] = 2.0*bx*(q[1]*q[2] - q[0]*q[3]) + 2.0*bz*(q[0]*q[1] + q[2]*q[3]) - mag[1];
//		F[5][0] = 2.0*bx*(q[0]*q[2] + q[1]*q[3]) + 2.0*bz*(0.5 - q[1]*q[1] - q[2]*q[2]) - mag[2];
//		
//		J[0][0] = -2.0*q[2];  J[0][1] = 2.0*q[3];   J[0][2] = -2.0*q[0];   J[0][3] = 2.0*q[1];
//		J[1][0] =  2.0*q[1];  J[1][1] = 2.0*q[0];   J[1][2] = 2.0*q[3];    J[1][3] = 2.0*q[2];
//		J[2][0] =  0.0;       J[2][1] = -4.0*q[1];  J[2][2] = -4.0*q[2];   J[2][3] = 0.0;
//		J[3][0] = -2.0*bz*q[2];   J[3][1] = 2.0*bz*q[3];  J[3][2] = -4.0*bx*q[2]-2.0*bz*q[0];   J[3][3] = -4.0*bx*q[3] + 2.0*bz*q[1];
//		J[4][0] = -2.0*bx*q[3] + 2.0*bz*q[1];  J[4][1] = 2.0*bx*q[2] + 2.0*bz*q[0];  J[4][2] = 2.0*bx*q[0] - 4.0*bz*q[2];  J[4][3] = -4.0*bx*q[3] + 2.0*bz*q[1];
//		J[5][0] = 2.0*bx*q[2];  J[5][1] = 2.0*bx*q[3] - 4.0*bz*q[1];  J[5][2] = 2.0*bx*q[0] - 4.0*bz*q[2];  J[5][3] = 2.0*bx*q[1];
//		
//		double[][] J_trans = Matrix_lib.matrix_tranpose(J);
//		step = Matrix_lib.matrix_multi(J_trans, F);
//		norm = Math.sqrt(step[0][0]*step[0][0]+step[1][0]*step[1][0]+step[2][0]*step[2][0]+step[3][0]*step[3][0]);
//		step[0][0] = step[0][0]/norm;
//		step[1][0] = step[1][0]/norm;
//		step[2][0] = step[2][0]/norm;
//		step[2][0] = step[3][0]/norm;
//		double[] step_q = {step[0][0],step[1][0],step[2][0],step[3][0]};
//		
//		w_erro = ARHS_Library.quat_product(q_conj, step_q);
//		w_bias[0] = w_bias[0] + 2.0*w_erro[0]*SamplePeriod*zeta;
//		w_bias[1] = w_bias[1] + 2.0*w_erro[1]*SamplePeriod*zeta;
//		w_bias[2] = w_bias[2] + 2.0*w_erro[2]*SamplePeriod*zeta;
//		
//		s_gyr[0] = 0.0;
//		s_gyr[1] = gyr[0] - w_bias[0];
//		s_gyr[2] = gyr[1] - w_bias[1];
//		s_gyr[3] = gyr[2] - w_bias[2];
//		
//		q_dot_omega = ARHS_Library.quat_product(q,s_gyr);
//		
//		q_dot[0] = 0.5*q_dot_omega[0] - beta*step_q[0];
//		q_dot[1] = 0.5*q_dot_omega[1] - beta*step_q[1];
//		q_dot[2] = 0.5*q_dot_omega[2] - beta*step_q[2];
//		q_dot[3] = 0.5*q_dot_omega[3] - beta*step_q[3];
//		
//		q[0] = q[0] + q_dot[0]*SamplePeriod;
//		q[1] = q[1] + q_dot[1]*SamplePeriod;
//		q[2] = q[2] + q_dot[2]*SamplePeriod;
//		q[3] = q[3] + q_dot[3]*SamplePeriod;
//		
//		norm = Math.sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
//		q[0] = q[0]/norm;
//		q[1] = q[1]/norm;
//		q[2] = q[2]/norm;
//		q[3] = q[3]/norm;
//				
//	}
	
	private  void arhs_update(double[] gyr,double[] acc,double[] mag,double SamplePeriod){
		
//		Log.v("CalAcc: " , acc[0]+ " " + acc[1] + " " + acc[2]);
//		acc = Calibrarion.AccCal(acc, AccW, AccB);

		
		double recipNorm;
		double s0, s1 , s2, s3;
		double qDot1, qDot2, qDot3, qDot4;
		double hx, hy;
		double _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
		double ax = acc[0], ay = acc[1], az = acc[2];
		double gx = gyr[0], gy = gyr[1], gz = gyr[2];
		double mx = mag[0], my = mag[1], mz = mag[2];
		
		if((mx == 0.0) && (my == 0.0) && (mz == 0.0)) {
			arhs_update_IMU(gyr, acc,SamplePeriod);
			return;
		}
		
		// Rate of change of quaternion from gyroscope
		qDot1 = 0.5 * (-q[1] * gx - q[2] * gy - q[3] * gz);
		qDot2 = 0.5 * (q[0] * gx + q[2] * gz - q[3] * gy);
		qDot3 = 0.5 * (q[0] * gy - q[1] * gz + q[3] * gx);
		qDot4 = 0.5 * (q[0] * gz + q[1]* gy - q[2] * gx);
		
		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

			// Normalize accelerometer measurement
			recipNorm = invSqrt(ax * ax + ay * ay + az * az);
			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;   

			// Normalize magnetometer measurement
			recipNorm = invSqrt(mx * mx + my * my + mz * mz);
			mx *= recipNorm;
			my *= recipNorm;
			mz *= recipNorm;

			// Auxiliary variables to avoid repeated arithmetic
			_2q0mx = 2.0 * q[0] * mx;
			_2q0my = 2.0 * q[0] * my;
			_2q0mz = 2.0 * q[0] * mz;
			_2q1mx = 2.0 * q[1] * mx;
			_2q0 = 2.0 * q[0];
			_2q1 = 2.0 * q[1];
			_2q2 = 2.0 * q[2];
			_2q3 = 2.0 * q[3];
			_2q0q2 = 2.0 * q[0] * q[2];
			_2q2q3 = 2.0 * q[2] * q[3];
			q0q0 = q[0] * q[0];
			q0q1 = q[0] * q[1];
			q0q2 = q[0] * q[2];
			q0q3 = q[0] * q[3];
			q1q1 = q[1] * q[1];
			q1q2 = q[1] * q[2];
			q1q3 = q[1] * q[3];
			q2q2 = q[2] * q[2];
			q2q3 = q[2] * q[3];
			q3q3 = q[3] * q[3];

			// Reference direction of Earth's magnetic field, having been compensition
			hx = mx * q0q0 - _2q0my * q[3] + _2q0mz * q[2] + mx * q1q1 + _2q1 * my * q[2] + _2q1 * mz * q[3] - mx * q2q2 - mx * q3q3;
			hy = _2q0mx * q[3] + my * q0q0 - _2q0mz * q[1] + _2q1mx * q[2] - my * q1q1 + my * q2q2 + _2q2 * mz * q[3] - my * q3q3;
//			_2bx = Math.sqrt(hx * hx + hy * hy);
//			_2bz = -_2q0mx * q[2] + _2q0my * q[1] + mz * q0q0 + _2q1mx * q[3] - mz * q1q1 + _2q2 * my * q[3] - mz * q2q2 + mz * q3q3;
			//应该是2倍
			_2bx = 2.0 * Math.sqrt(hx * hx + hy * hy);
			_2bz = 2.0 * (-_2q0mx * q[2] + _2q0my * q[1] + mz * q0q0 + _2q1mx * q[3] - mz * q1q1 + _2q2 * my * q[3] - mz * q2q2 + mz * q3q3);			
			_4bx = 2.0 * _2bx;
			_4bz = 2.0 * _2bz;
//			Log.v("bx:", " " + _2bx);

			// Gradient decent algorithm corrective step
			//做相应的修改，因为定义的参考坐标系的g值为（0,0，-1）
//			s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - ax) + _2q1 * (2.0 * q0q1 + _2q2q3 - ay) - _2bz * q[2] * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q[3] + _2bz * q[1]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q[2] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
//			s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - ax) + _2q0 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q[1] * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + _2bz * q[3] * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q[2] + _2bz * q[0]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q[3] - _4bz * q[1]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
//			s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - ax) + _2q3 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q[2] * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-_4bx * q[2] - _2bz * q[0]) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q[1] + _2bz * q[3]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q[0] - _4bz * q[2]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
//			s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - ax) + _2q2 * (2.0 * q0q1 + _2q2q3 - ay) + (-_4bx * q[3] + _2bz * q[1]) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q[0] + _2bz * q[2]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q[1] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
			s0 = _2q2 * (-2.0 * q1q3 + _2q0q2 - ax) - _2q1 * (-2.0 * q0q1 - _2q2q3 - ay) - _2bz * q[2] * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q[3] + _2bz * q[1]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q[2] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
			s1 = -_2q3 * (-2.0 * q1q3 + _2q0q2 - ax) - _2q0 * (-2.0 * q0q1 - _2q2q3 - ay) + 4.0 * q[1] * (-1.0 + 2.0 * q1q1 + 2.0 * q2q2 - az) + _2bz * q[3] * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q[2] + _2bz * q[0]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q[3] - _4bz * q[1]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
			s2 = _2q0 * (-2.0 * q1q3 + _2q0q2 - ax) - _2q3 * (-2.0 * q0q1 - _2q2q3 - ay) + 4.0 * q[2] * (-1.0 + 2.0 * q1q1 + 2.0 * q2q2 - az) + (-_4bx * q[2] - _2bz * q[0]) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q[1] + _2bz * q[3]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q[0] - _4bz * q[2]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
			s3 = -_2q1 * (-2.0 * q1q3 + _2q0q2 - ax) - _2q2 * (-2.0 * q0q1 - _2q2q3 - ay) + (-_4bx * q[3] + _2bz * q[1]) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q[0] + _2bz * q[2]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q[1] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz);
			
			recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
			s0 *= recipNorm;
			s1 *= recipNorm;
			s2 *= recipNorm;
			s3 *= recipNorm;

			// Apply feedback step
			qDot1 -= beta * s0;
			qDot2 -= beta * s1;
			qDot3 -= beta * s2;
			qDot4 -= beta * s3;
		}

		// Integrate rate of change of quaternion to yield quaternion
		q[0] += qDot1 * SamplePeriod;
		q[1] += qDot2 * SamplePeriod;
		q[2] += qDot3 * SamplePeriod;
		q[3] += qDot4 * SamplePeriod;

		// Normalise quaternion
		recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
		q[0] *= recipNorm;
		q[1] *= recipNorm;
		q[2] *= recipNorm;
		q[3] *= recipNorm;

	}
		
	
	private  void  arhs_update_IMU(double[] gyr, double[] acc, double SamplePeriod) {
		double recipNorm;
		double s0, s1 , s2, s3;
		double qDot1, qDot2, qDot3, qDot4;
		double _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
		double ax = acc[0], ay = acc[1], az = acc[2];
		double gx = gyr[0], gy = gyr[1], gz = gyr[2];
		
		// Rate of change of quaternion from gyroscope
		qDot1 = 0.5f * (-q[1] * gx - q[2] * gy - q[3] * gz);
		qDot2 = 0.5f * (q[0] * gx + q[2] * gz - q[3] * gy);
		qDot3 = 0.5f * (q[0] * gy - q[1] * gz + q[3] * gx);
		qDot4 = 0.5f * (q[0] * gz + q[1] * gy - q[2] * gx);
		
		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

			// Normalise accelerometer measurement
			recipNorm = invSqrt(ax * ax + ay * ay + az * az);
			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;   

			// Auxiliary variables to avoid repeated arithmetic
			_2q0 = 2.0 * q[0];
			_2q1 = 2.0 * q[1];
			_2q2 = 2.0 * q[2];
			_2q3 = 2.0 * q[3];
			_4q0 = 4.0 * q[0];
			_4q1 = 4.0 * q[1];
			_4q2 = 4.0 * q[2];
			_8q1 = 8.0 * q[1];
			_8q2 = 8.0 * q[2];
			q0q0 = q[0] * q[0];
			q1q1 = q[1] * q[1];
			q2q2 = q[2] * q[2];
			q3q3 = q[3] * q[3];

			// Gradient decent algorithm corrective step
//			s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
//			s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q[1] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
//			s2 = 4.0f * q0q0 * q[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
//			s3 = 4.0f * q1q1 * q[3] - _2q1 * ax + 4.0f * q2q2 * q[3] - _2q2 * ay;
			s0 = _4q0 * q2q2 - _2q2 * ax + _4q0 * q1q1 + _2q1 * ay;
			s1 = _4q1 * q3q3 + _2q3 * ax + 4.0f * q0q0 * q[1] + _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 - _4q1 * az;
			s2 = 4.0f * q0q0 * q[2] - _2q0 * ax + _4q2 * q3q3 + _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 - _4q2 * az;
			s3 = 4.0f * q1q1 * q[3] + _2q1 * ax + 4.0f * q2q2 * q[3] + _2q2 * ay;
			recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
			s0 *= recipNorm;
			s1 *= recipNorm;
			s2 *= recipNorm;
			s3 *= recipNorm;

			// Apply feedback step
			qDot1 -= beta * s0;
			qDot2 -= beta * s1;
			qDot3 -= beta * s2;
			qDot4 -= beta * s3;
		}

		// Integrate rate of change of quaternion to yield quaternion
		q[0] += qDot1 * SamplePeriod;
		q[1] += qDot2 * SamplePeriod;
		q[2] += qDot3 * SamplePeriod;
		q[3] += qDot4 * SamplePeriod;

		// Normalise quaternion
		recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
		q[0] *= recipNorm;
		q[1] *= recipNorm;
		q[2] *= recipNorm;
		q[3] *= recipNorm;  
		
//		q[1] = q[1];
//		q[2] = q[2];
//		q[3] = q[3];
		
		
	}
	
	private  double invSqrt(double x) {					
			return 1.0/ Math.sqrt(x);		
	}
	
	
	public  double[] arhs(double[] gyr, double[] acc, double[] mag, double SamplePeriod) {
//		System.out.print(arhs_state);
//		Log.v("SampleTime", " " + SamplePeriod);
//		Log.v("Acc: " , acc[0]+ " " + acc[1] + " " + acc[2]);
//		acc = Calibrarion.AccCal(acc, AccW, AccB);
    	if(arhs_state == 0){
			arhs_init(acc, gyr, mag);
			arhs_state++;
		}
		else {
			arhs_update(gyr, acc, mag, SamplePeriod);
//			Log.v("CalAcc: " , acc[0]+ " " + acc[1] + " " + acc[2]);
//			Log.d("Q: " , q[0]+ " " + q[1] + " " + q[2] + " " + q[3]);
		}
		return q;
	}

}
