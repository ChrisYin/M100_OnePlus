package com.zthit.AHRS;

public class Calibrarion {
	static double accel_alpha = 0.8;
	
	public static double[] AccCal(double[] acc,double[][] W, double[] B){
		double[] acc_c = new double[3];
		acc_c = acc.clone();
		acc_c[0] = acc_c[0] - B[0];
		acc_c[1] = acc_c[1] - B[1];
		acc_c[2] = acc_c[2] - B[2];
		acc_c[0] = W[0][0] * acc_c[0] + W[0][1] * acc_c[1] + W[0][2] * acc_c[2];
		acc_c[1] = W[1][0] * acc_c[0] + W[1][1] * acc_c[1] + W[1][2] * acc_c[2];
		acc_c[2] = W[2][0] * acc_c[0] + W[2][1] * acc_c[1] + W[2][2] * acc_c[2];
		return acc_c;
	}
	
	public static double[] GyroCalZ(double[] gyro, double[] B){
		double[] gyro_c = new double[3];
		gyro_c = gyro.clone();
		gyro_c[0] = gyro_c[0] - B[0];
		gyro_c[1] = gyro_c[1] - B[1];
		gyro_c[2] = gyro_c[2] - B[2];
		return gyro_c;
	}
	
	public static double[] Acc_Filter(double[] filtered, double[] raw){
		 filtered[0] = filtered[0] * accel_alpha + raw[0] * (1 - accel_alpha);
	     filtered[1] = filtered[1] * accel_alpha + raw[1] * (1 - accel_alpha);
	     filtered[2] = filtered[2] * accel_alpha + raw[2] * (1 - accel_alpha);
	     return filtered;
	}

}
