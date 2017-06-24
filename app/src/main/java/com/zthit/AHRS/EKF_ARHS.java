package com.zthit.AHRS;

public class EKF_ARHS {
	
	private static double[][] R = new double[3][3];          //测量噪声矩阵
	private static double[][] P = new double[7][7];          //协方差矩阵
	private static double[][] Q = new double[7][7];          //协方差噪声矩阵
	private static double[] bias = new double[3];            //陀螺仪初始偏差
	private static double[] rates = new double[3];           //减去bias后的角速度
	private static double[] X = new double[7];               //状态向量,由quat和bias组成
	private static double[] X_Pred = new double[7];
	private static double[] X_Last = new double[7];
	private static double[] quat = new double[4];
	static double[] quat_out = new double[4];	
	static int arhs_state = 0;                              //0表示初始状态
	
	private static void ahrs_init(double []accel,double []gyro,double []mag){
		//姿态初始化
		double[] angle = new double[3];
		angle[0] = ARHS_Library.accel2phi(accel);
		angle[1] = ARHS_Library.accel2theta(angle[0], accel);
		angle[2] = ARHS_Library.mag2psi(mag, angle[0], angle[1]);
		quat = ARHS_Library.eulers2quat(angle);
		quat = Matrix_lib.norm_array(quat);
		//将静态时的陀螺仪值作为初始bias
		bias = gyro;
		//初始化状态变量
		double[] Xtemp = {quat[0],quat[1],quat[2],quat[3],bias[0],bias[1],bias[2]};
		X = Xtemp;
		X_Last = Xtemp;
		X_Pred = Xtemp;
		//初始化实际角速率
		rates[0] = 0;    rates[1] = 0;     rates[2] = 0;
		//初始化协方差矩阵P
	    double[][] Ptemp = {{1,0,0,0,0,0,0},
	                        {0,1,0,0,0,0,0},
	                        {0,0,1,0,0,0,0},
	                        {0,0,0,1,0,0,0},
	                        {0,0,0,0,0,0,0},
	                        {0,0,0,0,0,0,0},
	                        {0,0,0,0,0,0,0}};          //7*7矩阵
	    P = Ptemp;
	    //初始化协方差噪声矩阵
	    double omega1 = 0.5*Math.pow(10, -5);
	    double omega2 = Math.pow(10, -3);
	    double[][] Qtemp = {{omega1,0,0,0,0,0,0},
	    		            {0,omega1,0,0,0,0,0},
	    		            {0,0,omega1,0,0,0,0},
	    		            {0,0,0,omega1,0,0,0},
	    		            {0,0,0,0,omega2,0,0},
	    		            {0,0,0,0,0,omega2,0},
	    		            {0,0,0,0,0,0,omega2}};     //7*7矩阵
	    Q = Qtemp;
	    //初始化测量噪声矩阵
	    double[][] Rtemp = {{1.3*1.3,0,0},
	    		            {0,1.3*1.3,0},
	    		            {0,0,1.3*1.3}};            //3*3矩阵
	    R = Rtemp;
		
	}
	
	//EKF预测阶段
	private static void arhs_predict(double[] gyro, double dt) {
		double p = gyro[0] - bias[0];
		double q = gyro[1] - bias[1];
		double r = gyro[2] - bias[2];
		//更新状态转移矩阵
		double[][] F = {{0,-p,-q,-r, quat[1], quat[2], quat[3]},
				        {p, 0, r,-q,-quat[0], quat[3],-quat[2]},
				        {q,-r, 0, p,-quat[3],-quat[0], quat[1]},
				        {r, q,-p, 0, quat[2],-quat[1],-quat[0]},
				        {0, 0, 0, 0,    0   ,   0    ,    0   },
				        {0, 0, 0, 0,    0   ,   0    ,    0   },
				        {0, 0, 0, 0,    0   ,   0    ,    0   }};         //7*7矩阵
		F = Matrix_lib.num_mutli(F, 0.5*dt);
		double[][] I7 = Matrix_lib.IMatrix(7);
		F = Matrix_lib.matrix_add(F, I7);
		double[][] FT = Matrix_lib.matrix_tranpose(F);
		double[][] X_Last_temp = Matrix_lib.arrt2matrix(X_Last);
		//估计状态变量
		double[][] X_Pred_temp = Matrix_lib.matrix_multi(F , X_Last_temp);
		X_Pred = Matrix_lib.matrix2arry(X_Pred_temp);
		quat[0] = X_Pred[0];
		quat[1] = X_Pred[1];
		quat[2] = X_Pred[2];
		quat[3] = X_Pred[3];
		quat = Matrix_lib.norm_array(quat);
		X_Pred[0] = quat[0];
		X_Pred[1] = quat[1];
		X_Pred[2] = quat[2];
		X_Pred[3] = quat[3];
		//估计协方差矩阵
		P = Matrix_lib.matrix_multi(Matrix_lib.matrix_multi(F, P), FT);
		P = Matrix_lib.matrix_add(P, Matrix_lib.num_mutli(Q, dt));
	}
	
	private static void arhs_update(double[] accel, double[] mag) {
		//获取测量值z
		double roll = ARHS_Library.accel2phi(accel);
		double pitch = ARHS_Library.accel2theta(roll,accel);
		double yaw = ARHS_Library.mag2psi(mag, roll, pitch);
		double[] measure = {roll,pitch,yaw};
		
		//获取测量估计值y
		roll = ARHS_Library.quat2roll(quat);
		pitch = ARHS_Library.quat2pitch(quat);
		yaw = ARHS_Library.quat2yaw(quat);
		double[] estimate = {roll,pitch,yaw};
		
		double[] erro = {measure[0] - estimate[0],
				         measure[1] - estimate[1],
				         measure[2] - estimate[2]};           
		double[][] erroM = Matrix_lib.arrt2matrix(erro);      //3*1矩阵
		
		//获取测量矩阵H
		double[][] dcm = ARHS_Library.quat2Reb(quat);
		double q0 = quat[0];
		double q1 = quat[1];
		double q2 = quat[2];
		double q3 = quat[3];
		
		double phi_err = 2/(dcm[2][2]*dcm[2][2] + dcm[1][2]*dcm[1][2]);
		double theta_err = 2/Math.sqrt(1 - dcm[0][2]*dcm[0][2]);
		double psi_err = 2/(dcm[0][0]*dcm[0][0] + dcm[0][1]*dcm[0][1]);
		
		double[] H1 = {(q1 * dcm[2][2])                      * phi_err,
				       (q0 * dcm[2][2] + 2 * q1 * dcm[1][2]) * phi_err,
				       (q3 * dcm[2][2] + 2 * q2 * dcm[1][2]) * phi_err,
				       (q2 * dcm[2][2])                      * phi_err,
				        0,
				        0,
				        0,};
		double[] H2 = {q2 * theta_err,
				      -q3 * theta_err,
				       q0 * theta_err,
				      -q1 * theta_err,
				       0,
				       0,
				       0};
		double[] H3 = {(q3 * dcm[0][0])                      * psi_err,
				       (q2 * dcm[0][0])                      * psi_err,
				       (q1 * dcm[0][0] + 2 * q2 * dcm[0][1]) * psi_err,
				       (q0 * dcm[0][0] + 2 * q3 * dcm[0][1]) * psi_err,
				        0,
				        0,
				        0};
		
		double[][] H = {H1,H2,H3};      //3*7矩阵
		double[][] HT = Matrix_lib.matrix_tranpose(H);
		double[][] E1 = Matrix_lib.matrix_multi(H, P);
		double[][] E2 = Matrix_lib.matrix_multi(E1, HT);
		double[][] E = Matrix_lib.matrix_add(E2, R);
	    E = Matrix_lib.rev(E);
		double[][] K = Matrix_lib.matrix_multi(P, Matrix_lib.matrix_multi(HT, E));
		double[][] X_Pred_temp = Matrix_lib.arrt2matrix(X_Pred);
		double[][] X_Last_temp = Matrix_lib.matrix_add(X_Pred_temp, Matrix_lib.matrix_multi(K, erroM));
		X_Last = Matrix_lib.matrix2arry(X_Last_temp);
		double[][] M1 = Matrix_lib.matrix_multi(K, H);
		double[][] M = Matrix_lib.matrix_multi(M1, P);
		P = Matrix_lib.matrix_minus(P, M);
		X = X_Last;

	}
	
	public static double[] arhs(double[] gyr, double[] acc, double[] mag, double samplePeriod ) {
		if(arhs_state == 0){
			ahrs_init(acc, gyr, mag);
			quat_out[0] = X[0];  quat_out[1] = X[1];  quat_out[2] = X[2];  quat_out[3] = X[3]; 
			arhs_state = 1;
		}
		else {
			arhs_predict(gyr, samplePeriod);
			arhs_update(acc, mag);
			quat_out[0] = X[0];  quat_out[1] = X[1];  quat_out[2] = X[2];  quat_out[3] = X[3];
		}
		return quat_out;
		
	}

}
