package com.zthit.AHRS;

import MatrixLib.Matrix;

public class EKF {
	
//	private  double[] R = {0.002151 * 0.002151,
//			               0.002759 * 0.002759,
//			               0.009543 * 0.009543};          //测量噪声矩阵，9300的数据
	
	private  double[] R = {0.001483 * 0.001483,
                           0.002380 * 0.002380,
                           0.008150 * 0.008150};          //测量噪声矩阵,Nexus的数据
	
	private  Matrix P = new Matrix(7, 7);         //协方差矩阵
	private  Matrix Q = new Matrix(7, 7);          //协方差噪声矩阵
	private  Matrix C = new Matrix(1, 7);
	private  double[] bias = new double[3];            //陀螺仪偏差
//	private  double[] rates = new double[3];           //减去bias后的角速度
	private  Matrix X = new Matrix(7, 1);               //状态向量,由quat和bias组成
//	private  Matrix X_Pred = new Matrix(7, 1);
//	private  Matrix X_Last = new Matrix(7, 1);
	private  Matrix quat = new Matrix(4,1);
	private  Matrix quat_dot = new Matrix(4,1);
	private  Matrix I7 = Matrix.identity(7, 7);	
    public double[] quat_out = new double[4];	
	public int arhs_state = 0;                              //0表示初始状态
	private  double measure,estimate,erro;
	
//	private double[][] AccW = {{0.9799,0.0016,0.0041},{-0.0019,1.0208,0.0005},{0.0008,0.0009,1.0142}};
//	private double[] AccB = {0.0608,0.0063,-0.4984};
	
	private  void ahrs_init(double []accel,double []gyro,double []mag){
		//姿态初始化
		double[] angle = new double[3];
		angle[0] = ARHS_Library.accel2phi(accel);
		angle[1] = ARHS_Library.accel2theta(angle[0], accel);
		angle[2] = ARHS_Library.mag2psi(mag, angle[0], angle[1]);
		double[] quatArray = ARHS_Library.eulers2quat(angle);
		quatArray = Matrix_lib.norm_array(quatArray);
		quat = new Matrix(quatArray,4);
		//将静态时的陀螺仪值作为初始bias
		bias = gyro.clone();             //必须使用clone()方法，否则bias将指向gyro相同的内存空间，gyro的值改变bias的值也会对应改变
		//初始化状态变量，启动时的测量值作为初值
		double[] Xtemp = {quatArray[0],quatArray[1],quatArray[2],quatArray[3],bias[0],bias[1],bias[2]};
		X = new Matrix(Xtemp,7);
//		X_Last = X;
//		X_Pred = X;
		//初始化实际角速率
//		rates[0] = 0;    rates[1] = 0;     rates[2] = 0;
		//初始化协方差矩阵P
	    double[][] Ptemp = {{1,0,0,0,0,0,0},
	                        {0,1,0,0,0,0,0},
	                        {0,0,1,0,0,0,0},
	                        {0,0,0,1,0,0,0},
	                        {0,0,0,0,0,0,0},
	                        {0,0,0,0,0,0,0},
	                        {0,0,0,0,0,0,0}};          //7*7矩阵
	    P = new Matrix(Ptemp);
	    //初始化噪声协方差矩阵
	    double omega1 = 0.5*Math.pow(10, -8);
	    double omega2 = Math.pow(10, -3);
	    double[][] Qtemp = {{omega1,0,0,0,0,0,0},
	    		            {0,omega1,0,0,0,0,0},
	    		            {0,0,omega1,0,0,0,0},
	    		            {0,0,0,omega1,0,0,0},
	    		            {0,0,0,0,omega2,0,0},
	    		            {0,0,0,0,0,omega2,0},
	    		            {0,0,0,0,0,0,omega2}};     //7*7矩阵
	    Q = new Matrix(Qtemp);
	    //初始化测量噪声矩阵
		
	}
	
	//EKF预测阶段
	private  void arhs_predict(double[] gyro, double dt) {
		double p = gyro[0] - bias[0];
		double q = gyro[1] - bias[1];
		double r = gyro[2] - bias[2];
//		Log.v("rate", p + " " + q + " " + r + " ");
		double[][] omegaArray = {{0,-p,-q,-r},
			                     {p, 0, r,-q},
			                     {q,-r, 0, p},
			                     {r, q,-p, 0}};
		Matrix omega = new Matrix(omegaArray);
		omega.timesEquals(0.5);
//		Log.v("q: " , quat.get(0, 0)+ " " + quat.get(1, 0) + " " + quat.get(2, 0) + " " + quat.get(3, 0));
		quat_dot = omega.times(quat);
		quat.plusEquals(quat_dot.times(dt));
//		Log.v("Q: " , quat.get(0, 0)+ " " + quat.get(1, 0) + " " + quat.get(2, 0) + " " + quat.get(3, 0));
		double[][] quatArray = quat.getArray();
		quatArray = Matrix_lib.norm_array(quatArray);
		quat = new Matrix(quatArray);
		
		//更新状态转移矩阵
		double[][] FArray = {{0,-p,-q,-r, quatArray[1][0], quatArray[2][0], quatArray[3][0]},
				             {p, 0, r,-q,-quatArray[0][0], quatArray[3][0],-quatArray[2][0]},
				             {q,-r, 0, p,-quatArray[3][0],-quatArray[0][0], quatArray[1][0]},
				             {r, q,-p, 0, quatArray[2][0],-quatArray[1][0],-quatArray[0][0]},
				             {0, 0, 0, 0,         0      ,        0       ,        0       },
				             {0, 0, 0, 0,         0      ,        0       ,        0       },
				             {0, 0, 0, 0,         0      ,        0       ,        0       }};         //7*7矩阵
		Matrix F = new Matrix(FArray);
		F.timesEquals(0.5*dt);
		F.plusEquals(I7);
        Matrix FT = F.transpose();
        P = (F.times(P)).times(FT);
		P.plusEquals(Q.times(dt));
	}
	
	private  Matrix get_dphi_dq(Matrix q){
		double[][] qArray = q.getArray();
		double[] qArrayD1 = Matrix_lib.matrix2arry(qArray);
		double[][] dcm = ARHS_Library.quat2Reb(qArrayD1);
		double phi_err = 2/(dcm[2][2]*dcm[2][2] + dcm[1][2]*dcm[1][2]);
		double q0 = quat.get(0, 0);
		double q1 = quat.get(1, 0);
		double q2 = quat.get(2, 0);
		double q3 = quat.get(3, 0);
		double[] CArray = {(q1 * dcm[2][2])                      * phi_err,
			               (q0 * dcm[2][2] + 2 * q1 * dcm[1][2]) * phi_err,
			               (q3 * dcm[2][2] + 2 * q2 * dcm[1][2]) * phi_err,
			               (q2 * dcm[2][2])                      * phi_err,
			               0,
			               0,
			               0,};
		Matrix CM = new Matrix(CArray,1);    //1*7列矩阵
		return CM;
	}
	
	private  Matrix get_dtheta_dp(Matrix q){
		double[][] qArray = q.getArray();
		double[] qArrayD1 = Matrix_lib.matrix2arry(qArray);
		double[][] dcm = ARHS_Library.quat2Reb(qArrayD1);
		double theta_err = 2/Math.sqrt(1 - dcm[0][2]*dcm[0][2]);
		double q0 = quat.get(0, 0);
		double q1 = quat.get(1, 0);
		double q2 = quat.get(2, 0);
		double q3 = quat.get(3, 0);
		double[] CArray = {q2 * theta_err,
			         	   -q3 * theta_err,
			         	   q0 * theta_err,
			         	   -q1 * theta_err,
			         	   0,
			         	   0,
			         	   0};
		Matrix CM = new Matrix(CArray,1);    //1*7列矩阵
		return CM;
		
	}
	
	private  Matrix get_dpsi_dq(Matrix q) {
		double[][] qArray = q.getArray();
		double[] qArrayD1 = Matrix_lib.matrix2arry(qArray);
		double[][] dcm = ARHS_Library.quat2Reb(qArrayD1);
		double psi_err = 2/(dcm[0][0]*dcm[0][0] + dcm[0][1]*dcm[0][1]);
		double q0 = quat.get(0, 0);
		double q1 = quat.get(1, 0);
		double q2 = quat.get(2, 0);
		double q3 = quat.get(3, 0);
		double[] CArray = {(q3 * dcm[0][0])                      * psi_err,
			       		   (q2 * dcm[0][0])                      * psi_err,
			       		   (q1 * dcm[0][0] + 2 * q2 * dcm[0][1]) * psi_err,
			       		   (q0 * dcm[0][0] + 2 * q3 * dcm[0][1]) * psi_err,
			       		   0,
			       		   0,
			       		   0};
		Matrix CM = new Matrix(CArray,1);    //1*7列矩阵
		return CM;
	}
	
	private  void arhs_update() {
		//获取测量值z
		Matrix CT = C.transpose();
		Matrix EM = (C.times(P)).times(CT);    //乘出来的结果是一个scale
		double E = EM.get(0, 0);
		E = E + R[arhs_state - 1];
		Matrix K = (P.times(CT)).times(1/E);
	    Matrix P_dot = (K.times(C)).times(P);
	    P.minusEquals(P_dot);
	    X.setMatrix(0, 3, 0, 0, quat);
	    X.set(4, 0, bias[0]);
	    X.set(5, 0, bias[1]);
	    X.set(6, 0, bias[2]);
	    X.plusEquals(K.times(erro));
	    quat.set(0, 0, X.get(0, 0));
	    quat.set(1, 0, X.get(1, 0));
	    quat.set(2, 0, X.get(2, 0));
	    quat.set(3, 0, X.get(3, 0));
	    bias[0] = X.get(4, 0);
	    bias[1] = X.get(5, 0);
	    bias[2] = X.get(6, 0);
//		Log.v("Bias: " , bias[0]+ " " + bias[1] + " " + bias[2]);
	    double[][] quatArray = quat.getArray();
		quatArray = Matrix_lib.norm_array(quatArray);
		quat = new Matrix(quatArray);
	}
	
	public  double[] arhs(double[] gyr, double[] acc, double[] mag, double samplePeriod ) {
//		acc = Calibrarion.AccCal(acc, AccW, AccB);
//		Log.v("Acc: " , acc[0]+ " " + acc[1] + " " + acc[2]);
		if(arhs_state == 0){
			ahrs_init(acc, gyr, mag);		
		}
		else {
			arhs_predict(gyr, samplePeriod);
//			Log.v("q: " , quat.get(0, 0)+ " " + quat.get(1, 0) + " " + quat.get(2, 0) + " " + quat.get(3, 0));
			if(arhs_state == 1){
				measure = ARHS_Library.accel2phi(acc);
				double[][] quatArray = quat.getArray();
				double[] quatArrayD1 = Matrix_lib.matrix2arry(quatArray);
				estimate = ARHS_Library.quat2roll(quatArrayD1);
				C = get_dphi_dq(quat);		
			}
			else if (arhs_state == 2){
				measure = ARHS_Library.accel2theta(ARHS_Library.accel2phi(acc),acc);
				double[][] quatArray = quat.getArray();
				double[] quatArrayD1 = Matrix_lib.matrix2arry(quatArray);
				estimate = ARHS_Library.quat2pitch(quatArrayD1);
				C = get_dtheta_dp(quat);
			}
			else if (arhs_state == 3){
				double[][] quatArray = quat.getArray();
				double[] quatArrayD1 = Matrix_lib.matrix2arry(quatArray);
//				double phi = ARHS_Library.quat2roll(quatArrayD1);
//				double theta = ARHS_Library.quat2pitch(quatArrayD1);
				double phi = ARHS_Library.accel2phi(acc);
				double theta = ARHS_Library.accel2theta(phi, acc);
				measure = ARHS_Library.mag2psi(mag, phi, theta);
				estimate = ARHS_Library.quat2yaw(quatArrayD1);
				C = get_dpsi_dq(quat);
			}
			erro = measure - estimate;
			
			if(Math.abs(erro) > Math.PI){
				if(measure < 0){
					erro = 2*Math.PI + erro;
				}
				else{
					erro = erro - 2*Math.PI;
				}
			}
			
//			Log.v("erro"," " + erro);
//			Log.v("q: " , quat.get(0, 0)+ " " + quat.get(1, 0) + " " + quat.get(2, 0) + " " + quat.get(3, 0));
			arhs_update();
//			Log.v("Q: " , quat.get(0, 0)+ " " + quat.get(1, 0) + " " + quat.get(2, 0) + " " + quat.get(3, 0));
		}
		arhs_state++;
		if(arhs_state == 4){
			arhs_state = 1;
		}
		quat_out[0] = quat.get(0, 0); 
		quat_out[1] = quat.get(1, 0); 
		quat_out[2] = quat.get(2, 0); 
		quat_out[3] = quat.get(3, 0); 
		return quat_out;
		
	}

}
