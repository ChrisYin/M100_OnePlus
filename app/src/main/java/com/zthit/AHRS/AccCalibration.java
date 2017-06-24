package com.zthit.AHRS;

import MatrixLib.Matrix;


/****************************************************************************************
 * OutAcc的前六组值分别对应Z轴的最大值，Z轴的最小值，X轴的最大值，X轴的最小值，Y轴的最大值
 * Y轴的最小值。
 ***************************************************************************************/
public class AccCalibration {
	private double[][] Acc;
	private double[][] OutAcc; 
    private int MeanNum = 50;
    private int dataNum;
    private double lambda = 1;
    private double kl = 0.01;
    double tol = Math.pow(10, -9);
    double Rold = Math.pow(10, 6);
    int itr = 200;
    double Mxx0 = 1, Mxy0 = 0, Mxz0 = 0, Myy0 = 1, Myz0 = 0, Mzz0 = 1;
    double Bx0 = 0, By0 = 0, Bz0 = 0;
//    public Matrix M = new Matrix(3, 3);
//    public Matrix B = new Matrix(3, 1);
    public double[][] M = new double[3][3];
    public double[] B = new double[3];
	
	public  AccCalibration(double[][] Acc) {
		this.Acc = Acc;
		this.dataNum = Acc[0].length/MeanNum;
		this.OutAcc = new double[3][dataNum];
//	    Log.d("dataNum:", this.dataNum + "  " + this.Acc[0].length);
	}
	
	private void DataProcessing(){
		//将输入的加速度计的值，做均值处理，并把前六组分别保存为各个轴的最大和最小值
		for (int i = 0; i < dataNum; i++){
			OutAcc[0][i] = 0;
			OutAcc[1][i] = 0;
			OutAcc[2][i] = 0;
			for (int j = 0; j < MeanNum; j++){
				OutAcc[0][i] = OutAcc[0][i] + Acc[0][i*MeanNum + j];
				OutAcc[1][i] = OutAcc[1][i] + Acc[1][i*MeanNum + j];
				OutAcc[2][i] = OutAcc[2][i] + Acc[2][i*MeanNum + j];
			}
			OutAcc[0][i] = OutAcc[0][i]/MeanNum;
			OutAcc[1][i] = OutAcc[1][i]/MeanNum;
			OutAcc[2][i] = OutAcc[2][i]/MeanNum;
//			Log.d("OutAcc:", OutAcc[0][i] + "  " + OutAcc[1][i] + "  " + OutAcc[2][i]);
		}
		//把数据单位转换为单位g
//		double Max_z = OutAcc[2][0];  	 double Min_z = OutAcc[2][1];
//		double Max_x = OutAcc[0][2]; 	 double Min_x = OutAcc[0][3];
//		double Max_y = OutAcc[1][4];     double Min_y = OutAcc[1][5];
//		double Zero_x = Math.abs((Max_x - Min_x)/2);
//		double Zero_y = Math.abs((Max_y - Min_y)/2);
//		double Zero_z = Math.abs((Max_z - Min_z)/2);
//		for (int i = 0; i < dataNum; i++){
//			OutAcc[0][i] = OutAcc[0][i]/Zero_x;
//			OutAcc[1][i] = OutAcc[1][i]/Zero_y;
//			OutAcc[2][i] = OutAcc[2][i]/Zero_z;
//		}
		
	}
	
	private double f(double Vx, double Vy, double Vz, double Mxx, double Mxy, double Mxz, 
	         double Myy, double Myz, double Mzz, double Bx, double By,double Bz) {
		double value;
		value = Math.pow(9.81, 2) - Math.pow(Mxx*(Vx - Bx) + Mxy*(Vy - By) + Mxz*(Vz - Bz), 2) 
				- Math.pow(Mxy*(Vx - Bx) + Myy*(Vy - By) + Myz*(Vz - Bz), 2)
				- Math.pow(Mxz*(Vx - Bx) + Myz*(Vy - By) + Mzz*(Vz - Bz), 2);
		return value;
	}

	private double f1(double Vx, double Vy, double Vz, double Mxx, double Mxy, double Mxz, 
			double Bx, double By,double Bz){
		double value;
		value = -2*(Vx - Bx)*(Mxx*(Vx - Bx) + Mxy*(Vy - By) + Mxz*(Vz - Bz));
		return value;
	}

	private double f2(double Vx, double Vy, double Vz, double Mxx, double Mxy, double Mxz, 
			double Myy, double Myz,  double Bx, double By,double Bz){
		double value;
		value = -2*(Vy - By)*(Mxx*(Vx - Bx) + Mxy*(Vy - By) + Mxz*(Vz - Bz)) - 2*(Vx - Bx)*(Mxy*(Vx - Bx) + Myy*(Vy - By) + Myz*(Vz - Bz));
		return value;
	}

	private double f3(double Vx, double Vy, double Vz, double Mxx, double Mxy, double Mxz, 
             	double Myz, double Mzz, double Bx, double By,double Bz){
		double value;
		value = -2*(Vx - Bx)*(Mxz*(Vx - Bx) + Myz*(Vy - By) + Mzz*(Vz - Bz)) - 2*(Vz - Bz)*(Mxx*(Vx - Bx) + Mxy*(Vy - By) + Mxz*(Vz - Bz));
		return value;		
	}

	private double f4(double Vx, double Vy, double Vz, double Mxy, double Myy, double Myz, 
			double Bx, double By,double Bz){
		double value;
		value = -2*(Vy - By)*(Mxy*(Vx - Bx) + Myy*(Vy - By) + Myz*(Vz - Bz));
		return value;
	}

	private double f5(double Vx, double Vy, double Vz, double Mxy, double Mxz,double Myy,  
			double Myz,double Mzz,double Bx, double By,double Bz){
		double value;
		value = -2*(Vy - By)*(Mxz*(Vx - Bx) + Myz*(Vy - By) + Mzz*(Vz - Bz)) - 2*(Vz - Bz)*(Mxy*(Vx - Bx) + Myy*(Vy - By) + Myz*(Vz - Bz));
		return value;
	}

	private double f6(double Vx, double Vy, double Vz, double Mxz, double Myz, double Mzz, 
			double Bx, double By,double Bz){
		double value;
		value = -2*(Vz - Bz)*(Mxz*(Vx - Bx) + Myz*(Vy - By) + Mzz*(Vz - Bz));
		return value;
	}

	private double f7(double Vx, double Vy, double Vz, double Mxx, double Mxy, double Mxz, 
			double Myy, double Myz, double Mzz, double Bx, double By,double Bz){
		double value;
		value = 2*Mxx*(Mxx*(Vx - Bx) + Mxy*(Vy - By) + Mxz*(Vz - Bz)) + 2*Mxy*(Mxy*(Vx - Bx) + Myy*(Vy - By) + Myz*(Vz - Bz)) 
		+ 2*Mxz*(Mxz*(Vx - Bx) + Myz*(Vy - By) + Mzz*(Vz - Bz));
		return value;
	}

	private double f8(double Vx, double Vy, double Vz, double Mxx, double Mxy, double Mxz, 
			double Myy, double Myz, double Mzz, double Bx, double By,double Bz){
		double value;
		value = 2*Mxy*(Mxx*(Vx - Bx) + Mxy*(Vy - By) + Mxz*(Vz - Bz)) + 2*Myy*(Mxy*(Vx - Bx) + Myy*(Vy - By) + Myz*(Vz - Bz)) 
				+ 2*Myz*(Mxz*(Vx - Bx) + Myz*(Vy - By) + Mzz*(Vz - Bz));
		return value;
	}

	private double f9(double Vx, double Vy, double Vz, double Mxx, double Mxy, double Mxz, 
			double Myy, double Myz, double Mzz, double Bx, double By,double Bz) {
		double value;
		value = 2*Mxz*(Mxx*(Vx - Bx) + Mxy*(Vy - By) + Mxz*(Vz - Bz)) + 2*Myz*(Mxy*(Vx - Bx) + Myy*(Vy - By) + Myz*(Vz - Bz)) 
				+ 2*Mzz*(Mxz*(Vx - Bx) + Myz*(Vy - By) + Mzz*(Vz - Bz));
		return value;
	}

	private void Caculation(){
		double[] Vx = OutAcc[0].clone();
		//Matrix Vxx = new Matrix(Vx, 1);
		//Vxx.print(1, 5);
		double[] Vy = OutAcc[1].clone();
		//Matrix Vyy = new Matrix(Vy, 1);
		//Vyy.print(1, 5);
		double[] Vz = OutAcc[2].clone();
		int m = Vx.length;
		Matrix R = new Matrix(m, 1, 0);
		//double R;
		Matrix J = new Matrix(1, 9, 0);
		double[] v = {Mxx0, Mxy0, Mxz0, Myy0, Myz0, Mzz0, Bx0, By0, Bz0};
		double[] vold = new double[v.length];
		//double[] vold;
		Matrix vm = new Matrix(v, v.length);

		for(int n = 0; n < itr; n++){
			Matrix H = new Matrix(9, 9, 0);
			Matrix D = new Matrix(9,1,0);
			for(int i = 0; i < m; i++ ){
				R.set(i, 0, f(Vx[i], Vy[i], Vz[i], Mxx0, Mxy0, Mxz0, Myy0, Myz0, Mzz0, Bx0, By0, Bz0));
//				System.out.print(R.get(i, 0));
				J.set(0, 0, f1(Vx[i], Vy[i], Vz[i], Mxx0, Mxy0, Mxz0, Bx0, By0, Bz0));
				J.set(0, 1, f2(Vx[i], Vy[i], Vz[i], Mxx0, Mxy0, Mxz0, Myy0, Myz0, Bx0, By0, Bz0));
				J.set(0, 2, f3(Vx[i], Vy[i], Vz[i], Mxx0, Mxy0, Mxz0, Myz0, Mzz0, Bx0, By0, Bz0));
				J.set(0, 3, f4(Vx[i], Vy[i], Vz[i], Mxy0, Myy0, Myz0, Bx0, By0, Bz0));
				J.set(0, 4, f5(Vx[i], Vy[i], Vz[i], Mxy0, Mxz0, Myy0, Myz0, Mzz0, Bx0, By0, Bz0));
				J.set(0, 5, f6(Vx[i], Vy[i], Vz[i], Mxz0, Myz0, Mzz0, Bx0, By0, Bz0));
				J.set(0, 6, f7(Vx[i], Vy[i], Vz[i], Mxx0, Mxy0, Mxz0, Myy0, Myz0, Mzz0, Bx0, By0, Bz0));
				J.set(0, 7, f8(Vx[i], Vy[i], Vz[i], Mxx0, Mxy0, Mxz0, Myy0, Myz0, Mzz0, Bx0, By0, Bz0));
				J.set(0, 8, f9(Vx[i], Vy[i], Vz[i], Mxx0, Mxy0, Mxz0, Myy0, Myz0, Mzz0, Bx0, By0, Bz0));
				//		J.print(9, 5);
				Matrix JT = J.transpose();
				H.plusEquals(JT.times(J));
				D.plusEquals(JT.times(R.get(i, 0)));				
			}
			double Rnew = R.normF();
//			System.out.print(Rnew);
			H = H.inverse();
			//	H.print(9, 5);
			vm.plusEquals((H.times(D)).times(-lambda));
//			vm.print(9, 6);
	
			for (int i = 0; i < v.length; i++){
				v[i] = vm.get(i, 0);
			}
	
			if(Rnew <= Rold){
				lambda = lambda - kl * lambda;
			}
			else {
				lambda = kl*lambda;
			}
	
			 //收敛准则判定
			if(n > 1){
				double[] vd = new double[v.length];
				double[] vs = new double[v.length];
				for(int i = 0; i < v.length; i++){
					vd[i] = v[i] - vold[i];
					vs[i] = v[i] + vold[i];
					//			System.out.println(v[i]);
					//			System.out.println(vold[i]);
					//			System.out.println(vs[i]);
			
				}
				double temp = getMaxAbsValue(vs);
//				System.out.println(temp);
				for(int i = 0; i < v.length; i++){
					vd[i] = 2 * vd[i] / temp;
//					System.out.println(vd[i]);
				}
				temp = Math.abs(getMaxValue(vd));
				System.out.println(temp);
				if (temp <= tol){
					System.out.print(n);
					break;
				}	
			}
			
			
			Mxx0 = v[0];
			Mxy0 = v[1];
			Mxz0 = v[2];
			Myy0 = v[3];
			Myz0 = v[4];
			Mzz0 = v[5];
			Bx0 = v[6];
			By0 = v[7];
			Bz0 = v[8];
			vold = v.clone();
			Rold = Rnew;
		}
//		M.set(0, 0, Mxx0);   M.set(0, 1, Mxy0);   M.set(0, 2, Mxz0);
//		M.set(1, 0, Mxy0);   M.set(1, 1, Myy0);   M.set(1, 2, Myz0);
//		M.set(2, 0, Mxz0);   M.set(2, 1, Myz0);   M.set(2, 2, Mzz0);
//		B.set(0, 0, Bx0);
//		B.set(1, 0, By0);
//		B.set(2, 0, Bz0);
//		M.print(3, 5);
//		B.print(1, 5);
		M[0][0] = Mxx0;     M[0][1] = Mxy0;    M[0][2] = Mxz0;
		M[1][0] = Mxy0;     M[1][1] = Myy0;    M[1][2] = Myz0;
		M[2][0] = Mxz0;     M[2][1] = Myz0;    M[2][2] = Mzz0;
		B[0] = Bx0;
		B[1] = By0;
		B[2] = Bz0;

		//for(int i = 0; i < OutAcc[0].length; i++){
		//	Matrix Cal = new Matrix(3,1);
		//	Cal.set(0, 0, OutAcc[0][i]);
		//	Cal.set(1, 0, OutAcc[1][i]);
		//	Cal.set(2, 0, OutAcc[2][i]);
		//	Cal = M.times(Cal.minus(B));
		//	double g = Cal.normF();
		//	System.out.println(g);
		//}
	}

	private double getMaxAbsValue(double[] A) {
		double temp = Math.abs(A[0]);
		for(int i = 0; i < A.length; i++){
			if(temp < Math.abs(A[i])){
				temp = Math.abs(A[i]);
			}
		}
		return temp;
	}

	private double getMaxValue(double[] A) {
		double temp = A[0];
		for(int i = 0; i < A.length; i++){
			if(temp < A[i]){
				temp = A[i];
			}
		}
		return temp;
	}
	
	public void Calibration() {
		DataProcessing();
		Caculation();
	}
	

}
