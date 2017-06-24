package com.zthit.AHRS;

import MatrixLib.EigenvalueDecomposition;
import MatrixLib.Matrix;

public class MagCalibration {
//	private double[][] Mag;
	private double[][] OutMag;
	private int m;
	public Matrix Bias = new Matrix(3, 1, 0);
	public Matrix Scale = Matrix.identity(3, 3);
	
	
	public MagCalibration (double[][] Mag) {
//		this.Mag = Mag;
		this.m = Mag[0].length;               //获取数据长度
//		this.InNum = m/5;                     
//		this.dataFreq = InNum/dataNum;        //用于进行数据计算时的插值间距
		OutMag = new double[3][m];
		OutMag = Mag.clone();
//		for(int i = 0; i<m;i++){
////					Log.d("Mag: " , this.Mag[0][i]+ " " + this.Mag[1][i] + " " + this.Mag[2][i]);
//
//		}
	}
	
//	private void DataProcessing() {
//		//数据预处理，每5个数据求均值
//		double[][] TempMag = new double[3][InNum];
//		for (int i = 0; i < InNum; i++) {
//			TempMag[0][i] = Mag[0][5*i] + Mag[0][5*i+1] + Mag[0][5*i+2] + Mag[0][5*i+3] + Mag[0][5*i+4];
//			TempMag[1][i] = Mag[1][5*i] + Mag[1][5*i+1] + Mag[1][5*i+2] + Mag[1][5*i+3] + Mag[1][5*i+4];
//			TempMag[2][i] = Mag[2][5*i] + Mag[2][5*i+1] + Mag[2][5*i+2] + Mag[2][5*i+3] + Mag[2][5*i+4];
//			TempMag[0][i] = 0.2 * TempMag[0][i];
//			TempMag[1][i] = 0.2 * TempMag[1][i];
//			TempMag[2][i] = 0.2 * TempMag[2][i];	
////			Log.d("TempMag: " , TempMag[0][i]+ " " + TempMag[1][i] + " " + TempMag[2][i]);
//		}
//		OutMag = TempMag;
//	}
	
	private double[]  CalHardIron(){
		Matrix HardIronXtX = new Matrix(4, 4, 0);
		Matrix HardIronXtY = new Matrix(4, 1, 0);
		Matrix HardIronX = new Matrix(1, 4);
		Matrix HardIronY = new Matrix(1, 1);
		Matrix HardIronXT = new Matrix(4, 1);
		double[][] Bp = new double[3][m];
		Bp = OutMag.clone();
		
//		for (int i = 0; i < dataNum; i++) {
//			Bp[0][i] = OutMag[0][dataFreq*(i+1)-1];
//			Bp[1][i] = OutMag[1][dataFreq*(i+1)-1];
//			Bp[2][i] = OutMag[2][dataFreq*(i+1)-1];
////			Log.d("OutMag: " , OutMag[0][dataFreq*i]+ " " + OutMag[1][dataFreq*i] + " " + OutMag[2][dataFreq*i]);
////			Log.d("Bp: " , Bp[0][i]+ " " + Bp[1][i] + " " + Bp[2][i]);
//		}
		for (int i = 0; i < m; i++) {
			HardIronX.set(0, 0, Bp[0][i]);
			HardIronX.set(0, 1, Bp[1][i]);
			HardIronX.set(0, 2, Bp[2][i]);
			HardIronX.set(0, 3, 1);
//			HardIronX.print(4, 6);
			double y = Bp[0][i]*Bp[0][i] + Bp[1][i]*Bp[1][i] + Bp[2][i]*Bp[2][i];
			HardIronY.set(0, 0, y);
			HardIronXT = HardIronX.transpose();
//			HardIronXT.print(1, 6);
			HardIronXtX.plusEquals(HardIronXT.times(HardIronX));
			HardIronXtY.plusEquals(HardIronXT.times(HardIronY));

		}
//		HardIronXtX.print(4, 5);
//		HardIronXtY.print(1, 5);
		Matrix HardIronXtX_inv = HardIronXtX.inverse();
		Matrix HardSolution = HardIronXtX_inv.times(HardIronXtY);
		double[] V = new double[3];
		V[0] = 0.5*HardSolution.get(0, 0);
		V[1] = 0.5*HardSolution.get(1, 0);
		V[2] = 0.5*HardSolution.get(2, 0);
		for(int i = 0;i < m;i++){
			OutMag[0][i] = OutMag[0][i] - V[0];
			OutMag[1][i] = OutMag[1][i] - V[1];
			OutMag[2][i] = OutMag[2][i] - V[2];
		}
		return V;
						
	} 
	
//	private void CalSoftIron() {
////		Matrix SoftIronScale = Matrix.identity(3, 3);
//		Matrix SoftIronXtX = new Matrix(4, 4, 0);
//		Matrix SoftIronX = new Matrix(1,4);
//		Matrix SoftIronXT = new Matrix(4, 1);
//		double[][] Bp = new double[3][dataNum];
//		for (int i = 0; i < dataNum; i++) {
//			Bp[0][i] = OutMag[0][dataFreq*i];
//			Bp[1][i] = OutMag[1][dataFreq*i];
//			Bp[2][i] = OutMag[2][dataFreq*i];
////			Log.d("OutMag: " , OutMag[0][dataFreq*i]+ " " + OutMag[1][dataFreq*i] + " " + OutMag[2][dataFreq*i]);
////			Log.d("Bp: " , Bp[0][i]+ " " + Bp[1][i] + " " + Bp[2][i]);
//		}
//		for (int i = 0; i < dataNum;i++){
//			SoftIronX.set(0, 0, Bp[0][i]*Bp[0][i]);
//			SoftIronX.set(0, 1, Bp[1][i]*Bp[1][i]);
//			SoftIronX.set(0, 2, Bp[2][i]*Bp[2][i]);
//			SoftIronX.set(0, 3, 1);
//			SoftIronXT = SoftIronX.transpose();
//			SoftIronXtX.plusEquals(SoftIronXT.times(SoftIronX));
//		}
////		SoftIronXtX.print(4, 5);
//		EigenvalueDecomposition E = SoftIronXtX.eig();
//		Matrix eignVector = E.getV();
//		Matrix eignValue = E.getD();
////		eignValue.print(4, 5);
////		eignVector.print(4, 5);
//		double mineignValue = eignValue.get(0, 0);
//		int mincolVector = 0;
//		for(int i = 0; i < 4; i++){
//			if(mineignValue > eignValue.get(i, i)){
//				mineignValue = eignValue.get(i, i);
//				mincolVector = i;
//			}
//		}
//		Matrix SoftIronScaleInv2 = new Matrix(3, 3, 0);
//		SoftIronScaleInv2.set(0, 0, eignVector.get(0, mincolVector));
//		SoftIronScaleInv2.set(1, 1, eignVector.get(1, mincolVector));
//		SoftIronScaleInv2.set(2, 2, eignVector.get(2, mincolVector));
//		if(SoftIronScaleInv2.det() < 0){
//			SoftIronScaleInv2 = SoftIronScaleInv2.uminus();
//			eignVector.set(3, mincolVector, -eignVector.get(3, mincolVector));
//		}
////		SoftIronScaleInv2.print(3, 5);
//		Matrix SoftIronScaleInv = new Matrix(3, 3, 0);
//		double s0 = Math.abs(SoftIronScaleInv2.get(0, 0));
//		double s1 = Math.abs(SoftIronScaleInv2.get(1, 1));
//		double s2 = Math.abs(SoftIronScaleInv2.get(2, 2));
//		SoftIronScaleInv.set(0, 0, Math.sqrt(s0));
//		SoftIronScaleInv.set(1, 1, Math.sqrt(s1));
//		SoftIronScaleInv.set(2, 2, Math.sqrt(s2));
//		double detSoftIronScaleInv = SoftIronScaleInv.det();
//		detSoftIronScaleInv = Math.cbrt(detSoftIronScaleInv);
//		SoftIronScaleInv.timesEquals(1.0/detSoftIronScaleInv);
//		SoftIronScaleInv.print(3, 5);
//		W_inv = SoftIronScaleInv.getArrayCopy();
//	}
	
	private double[][] CalSoftIron() {
//		Matrix SoftIronScale = Matrix.identity(3, 3);
		Matrix SoftIronXtX = new Matrix(7, 7, 0);
		Matrix SoftIronX = new Matrix(1,7);
		Matrix SoftIronXT = new Matrix(7, 1);
		double[][] Bp = new double[3][m];
		Bp = OutMag.clone();
		
//		for (int i = 0; i < m; i++) {
//			Bp[0][i] = OutMag[0][dataFreq*(i+1)-1];
//			Bp[1][i] = OutMag[1][dataFreq*(i+1)-1];
//			Bp[2][i] = OutMag[2][dataFreq*(i+1)-1];
////			Log.d("OutMag: " , OutMag[0][dataFreq*i]+ " " + OutMag[1][dataFreq*i] + " " + OutMag[2][dataFreq*i]);
////			Log.d("Bp: " , Bp[0][i]+ " " + Bp[1][i] + " " + Bp[2][i]);
//		}
		for (int i = 0; i < m;i++){
			SoftIronX.set(0, 0, Bp[0][i]*Bp[0][i]);
			SoftIronX.set(0, 1, Bp[1][i]*Bp[1][i]);
			SoftIronX.set(0, 2, Bp[2][i]*Bp[2][i]);
			SoftIronX.set(0, 3, Bp[0][i]*Bp[1][i]);
			SoftIronX.set(0, 4, Bp[2][i]*Bp[0][i]);
			SoftIronX.set(0, 5, Bp[1][i]*Bp[2][i]);
			SoftIronX.set(0, 6, 1);
			SoftIronXT = SoftIronX.transpose();
			SoftIronXtX.plusEquals(SoftIronXT.times(SoftIronX));
		}
//		SoftIronXtX.print(7, 5);
		EigenvalueDecomposition E = SoftIronXtX.eig();
		Matrix eignVector = E.getV();
		Matrix eignValue = E.getD();
//		eignValue.print(4, 5);
//		eignVector.print(4, 5);
		double mineignValue = eignValue.get(0, 0);
		int mincolVector = 0;
		for(int i = 0; i < 4; i++){
			if(mineignValue > eignValue.get(i, i)){
				mineignValue = eignValue.get(i, i);
				mincolVector = i;
			}
		}
		Matrix SoftIronScaleInv2 = new Matrix(3, 3, 0);
		SoftIronScaleInv2.set(0, 0, eignVector.get(0, mincolVector));
		SoftIronScaleInv2.set(1, 1, eignVector.get(1, mincolVector));
		SoftIronScaleInv2.set(2, 2, eignVector.get(2, mincolVector));
		SoftIronScaleInv2.set(0, 1, 0.5*eignVector.get(3, mincolVector));
		SoftIronScaleInv2.set(1, 0, 0.5*eignVector.get(3, mincolVector));
		SoftIronScaleInv2.set(0, 2, 0.5*eignVector.get(4, mincolVector));
		SoftIronScaleInv2.set(2, 0, 0.5*eignVector.get(4, mincolVector));
		SoftIronScaleInv2.set(1, 2, 0.5*eignVector.get(5, mincolVector));
		SoftIronScaleInv2.set(2, 1, 0.5*eignVector.get(5, mincolVector));
		if(SoftIronScaleInv2.det() < 0){
			SoftIronScaleInv2 = SoftIronScaleInv2.uminus();
			eignVector.set(3, mincolVector, -eignVector.get(6, mincolVector));
		}
//		SoftIronScaleInv2.print(3, 5);
		EigenvalueDecomposition Es = SoftIronScaleInv2.eig();
		Matrix P = Es.getV();
		Matrix Diagnol = Es.getD();
//		P.print(3, 5);
//		Diagnol.print(3, 5);
		Matrix Pinv = P.inverse();
		double s0 = Math.abs(Diagnol.get(0, 0));
		double s1 = Math.abs(Diagnol.get(1, 1));
		double s2 = Math.abs(Diagnol.get(2, 2));
		Diagnol.set(0, 0, Math.sqrt(s0));
		Diagnol.set(1, 1, Math.sqrt(s1));
		Diagnol.set(2, 2, Math.sqrt(s2));
		
//		SoftIronScaleInv2.print(3, 5);
		Matrix SoftIronScaleInv = new Matrix(3, 3, 0);
		SoftIronScaleInv = (P.times(Diagnol)).times(Pinv);
		double detSoftIronScaleInv = SoftIronScaleInv.det();
		detSoftIronScaleInv = Math.cbrt(detSoftIronScaleInv);
		SoftIronScaleInv.timesEquals(1.0/detSoftIronScaleInv);
//		SoftIronScaleInv.print(3, 5);
		double[][] W = SoftIronScaleInv.getArrayCopy();
		Matrix OutMagM = new Matrix(OutMag);
		OutMagM = SoftIronScaleInv.times(OutMagM);
		OutMag = OutMagM.getArrayCopy();
//		for (int i = 0; i < dataNum; i++) {
//			Log.d("OutMag: " , OutMag[0][i]+ " " + OutMag[1][i] + " " + OutMag[2][i]);
////			Log.d("Bp: " , Bp[0][i]+ " " + Bp[1][i] + " " + Bp[2][i]);
//		}
		return W;
		
	}
	
	public void Calibration() {
//		DataProcessing();
		double[] HardBiasTemp = new double[3];
		double[][] W = new double[3][3];
		for(int i = 0; i < 20; i++){
			HardBiasTemp = CalHardIron().clone();
			W = CalSoftIron().clone();
			Matrix HardBiasMatrix = new Matrix(HardBiasTemp, 3);
			Matrix WMatrix = new Matrix(W);
			Bias.plusEquals((Scale.inverse()).times(HardBiasMatrix));
			Scale = Scale.times(WMatrix);			
		}


	}

}
