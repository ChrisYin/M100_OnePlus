package com.zthit.AHRS;

public class Matrix_lib {
	
	//*********************************************************************************************//
		//*********************************************************************************************//
		//******************************矩阵的相关运算*************************************************//
		public static double get_norm(double[] array) {
			double norm = 0;
			for(int i=0; i<array.length; i++){
				norm += array[i]*array[i];
			}
			norm = Math.sqrt(norm);
			return norm;
		}
		
		public static double get_norm(double[][] array) {
			double norm = 0;
			for(int i=0; i<array.length; i++){
				norm += array[i][0]*array[i][0];
			}
			norm = Math.sqrt(norm);
			return norm;
		}
		
		public static double[] norm_array(double[] array) {
			double norm = get_norm(array);
			for (int i = 0; i < array.length; i++) {
				array[i] = array[i]/norm;
			}
			return array;
		}
		
		public static double[][] norm_array(double[][] array) {
			double norm = get_norm(array);
			for (int i = 0; i < array.length; i++) {
				array[i][0] = array[i][0]/norm;
			}
			return array;
		}
		
		public static double[][] IMatrix(int m) {
			//定义一个m维的单位矩阵
			double[][] matrix = new double[m][m];
			for(int i=0; i<m; i++){
				for(int j=0; j<m;j++){
					if(i==j){
						matrix[i][j] = 1;
					}
					else{
						matrix[i][j] = 0;
					}
				}
			}
			return matrix;
		}
		
		public static double[][] num_mutli(double[][] matrix, double num) {
			//矩阵的数乘
			double[][] result = new double[matrix.length][matrix[0].length];
			for(int i=0; i<matrix.length; i++){
				for(int j=0; j<matrix[0].length;j++){
					result[i][j] = num * matrix[i][j];
				}
			}
			return result;
			
		}
		
		public static double[][] matrix_multi(double[][] matrixa, double[][] matrixb) {
			//矩阵相乘matrixa * matrixb
			 double[][] result = new double[matrixa.length][matrixb[0].length];  
	         for(int i=0; i<matrixa.length; i++) {  
	             for(int j=0; j<matrixb[0].length; j++) {  
	                 // i will complete this tomorrow @2012/09/17  
	                 result[i][j] = calculateSingleResult(matrixa, matrixb, i, j);   
	             }  
	         }  
	         return result;  
			
		}
		
		private static double calculateSingleResult(double[][] matrixa, double[][] matrixb, int row, int col) {
			//矩阵相乘时计算矩阵的某个元素值
	        double result = 0.0;  
	        for(int k=0; k<matrixa[0].length; k++) {  
	            result += matrixa[row][k] * matrixb[k][col];  
	        }  
	        return result;  
	    }  
		
		public static double[][] matrix_tranpose(double[][] matrix) {  
			//矩阵的转置
	        double[][] temp = new double[matrix[0].length][matrix.length];  
	        for (int i = 0; i < matrix.length; i++) {  
	            for (int j = 0; j < matrix[0].length; j++) {  
	                temp[j][i] = matrix[i][j];  
	            }  
	        }  
	        return temp;  
	    }  
		
		public static double[][] matrix_add(double[][] matrixa, double[][] matrixb) {
			//矩阵求和
			try {  
	            if ((matrixa.length != matrixb.length) || matrixa[0].length != matrixb[0].length)  
	                throw new Exception("矩阵维数不相等，不能相加");  
	        } catch (Exception e) {  
	            e.printStackTrace();  
	        }  
			int m = matrixa.length;
			int n = matrixa[0].length;
			double[][] Sum = new double[m][n];
			for(int i = 0; i<m; i++){
				for(int j = 0; j<n; j++){
					Sum[i][j] = matrixa[i][j] + matrixb[i][j];
				}
			}
			return Sum;
		}
		
		public static double[][] matrix_minus(double[][] matrixa, double[][] matrixb) {
			//矩阵求差matrixa - matrixb
			try {  
	            if ((matrixa.length != matrixb.length) || matrixa[0].length != matrixb[0].length)  
	                throw new Exception("矩阵维数不相等，不能相加");  
	        } catch (Exception e) {  
	            e.printStackTrace();  
	        }  
			int m = matrixa.length;
			int n = matrixa[0].length;
			double[][] Sum = new double[m][n];
			for(int i = 0; i<m; i++){
				for(int j = 0; j<n; j++){
					Sum[i][j] = matrixa[i][j] - matrixb[i][j];
				}
			}
			return Sum;
		}
		
	    
		public static double[] matrix2arry(double[][] matrix) {
			//把n行1列矩阵转换为一维数组
			int m = matrix.length;
			double[] arry = new double[m];
			for(int i=0; i<m; i++){
				arry[i] = matrix[i][0];
			}
			return arry;					
		}
		
		public static double[][] arrt2matrix(double[] arry) {
			//将1维数组转换为n行1列矩阵
			int m = arry.length;
			double[][] matrix = new double[m][1];
			for(int i=0; i<m; i++){
				 matrix[i][0] = arry[i];
			}
			return matrix;
		}
		
		public static double[][] chgCol(double[][] matrix,int chgfrom, int chgto) {
			//矩阵交换两列
			if ((chgfrom > matrix[0].length) || (chgto > matrix[0].length) || (chgfrom < 1)
					|| (chgto < 1)) {
				return null;
			}
			else{
				double[] temp = new double[matrix.length];
			    for (int i = 0; i < matrix.length; i++) {
			    	temp[i] = matrix[i][chgfrom - 1];
				    matrix[i][chgfrom - 1] = matrix[i][chgto - 1];
				    matrix[i][chgto - 1] = temp[i];
				
			    }
			    return matrix;
			}		
		}
		
		public static double[][] chgRow(double[][] matrix, int chgfrom, int chgto) {
			//矩阵交换两行
			if ((chgfrom > matrix.length) || (chgto > matrix.length) || (chgfrom < 1)
					|| (chgto < 1)) {
			//	System.out.println("\n矩阵交换两行 parameter error");
				matrix = null;
				return null;
			}
			else{
				double[] temp = new double[matrix[0].length];
			    for (int i = 0; i < matrix[0].length; i++) {
			    	temp[i] = matrix[chgfrom - 1][i];
				    matrix[chgfrom - 1][i] = matrix[chgto - 1][i];
				    matrix[chgto - 1][i] = temp[i];
				
			}
			    return matrix;		
			}
			
		}
		
		public static double[][] rev(double[][] matrix){
			//矩阵求逆
			 int len = matrix.length;
			 int wid = matrix[0].length;
			 if((len <= 0) || (wid <= 0) || (len != wid)){
				//System.out.println("\n矩阵求逆（线性变换法） The matrix is not legal");
				 return null;
			 }
			 //构造A|E矩阵
			 double[][] matrixA = new double[len][wid*2];
			 for (int i=1; i<=len; i++)
				 for (int j=1; j<=wid; j++){
				 matrixA[i-1][j-1] = matrix[i-1][j-1];
			 }
			 
			 for (int i=1; i<=len; i++)
				 for (int j=wid+1; j<=wid*2; j++){
				 if (i==j-wid){
					 matrixA[i-1][j-1] = 1;
				 }
				 else 
					 matrixA[i-1][j-1] = 0;
			 }
			 //开始对A|E矩阵作线性变换
			 for (int i=1; i<=len; i++){
				 double max = matrixA[i-1][i-1];
				 double absmax = Math.abs(max);
				 
				 int maxrow = i;
				 for (int j=i; j<=len; j++){
//					 double max2 = matrixA.getElement(j,i);
					 double max2 = matrixA[j-1][i-1];
					 double absmax2 = Math.abs(max2);
					 if (absmax2>absmax){
						 maxrow = j;
						 max = max2;
						 absmax = absmax2;
					 }
				 }
				 //System.out.println(max);
				 if (absmax==0){
					// System.out.println("\n矩阵不满秩");
					 return null;
				 }
				 chgRow(matrixA,i,maxrow);
				 //return matrixA;
				 //对第i行作归一化处理
				 for (int k=1; k<=wid*2; k++){
					 double temp = matrixA[i-1][k-1] / max;
					 matrixA[i-1][k-1] = temp;
				 }
				 
				 //对其他行进行处理
				 //对上三角阵进行处理
				 for (int m=1; m<i; m++){
					 double para = matrixA[m-1][i-1];
					 if (para!=0){
						 for (int n=1; n<=wid*2; n++){
							 double num = matrixA[m-1][n-1];
							 num = num - matrixA[i-1][n-1] * para;
							 matrixA[m-1][n-1] = num;
						 }
					 }
				 }

				 //对下三角阵进行处理
				 for (int m=i+1; m<=len; m++){
					 double para = matrixA[m-1][i-1];
					 if (para!=0){
						 for (int n=1; n<=wid*2; n++){
							 double num = matrixA[m-1][n-1];
							 num = num - matrixA[i-1][n-1]* para;
							 matrixA[m-1][n-1] = num;
						 }
					 }
				 }
				 //return matrixA;
			 }
			 /*
			 * 返回结果E|B矩阵
			 * */
			 double[][] matrixB = new double[len][wid];
			 for (int i = 1; i <= len; i++)
				 for (int j = wid+1; j <= wid*2; j++) {
					 double temp = matrixA[i-1][j-1];
					 matrixB[i-1][j-wid-1]=temp;
				 }
			 return matrixB;
		 }	

}
