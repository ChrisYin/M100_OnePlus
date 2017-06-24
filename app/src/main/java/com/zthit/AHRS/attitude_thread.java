package com.zthit.AHRS;

import java.io.File;
import java.io.FileOutputStream;
import java.text.DecimalFormat;

import android.os.Environment;
import android.util.Log;



public class attitude_thread extends Thread {
	
	//使用不同算法的时候注意对run中的对应修改以及threadPause中的对应修改。。。
	//因为arhs_state是static变量，如果不修改回去则会出现不能还原的初始0值
	private File dir =  Environment.getExternalStorageDirectory();
	private String FileName_attitude = "attitude_data.txt";
	private String Attitude_message = new String();
	
	private double[][] AccM = new double[3][3];
	private double[] AccB = new double[3];
	private double[][] MagW = new double[3][3];
	private double[] MagV = new double[3];
	
    public double[] acc = new double[3];
    public double[] gyr = new double[3];
    public double[] mag = new double[3];
    public double[] q = new double[4];
    public double[] acc_c = new double[3];    // 为了避免影响到读取的测量值，对每组传感器的值都赋给到一个新的变量中输入到姿态算法中去
    public double[] gyr_c = new double[3];
    public double[] mag_c = new double[3];
    public double[] acc_filter = new double[3];
//    double samplePeriod;
//    double time_0,time_1;
   // double[] Euler_angle = new double[3];
    private boolean isRun = true;
    private boolean isWait = true;
	final  private double NS2S = 1.0/1000000000.0;         //纳秒转毫秒
	private boolean isStart = false;
	private double startTime; 
//	EKF ARHS = new EKF();
	Madgwick_AHRS ARHS = new Madgwick_AHRS();
//	Mahnoy_ARHS_Nomag ARHS = new Mahnoy_ARHS_Nomag();
//  Mahony_ARHS ARHS = new Mahony_ARHS();
    
    public attitude_thread(double[] acc1, double[] gyr1, double[] mag1,double[][] AccM,double[] AccB,double[][] MagW, double[] MagV){
    	this.gyr = gyr1;    //这样的赋值其实代表的是指针，对应同一个内存空间
    	this.acc = acc1;	
    	this.mag = mag1;
    	this.AccM = AccM;
    	this.AccB = AccB;
    	this.MagW = MagW;
    	this.MagV = MagV;
    	
//		this.samplePeriod = samplePeriod1;
    }
    
	public void run() {
		super.run();
		this.setName("GetAttitude");			
			while(isRun){				
				try {
					synchronized (this) {
						while(isWait){				
							wait();
						} 				
					}
					if(!isStart){
						startTime = System.nanoTime();
						isStart = true;
						acc_filter = acc.clone();
//						Log.v("MagW: ",  MagW[0][0] + " " + MagW[1][1] + " " + MagW[2][2]);
//						Log.v("AccM: ",  AccM[0][0] + " " + AccM[1][1] + " " + AccM[2][2]);
					}
					else {
						double samplePeriod = (System.nanoTime() - startTime) * NS2S;  //获取两次update之间的时间,作为算法中的时间间隔
						startTime = System.nanoTime();
//						Log.i("acc", acc[0]+" " + acc[1]+ " " + acc[2]);
						acc_c = (Calibrarion.AccCal(acc, AccM, AccB)).clone();      //校准角速度计的值
//						Log.i("acc_c0", acc_c[0]+" " + acc_c[1]+ " " + acc_c[2]);
//						Log.i("acc_filter1", acc_filter[0]+" " + acc_filter[1]+ " " + acc_filter[2]);
						acc_c = (Calibrarion.Acc_Filter(acc_filter, acc_c)).clone();
//						Log.i("acc_c", acc_c[0]+" " + acc_c[1]+ " " + acc_c[2]);
//						Log.i("acc_filter2", acc_filter[0]+" " + acc_filter[1]+ " " + acc_filter[2]);
//			    		Log.i("Time", " " + samplePeriod);
						gyr_c = gyr.clone();
						mag_c = mag.clone();
						DecimalFormat df = new DecimalFormat("####.######");
						
						
						Attitude_message = "Attitude" + " " + df.format(samplePeriod);
						attitude_data_format(gyr_c);
						attitude_data_format(acc_c);
						attitude_data_format(mag_c);
						
						q = ARHS.arhs(gyr_c, acc_c, mag_c, samplePeriod);													
//						q = ARHS.arhs(gyr, acc, samplePeriod);
						double[] euler = ARHS_Library.quat2eulers(q);
						
						attitude_data_format(euler);						
						Attitude_message = Attitude_message + "\n";
//						Log.v("Attitude: " , Attitude_message);
						writeFileSdcard(Attitude_message, FileName_attitude);
						sleep(10);
			    
						
//			    		Log.d("q: " , q[0]+ " " + q[1] + " " + q[2] + " " + q[3]);
					}
			    }catch (InterruptedException e) {
				// TODO Auto-generated catch block
				    e.printStackTrace();
			    }	
		    }
		}					
//	}
				
	//线程终止
	public void threadStop() {
		isRun = false;
		isStart = false;
	}
	
	//线程暂定
	public void threadPause() {
		isWait = true;
		isStart = false;
        ARHS.arhs_state = 0;
	}
	
	//线程唤醒
	public synchronized void threadResume() {
		isWait = false;
		notify();
	}
	
	public void writeFileSdcard(String message,String Filename) {
		
		try {
	        //FileOutputStream fout = openFileOutput(fileName, Context.MODE_APPEND);
	    	boolean append = true;
	    	File sensorDataFile = new File(dir.getAbsolutePath() + File.separator + Filename);
	    	FileOutputStream fout = new FileOutputStream(sensorDataFile,append);
	        byte [] bytes = message.getBytes();
	        	
	        fout.write(bytes);
	        fout.close();
	        } catch(Exception e) {
	        	   e.printStackTrace();
	          }
    }
	
	public void attitude_data_format(double[] data) {
		
		DecimalFormat df = new DecimalFormat("####.######");
		Attitude_message += " " + df.format(data[0]) + " ";
		Attitude_message += df.format(data[1]) + " ";
		Attitude_message += df.format(data[2]);
		
	}
	
	

}
