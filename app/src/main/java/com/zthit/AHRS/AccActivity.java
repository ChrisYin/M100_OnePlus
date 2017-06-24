package com.zthit.AHRS;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

import android.app.Activity;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.SharedPreferences.Editor;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.TextView;

/*********************************************************************************************
 * 本程序用于手机端采集加速度计的值，首先点击start按钮启动SensorEventListener，然后点击record按钮
 * 向手机sd卡中写入采集到的aver_num个数据，再次点击record按钮则停止写入，然后将手机换一个方位再次点击不同
 * 又写入另外50个数据，一次操作，得到要获取的数据。注意在AndroidManifest.xml文件中加入sd卡的权限。
 * 1.Z轴朝上（屏幕朝上），2.Z轴朝下（屏幕朝下），3.X轴朝上，4.X轴朝下，5.Y轴朝上（手机竖起），
 * 6.Y轴朝下（手机倒立）。
 ********************************************************************************************/
public class AccActivity extends Activity implements OnClickListener, SensorEventListener {
	
	public Button mStartButton, mStopButton,mCaliration,mBack,mRecord,mSave;
	private boolean doWrite = false;
	//Create a new SensorManager to implement SensorEvent
	SensorManager mSensorManager;
	TextView Accview;
	double[] cur_Acc = new double[3];
	int dataLength = 0;                //用于判断是否每个方位的数据是否到达aver_num个
	int aver_num = 50;
	double[][] AccData;
	ArrayList<Double> AccX = new ArrayList<Double>();
	ArrayList<Double> AccY = new ArrayList<Double>();
	ArrayList<Double> AccZ = new ArrayList<Double>();
	
	double[] B = new double[3];
	double[][] M = new double[3][3];

	String Sensor_message = new String();
	File dir = null;

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_acc);		
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);    //设置屏幕常亮
		Accview = (TextView)findViewById(R.id.AccView);
		mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);       //获取传感器service
		//关联三个Button，并依次使用OnClickListener在这个activity当中监听
		mStartButton = (Button) findViewById(R.id.Start);                         
        mStartButton.setOnClickListener(this);
        mStopButton = (Button) findViewById(R.id.Stop);
        mStopButton.setOnClickListener(this);
        mRecord = (Button) findViewById(R.id.Record);
        mRecord.setOnClickListener(this);
        mCaliration = (Button) findViewById(R.id.Calibration);
        mCaliration.setOnClickListener(this);
        mBack = (Button) findViewById(R.id.Back);
        mBack.setOnClickListener(this);
        mSave = (Button) findViewById(R.id.Save);
        mSave.setOnClickListener(this);
        
        try {
			boolean sdCardExist = Environment.getExternalStorageState().equals(  
                    android.os.Environment.MEDIA_MOUNTED);                   //检测是否存在外部sd卡
			if(sdCardExist)
			{
				//如果存在sd卡，则获取sd的路径
				dir = Environment.getExternalStorageDirectory();             //获取sd卡路径目录
				File sensorDataFile = new File(dir.getAbsolutePath()+File.separator+"Acc.txt");   //定义一个File用来关联Acc.txt
				if(!sensorDataFile.exists()){
					//若果不存在该文件则新建一个
					sensorDataFile.createNewFile();
				}
			}			
			
        } catch (IOException e) {
			e.printStackTrace();
			System.out.println("No SDcard!");
        }
        
	}


	@Override
	public void onClick(View v) {
		// TODO Auto-generated method stub
		//Button的click动作监听，click 不通话Button时应该产生哪种效果
		if (v.getId() == R.id.Start) {			
//			doWrite = true;
//			dataLength = 0;
			StartSensor();			
		}
		if (v.getId() == R.id.Stop) {			
			doWrite = false;
			mSensorManager.unregisterListener(this);
			AccData = new double[3][AccX.size()];
			for(int i = 0; i < AccX.size();i++){
				AccData[0][i] = AccX.get(i);
				AccData[1][i] = AccY.get(i);
				AccData[2][i] = AccZ.get(i);
//				Log.d("AccData: " , AccData[0][i]+ " " + AccData[1][i] + " " + AccData[2][i]);
			}
		}		
		if (v.getId() == R.id.Record) {
			if(!doWrite){
				doWrite = true;
				dataLength = 0;
			}
			else{
				doWrite = false;
				dataLength = 0;
			}
		}
		if(v.getId() == R.id.Back){
			mSensorManager.unregisterListener(this);
			Intent intent = new Intent();
			intent.setClass(AccActivity.this, SensorActivity.class);
			AccActivity.this.startActivity(intent);
			AccActivity.this.finish();
		}
		if(v.getId() == R.id.Calibration){
			AccCalibration accCal = new AccCalibration(AccData);
			accCal.Calibration();
			B =  accCal.B;
			M = accCal.M;
			Accview.setText("ScaleMatrix is:\t" + M[0][0] + "\t" + M[0][1] + "\t" + M[0][2] + "\n" +
					        "				\t" + M[1][0] + "\t" + M[1][1] + "\t" + M[1][2] + "\n" +
					        "				\t" + M[2][0] + "\t" + M[2][1] + "\t" + M[2][2] + "\n" +
					        "Bias is       :\t" + B[0] + "\t" + B[1] + "\t" + B[2] + "\n" );
		}
		
		if(v.getId() == R.id.Save){
			save_config(M,B);			
		}
		
	}
	
	public void save_config(double[][] W, double[] V){
		SharedPreferences sharedPreferences = getSharedPreferences("config", MODE_PRIVATE);
		Editor editor = sharedPreferences.edit();
		editor.putFloat("AccM11", (float) W[0][0]);
		editor.putFloat("AccM12", (float) W[0][1]);
		editor.putFloat("AccM13", (float) W[0][2]);
		editor.putFloat("AccM21", (float) W[1][0]);
		editor.putFloat("AccM22", (float) W[1][1]);
		editor.putFloat("AccM23", (float) W[1][2]);
		editor.putFloat("AccM31", (float) W[2][0]);
		editor.putFloat("AccM32", (float) W[2][1]);
		editor.putFloat("AccM33", (float) W[2][2]);
		editor.putFloat("AccB1",  (float) V[0]);
		editor.putFloat("AccB2",  (float) V[1]);
		editor.putFloat("AccB3",  (float) V[2]);
		editor.commit();	
	}	
	
	public void sensor_data_format(double[] data,String name) {
		
		DecimalFormat df = new DecimalFormat("####.######");
		Sensor_message = name +" "+ df.format(data[0])+" ";
		Sensor_message += df.format(data[1])+" ";
		Sensor_message += df.format(data[2])+"\n";
		
	}
	
	
	public void writeFileSdcard(String message) {
				
		try {
	        //FileOutputStream fout = openFileOutput(fileName, Context.MODE_APPEND);
	    	boolean append = true;
	    	File sensorDataFile = new File(dir.getAbsolutePath() + File.separator + "Acc.txt");
	    	FileOutputStream fout = new FileOutputStream(sensorDataFile,append);      //在文件已有的数据后面继续添加
	        byte [] bytes = message.getBytes();
	        	
	        fout.write(bytes);
	        fout.close();
	        } catch(Exception e) {
	        	   e.printStackTrace();
	          }
    }
	
public void StartSensor() {
		try {
            //clean the old data in the sensor_data.txt
	    	File sensorDataFile = new File(dir.getAbsolutePath() + File.separator + "Acc.txt");
	    	FileOutputStream fout = new FileOutputStream(sensorDataFile);	     //覆盖掉之前的数据，写入新的数据，在这里其实就是       	
	        fout.close(); 														 //起到情况之前数据的作用
	        } catch(Exception e) { 
	        	   e.printStackTrace();
	          }
		sensorRegister(Sensor.TYPE_ACCELEROMETER);		
//		sensorRegister(Sensor.TYPE_MAGNETIC_FIELD);
	}
	
	public double[] getSensorData(double[] sensor_data, SensorEvent event) {
		
		sensor_data[0] = event.values[0];
		sensor_data[1] = -event.values[1];
		sensor_data[2] = -event.values[2];
		
		return sensor_data;	
	}
	
	public void sensorRegister(int type){
		Sensor sensor;
		List<Sensor> list = mSensorManager.getSensorList(type);  //获取一个Type类型的传感器并
		sensor = list.get(0);                                    //将这个类型的传感器赋值到sensor中
//		mSensorManager.registerListener(this, sensor,SensorManager.SENSOR_DELAY_NORMAL);
		mSensorManager.registerListener(this, sensor,SensorManager.SENSOR_DELAY_GAME);         //在该activity中注册该Type类型的传感器，并设置数据采集频率
	}

	@Override
	public void onSensorChanged(SensorEvent event) {
		// TODO Auto-generated method stub
		String s= new String();
		for(double v : event.values){					
	        s += v+ "\n";
	    }
		getSensorData(cur_Acc, event);
		Accview.setText("Accelerometer information is:\n"+s);
		if(doWrite){
			if(dataLength < aver_num){
				//每个方位记录50个数据
				sensor_data_format(cur_Acc, "Accelerometer");    //将cur_Acc按一定的格式写入到全局变量Sensor_message中
	            writeFileSdcard(Sensor_message);                 //将数据写入到sd卡中
	            dataLength++;                                    //数据长度加1
	            AccX.add(cur_Acc[0]);
	            AccY.add(cur_Acc[1]);
	            AccZ.add(cur_Acc[2]);
			}
			
		}
	}

	@Override
	public void onAccuracyChanged(Sensor sensor, int accuracy) {
		// TODO Auto-generated method stub
		Log.d("Sensor","onAccuracyChanged: " + sensor + ", accuracy: " + accuracy);
	}
	
	public void onPause(){
		super.onPause();
		mSensorManager.unregisterListener(this);
	}

}

