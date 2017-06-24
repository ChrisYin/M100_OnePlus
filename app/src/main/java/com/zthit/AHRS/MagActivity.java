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

public class MagActivity extends Activity implements OnClickListener, SensorEventListener {
	
	public Button mStartButton, mStopButton,mCalibration,mBack,mSave;
	private boolean doWrite = false;
	int dataCount = 0;
	//Create a new SensorManager to implement SensorEvent
	SensorManager mSensorManager;
	TextView Magview; 
	double[] cur_Mag = new double[3];
	double[][] MagData;
	ArrayList<Double> MagX = new ArrayList<Double>();
	ArrayList<Double> MagY = new ArrayList<Double>();
	ArrayList<Double> MagZ = new ArrayList<Double>();
//	int num = 0;
	double[][] V = new double[3][3];
	double[][] W = new double[3][3];
	
	String Sensor_message = new String();
	File dir = null;

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_mag);
		
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);   //设置屏幕常亮
		Magview = (TextView)findViewById(R.id.MagView);
		mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);       //获取传感器service
		//关联三个Button，并依次使用OnClickListener在这个activity当中监听
		mStartButton = (Button) findViewById(R.id.Start);
        mStartButton.setOnClickListener(this);
        mStopButton = (Button) findViewById(R.id.Stop);
        mStopButton.setOnClickListener(this);
        mBack = (Button) findViewById(R.id.Back);
        mBack.setOnClickListener(this);
        mCalibration = (Button) findViewById(R.id.Calibration);
        mCalibration.setOnClickListener(this);
        mSave = (Button) findViewById(R.id.Save);
        mSave.setOnClickListener(this);
        
        try {
			boolean sdCardExist = Environment.getExternalStorageState().equals(  
                    android.os.Environment.MEDIA_MOUNTED);                    //检测是否存在外部sd卡
			if(sdCardExist)
			{
				//如果存在sd卡，则获取sd的路径
				dir = Environment.getExternalStorageDirectory();             //获取sd卡路径目录
				File sensorDataFile = new File(dir.getAbsolutePath()+File.separator+"Mag.txt");       //定义一个File用来关联Mag.txt
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
			doWrite = true;
			StartSensor();			
		}
		
		if (v.getId() == R.id.Stop) {			
			doWrite = false;
			mSensorManager.unregisterListener(this);
			MagData = new double[3][MagX.size()];
			for(int i = 0; i < MagX.size();i++){
				MagData[0][i] = MagX.get(i);
				MagData[1][i] = MagY.get(i);
				MagData[2][i] = MagZ.get(i);
//				Log.d("MagData: " , MagData[0][i]+ " " + MagData[0][i] + " " + MagData[2][i]);
			}
			
		}
		
		if(v.getId() == R.id.Back){
			mSensorManager.unregisterListener(this);
			Intent intent = new Intent();
			intent.setClass(MagActivity.this, SensorActivity.class);
			MagActivity.this.startActivity(intent);
			MagActivity.this.finish();
		}
		
		if(v.getId() == R.id.Calibration){
			MagCalibration magCal = new MagCalibration(MagData);
			magCal.Calibration();
			V = (magCal.Bias).getArrayCopy();
			W = (magCal.Scale).getArrayCopy();

			Magview.setText("HardIron is:\t" + V[0][0] + "\t" + V[1][0] + "\t" + V[2][0] + "\n" +
					        "SoftIron is:\t" + W[0][0] + "\t" + W[0][1] + "\t" + W[0][2] + "\n" +
					        "            \t" + W[1][0] + "\t" + W[1][1] + "\t" + W[1][2] + "\n" +
					        "            \t" + W[2][0] + "\t" + W[2][1] + "\t" + W[2][2] + "\n" );
		}
		
		if(v.getId() == R.id.Save){
			save_config(W,V);			
		}
		
	}

	public void save_config(double[][] W, double[][] V){
		SharedPreferences sharedPreferences = getSharedPreferences("config", MODE_PRIVATE);
		Editor editor = sharedPreferences.edit();
		editor.putFloat("MagW11", (float) W[0][0]);
		editor.putFloat("MagW12", (float) W[0][1]);
		editor.putFloat("MagW13", (float) W[0][2]);
		editor.putFloat("MagW21", (float) W[1][0]);
		editor.putFloat("MagW22", (float) W[1][1]);
		editor.putFloat("MagW23", (float) W[1][2]);
		editor.putFloat("MagW31", (float) W[2][0]);
		editor.putFloat("MagW32", (float) W[2][1]);
		editor.putFloat("MagW33", (float) W[2][2]);
		editor.putFloat("MagV1",  (float) V[0][0]);
		editor.putFloat("MagV2",  (float) V[1][0]);
		editor.putFloat("MagV3",  (float) V[2][0]); 
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
				File sensorDataFile = new File(dir.getAbsolutePath() + File.separator + "Mag.txt");
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
	    	File sensorDataFile = new File(dir.getAbsolutePath() + File.separator + "Mag.txt");
	    	FileOutputStream fout = new FileOutputStream(sensorDataFile);	    //覆盖掉之前的数据，写入新的数据，在这里其实就是           	 
	        fout.close();                   									//起到情况之前数据的作用
	        } catch(Exception e) {
	        	   e.printStackTrace();
	          }
		sensorRegister(Sensor.TYPE_MAGNETIC_FIELD);				
	}
	
	public double[] getSensorData(double[] sensor_data, SensorEvent event) {
		
		sensor_data[0] = event.values[0];
		sensor_data[1] = -event.values[1];
		sensor_data[2] = -event.values[2];
		
		return sensor_data;	
	}
	
	public void sensorRegister(int type){
		Sensor sensor;
		List<Sensor> list = mSensorManager.getSensorList(type);      //获取一个Type类型的传感器并
		sensor = list.get(0);										 //将这个类型的传感器赋值到sensor中
//		mSensorManager.registerListener(this, sensor,SensorManager.SENSOR_DELAY_NORMAL);
		mSensorManager.registerListener(this, sensor,SensorManager.SENSOR_DELAY_GAME);      //在该activity中注册该Type类型的传感器，并设置数据采集频率
	}

	@Override
	public void onSensorChanged(SensorEvent event) {
		// TODO Auto-generated method stub
		String s= new String();
		for(double v : event.values){					
	        s += v+ "\n";
	    }
		getSensorData(cur_Mag, event);               
		Magview.setText("Magnetic information is:\n"+s);
		if(doWrite){
			if(dataCount <= 100){
				dataCount++;
			}
			else{
				sensor_data_format(cur_Mag, "Magnetometer");   //将cur_Mag按一定的格式写入到全局变量Sensor_message中
				writeFileSdcard(Sensor_message);
				MagX.add(cur_Mag[0]);
				MagY.add(cur_Mag[1]);
				MagZ.add(cur_Mag[2]);
//				Log.d("MagData: " , Mag11[0]+ " " + Mag11[1] + " " + Mag11[2]);
//				num ++;
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