package com.zthit.AHRS;

/*这个程序修改是用于SLAM当时需呀gps用的*/

import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.text.DecimalFormat;
import java.util.Iterator;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;
import java.util.UUID;

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.SharedPreferences.Editor;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Criteria;
import android.location.GpsSatellite;
import android.location.GpsStatus;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.location.LocationProvider;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.Message;
import android.provider.Settings;
import android.text.format.Time;
import android.util.Log;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;

import com.jilk.ros.ROSClient;
import com.jilk.ros.rosbridge.ROSBridgeClient;

public class SensorActivity extends Activity implements SensorEventListener, OnClickListener {
    private Button mRecordButton, mStopButton, mConnectButton, mDisconnectButton, mResetButton;
    private Button mMagActivity, mAccActivity;
    private Button Z_UP,Z_DOWN,Z_CMD;
    private Switch TakeOff,Landing, freez, freerp,freeyaw;
    private TextView height;
    private boolean doWrite = false;
    private boolean doConnect = false;
    private boolean tk=false;
    private boolean ld=false;
    private boolean fz =false;
    private boolean frp =false;
    private boolean fyaw=false;
    private int zdata=0;
    //Create a new SensorManager to implement SensorEvent
    private SensorManager mSensorManager;
    private LocationManager mLocation;
    private Location location_save;
    private TextView Gpsview, Countview;
    private TextView mDisplay;
//	public EditText mIpText;

    private File dir = null;                                           //定义SD卡文件目录
    private String FileName_flight = "flight_data.txt";
    private String FileName_attitude = "attitude_data.txt";
    private boolean acc_status, gyr_status, mag_status;       //用于判断个传感器获取数据的状态
    public double[] cur_Acc = new double[3];                         //用于保持个传感器的数据
    public double[] cur_Gyr = new double[3];
    public double[] cur_Mag = new double[3];
//	private String Sensor_message = new String();

    //用于ARHS计算时用的变量及初始值
//	private boolean sensor_status;                               //用于判断系统的状态是初始还是运行,false表示初始化阶段
//	final  private double NS2S = 1.0/1000000000.0;         //纳秒转毫秒
    public double[][] AccM = new double[3][3];
    public double[] AccB = new double[3];
    public double[][] MagW = new double[3][3];
    public double[] MagV = new double[3];
    //	double cur_time;
    private double[] Quat = new double[4];
    private double[] Euler_angle = new double[3];

    //用于Bluetooth连接的变量
    private final UUID MY_UUID = UUID
            .fromString("00001101-0000-1000-8000-00805F9B34FB"); // 两台设备要一致才行
    private BluetoothAdapter bluetoothAdapter;
    //private BluetoothDevice btdevice;
    private BluetoothSocket clientSocket;
    private boolean btc_state = false;
    public BufferedReader br;
    private Bluetooth_read bluetooth_readThread;
    private int record_cn;

    // Intent request codes
    private static final int REQUEST_CONNECT_DEVICE = 1;
    private static final int REQUEST_ENABLE_BT = 2;

    //一些状态标志位
    private boolean record_state = false;
    //	private boolean gpsconnect_state = false;
    private boolean isTimerStart;
    private Timer timer = new Timer();                          //定义一个定时器和定时器任务

    private TimerTask task = new TimerTask() {
        public void run() {
//			if(record_state && gpsconnect_state){
            if (record_state) {
                //sendMsg();
//				System.out.println(socket.isClosed() + " ; " +socket.isConnected() + "; " +socket.isOutputShutdown());
                String flight_msg = new String();
                DecimalFormat df1 = new DecimalFormat("####.########");
                DecimalFormat df2 = new DecimalFormat("####.####");
                Log.i("DEBUG", "aaaaaaaaaaaaaa");
                flight_msg = "flm" + " " + df1.format(location_save.getLongitude());
                flight_msg += " " + df1.format(location_save.getLatitude());
                flight_msg += " " + df2.format(location_save.getAltitude());
                flight_msg += " " + df2.format(location_save.getSpeed());
                flight_msg += " " + df2.format(location_save.getBearing());
                flight_msg += " " + df2.format(cur_Acc[0]) + " " + df2.format(cur_Acc[1]) + " " + df2.format(cur_Acc[2]);
                flight_msg += " " + df2.format(cur_Gyr[0]) + " " + df2.format(cur_Gyr[1]) + " " + df2.format(cur_Gyr[2]);
                flight_msg += " " + df2.format(cur_Mag[0]) + " " + df2.format(cur_Mag[1]) + " " + df2.format(cur_Mag[2]) + "\n";
                writeFileSdcard(flight_msg, FileName_flight);
                Message msg = new Message();
                msg.obj = "Record";
                TimerHandler.sendMessage(msg);
                msg = Message.obtain();
            } else {

            }
        }
    };

    // 姿态算法线程
    private attitude_thread attiudeThread = new attitude_thread(cur_Acc, cur_Gyr, cur_Mag, AccM, AccB, MagW, MagV);
    private boolean thread_status;                 //判断线程是否已经启动


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_sensor);

//		timer.scheduleAtFixedRate(task, 0, 500);		   //设置定时任务
        get_config();
//		Log.v("MagM: ",  MagW[0][0] + " " + MagW[1][1] + " " + MagW[2][2]);
        init();
        openBluetooth();
        openGPSSetting();
        thread_status = false;   //false表示线程没有start；
        isTimerStart = false;
        record_cn = 0;
//		btr_status = false;

        //判断是否存在sd卡
        try {
            boolean sdCardExist = Environment.getExternalStorageState().equals(
                    android.os.Environment.MEDIA_MOUNTED);
            if (sdCardExist) {
                dir = Environment.getExternalStorageDirectory();             //获取sd卡路径目录
                File flightDataFile = new File(dir.getAbsolutePath() + File.separator + FileName_flight);
                if (!flightDataFile.exists()) {
                    //若果不存在该文件则新建一个
                    flightDataFile.createNewFile();
                }

                File attitudeDataFile = new File(dir.getAbsolutePath() + File.separator + FileName_attitude);
                if (!attitudeDataFile.exists()) {
                    //若果不存在该文件则新建一个
                    attitudeDataFile.createNewFile();
                }
            }

        } catch (IOException e) {
            e.printStackTrace();
            System.out.println("No SDcard!");
        }

        mDisplay.setText("Stop!" + getTime());
    }

    //初始化各个控件对应
    private void init() {

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);   //使屏幕强制横屏
        mDisplay = (TextView) findViewById(R.id.StateDisplay);
        Gpsview = (TextView) findViewById(R.id.GpsView);
        Countview = (TextView) findViewById(R.id.CountView);
        mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
//		mIpText = (EditText)findViewById(R.id.EnterIp);
//		mIpText.setText("192.168.1.10X");
        mRecordButton = (Button) findViewById(R.id.Record);
        mRecordButton.setOnClickListener(this);
        mStopButton = (Button) findViewById(R.id.Stop);
        mStopButton.setOnClickListener(this);
        mDisconnectButton = (Button) findViewById(R.id.Disconnect);
        mDisconnectButton.setOnClickListener(this);
        mConnectButton = (Button) findViewById(R.id.Connect);
        mConnectButton.setOnClickListener(this);
//        mIPButton = (Button) findViewById(R.id.Ip_Save);
//        mIPButton.setOnClickListener(this);
        mMagActivity = (Button) findViewById(R.id.MagCalibration);
        mMagActivity.setOnClickListener(this);
        mAccActivity = (Button) findViewById(R.id.AccCalibration);
        mAccActivity.setOnClickListener(this);
        mAccActivity = (Button) findViewById(R.id.GyroCalibration);
        mAccActivity.setOnClickListener(this);
        mResetButton = (Button) findViewById(R.id.Reset);
        mResetButton.setOnClickListener(this);

        Z_UP =(Button)findViewById(R.id.Z_up);
        Z_UP.setOnClickListener(this);
        Z_DOWN=(Button)findViewById(R.id.Z_down);
        Z_DOWN.setOnClickListener(this);
        Z_CMD=(Button)findViewById(R.id.cm_z);
        Z_CMD.setOnClickListener(this);
        TakeOff=(Switch)findViewById(R.id.TO_Switch);
        TakeOff.setOnCheckedChangeListener(new SwitchCheckedListener());
        Landing=(Switch)findViewById(R.id.LD_Switch);
        Landing.setOnCheckedChangeListener(new SwitchCheckedListener());
        freerp =(Switch)findViewById(R.id.freerp_Switch);
        freerp.setOnCheckedChangeListener(new SwitchCheckedListener());
        freeyaw =(Switch)findViewById(R.id.freeyaw_Switch);
        freeyaw.setOnCheckedChangeListener(new SwitchCheckedListener());
        freez =(Switch)findViewById(R.id.freez_Switch);
        freez.setOnCheckedChangeListener(new SwitchCheckedListener());


    }

    private void openBluetooth() {
        bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        if (!bluetoothAdapter.isEnabled()) {
            Toast.makeText(this, "请开启蓝牙...", Toast.LENGTH_SHORT).show();
            Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(enableBtIntent, 2);
        }
//		String address = "20:13:08:14:28:66"; 
//		btdevice = bluetoothAdapter.getRemoteDevice(address);
    }

    private void openGPSSetting() {
        mLocation = (LocationManager) getSystemService(Context.LOCATION_SERVICE);
        //判断GPS是否正常启动
        if (!mLocation.isProviderEnabled(LocationManager.GPS_PROVIDER)) {
            Toast.makeText(this, "请开启GPS导航...", Toast.LENGTH_SHORT).show();
            //返回开启GPS导航设置界面
            Intent intent = new Intent(Settings.ACTION_LOCATION_SOURCE_SETTINGS);
            startActivityForResult(intent, 0);
            return;
        }

        //为获取地理位置信息时设置查询条件
        String bestProvider = mLocation.getBestProvider(getCriteria(), true);
        //获取位置信息
        //如果不设置查询要求，getLastKnownLocation方法传人的参数为LocationManager.GPS_PROVIDER
        Location location = mLocation.getLastKnownLocation(bestProvider);
        location_save = location;  // 防止gps没有定位成功location_save 为空，导致程序跑死
        updateView(location);
        //监听状态
        mLocation.addGpsStatusListener(listener);
        //绑定监听，有4个参数    
        //参数1，设备：有GPS_PROVIDER和NETWORK_PROVIDER两种
        //参数2，位置信息更新周期，单位毫秒    
        //参数3，位置变化最小距离：当位置距离变化超过此值时，将更新位置信息    
        //参数4，监听    
        //备注：参数2和3，如果参数3不为0，则以参数3为准；参数3为0，则通过时间来定时更新；两者为0，则随时刷新   

        // 1秒更新一次，或最小位移变化超过1米更新一次；
        //注意：此处更新准确度非常低，推荐在service里面启动一个Thread，在run中sleep(10000);然后执行handler.sendMessage(),更新位置

    }

    private Criteria getCriteria() {
        Criteria criteria = new Criteria();
        //设置定位精确度 Criteria.ACCURACY_COARSE比较粗略，Criteria.ACCURACY_FINE则比较精细 
        criteria.setAccuracy(Criteria.ACCURACY_FINE);
        //设置是否要求速度
        criteria.setSpeedRequired(true);
        // 设置是否允许运营商收费  
        criteria.setCostAllowed(false);
        //设置是否需要方位信息
        criteria.setBearingRequired(false);
        //设置是否需要海拔信息
        criteria.setAltitudeRequired(true);
        // 设置对电源的需求  
        criteria.setPowerRequirement(Criteria.POWER_LOW);
        return criteria;
    }

    //状态监听
    GpsStatus.Listener listener = new GpsStatus.Listener() {
        public void onGpsStatusChanged(int event) {
            switch (event) {
                //第一次定位
                case GpsStatus.GPS_EVENT_FIRST_FIX:
//                Log.i("GPS Information", "第一次定位");
                    break;
                //卫星状态改变
                case GpsStatus.GPS_EVENT_SATELLITE_STATUS:
//                Log.i("GPS Information", "卫星状态改变");
                    //获取当前状态
                    GpsStatus gpsStatus = mLocation.getGpsStatus(null);
                    //获取卫星颗数的默认最大值
                    int maxSatellites = gpsStatus.getMaxSatellites();
                    //创建一个迭代器保存所有卫星
                    Iterator<GpsSatellite> iters = gpsStatus.getSatellites().iterator();
                    int count = 0;
                    while (iters.hasNext() && count <= maxSatellites) {
                        GpsSatellite s = iters.next();
                        count++;
                    }
                    System.out.println("搜索到：" + count + "颗卫星");
                    break;
                //定位启动
                case GpsStatus.GPS_EVENT_STARTED:
//                Log.i("GPS Information", "定位启动");
                    break;
                //定位结束
                case GpsStatus.GPS_EVENT_STOPPED:
//                Log.i("GPS Information", "定位结束");
                    break;
            }
        }

        ;
    };

    private void updateView(Location location) {
        if (location != null) {
            Gpsview.setText("经度：" + String.valueOf(location.getLongitude()));
            Gpsview.append("\n纬度：" + String.valueOf(location.getLatitude()));
            Gpsview.append("\n海拔：" + String.valueOf(location.getAltitude()));
            Gpsview.append("\n速度：" + String.valueOf(location.getSpeed()));
            Gpsview.append("\n航向：" + String.valueOf(location.getBearing()));
        }

    }

    private LocationListener locationListener = new LocationListener() {

        /**
         * 位置信息变化时触发
         */
        public void onLocationChanged(Location location) {
            updateView(location);
            location_save = location;
//            gpsconnect_state = true;

        }

        /**
         * GPS状态变化时触发，该方法在有的手机上面可以触发，有的手机上面无法触发
         */
        public void onStatusChanged(String provider, int status, Bundle extras) {
            switch (status) {
                //GPS状态为可见时
                case LocationProvider.AVAILABLE:
//            	Toast.makeText(SensorActivity.this, "GPS 已定位", Toast.LENGTH_SHORT).show();
//            	 gpsconnect_state = true;
//                Log.i("GPS Information", "当前GPS状态为可见状态");
                    break;
                //GPS状态为服务区外时
                case LocationProvider.OUT_OF_SERVICE:
//            	Toast.makeText(SensorActivity.this, "GPS 定位丢失", Toast.LENGTH_SHORT).show();
//            	gpsconnect_state = false;
//                Log.i("GPS Information", "当前GPS状态为服务区外状态");
                    break;
                //GPS状态为暂停服务时
                case LocationProvider.TEMPORARILY_UNAVAILABLE:
//            	Toast.makeText(SensorActivity.this, "GPS 暂时丢失", Toast.LENGTH_SHORT).show();
//                Log.i("GPS Information", "当前GPS状态为暂停服务状态");
                    break;
            }
        }

        /**
         * GPS开启时触发，设置中开启GPS
         */
        public void onProviderEnabled(String provider) {
            Location location = mLocation.getLastKnownLocation(provider);
//            gpsconnect_state = true;
            updateView(location);
//            Toast.makeText(getApplicationContext(), "GPS 已定位", Toast.LENGTH_SHORT).show();
//            Log.i("Flight Information", "GPS START LOCATE!");
        }

        /**
         * GPS禁用时触发，设置中关闭GPS
         */
        public void onProviderDisabled(String provider) {
            updateView(null);
//            gpsconnect_state = false;
//            Toast.makeText(getApplicationContext(), "GPS 定位丢失", Toast.LENGTH_SHORT).show();
//            Log.i("Flight Information", "GPS STOP LOCATE!");
        }

    };

    //从配置文件中调用传感器校正的参数
    private void get_config() {
        SharedPreferences sharedPreferences = getSharedPreferences("config", MODE_PRIVATE);

        AccM[0][0] = sharedPreferences.getFloat("AccM11", 1.0f);
        AccM[0][1] = sharedPreferences.getFloat("AccM12", 0.0f);
        AccM[0][2] = sharedPreferences.getFloat("AccM13", 0.0f);
        AccM[1][0] = sharedPreferences.getFloat("AccM21", 0.0f);
        AccM[1][1] = sharedPreferences.getFloat("AccM22", 1.0f);
        AccM[1][2] = sharedPreferences.getFloat("AccM23", 0.0f);
        AccM[2][0] = sharedPreferences.getFloat("AccM31", 0.0f);
        AccM[2][1] = sharedPreferences.getFloat("AccM32", 0.0f);
        AccM[2][2] = sharedPreferences.getFloat("AccM33", 1.0f);
        AccB[0] = sharedPreferences.getFloat("AccB1", 0.0f);
        AccB[1] = sharedPreferences.getFloat("AccB2", 0.0f);
        AccB[2] = sharedPreferences.getFloat("AccB3", 0.0f);

        MagW[0][0] = sharedPreferences.getFloat("MagW11", 1.0f);
        MagW[0][1] = sharedPreferences.getFloat("MagW12", 0.0f);
        MagW[0][2] = sharedPreferences.getFloat("MagW13", 0.0f);
        MagW[1][0] = sharedPreferences.getFloat("MagW21", 0.0f);
        MagW[1][1] = sharedPreferences.getFloat("MagW22", 1.0f);
        MagW[1][2] = sharedPreferences.getFloat("MagW23", 0.0f);
        MagW[2][0] = sharedPreferences.getFloat("MagW31", 0.0f);
        MagW[2][1] = sharedPreferences.getFloat("MagW32", 0.0f);
        MagW[2][2] = sharedPreferences.getFloat("MagW33", 1.0f);
        MagV[0] = sharedPreferences.getFloat("MagV1", 0.0f);
        MagV[1] = sharedPreferences.getFloat("MagV2", 0.0f);
        MagV[2] = sharedPreferences.getFloat("MagV3", 0.0f);
    }

    //重置配置文件中的所有传感器校准参数为理想值
    private void reset_config() {
        SharedPreferences sharedPreferences = getSharedPreferences("config", MODE_PRIVATE);
        Editor editor = sharedPreferences.edit();
        editor.putFloat("AccM11", 1.0f);
        editor.putFloat("AccM12", 0.0f);
        editor.putFloat("AccM13", 0.0f);
        editor.putFloat("AccM21", 0.0f);
        editor.putFloat("AccM22", 1.0f);
        editor.putFloat("AccM23", 0.0f);
        editor.putFloat("AccM31", 0.0f);
        editor.putFloat("AccM32", 0.0f);
        editor.putFloat("AccM33", 1.0f);
        editor.putFloat("AccB1", 0.0f);
        editor.putFloat("AccB2", 0.0f);
        editor.putFloat("AccB3", 0.0f);

        editor.putFloat("MagW11", 1.0f);
        editor.putFloat("MagW12", 0.0f);
        editor.putFloat("MagW13", 0.0f);
        editor.putFloat("MagW21", 0.0f);
        editor.putFloat("MagW22", 1.0f);
        editor.putFloat("MagW23", 0.0f);
        editor.putFloat("MagW31", 0.0f);
        editor.putFloat("MagW32", 0.0f);
        editor.putFloat("MagW33", 1.0f);
        editor.putFloat("MagV1", 0.0f);
        editor.putFloat("MagV2", 0.0f);
        editor.putFloat("MagV3", 0.0f);
        editor.commit();
    }


    @Override
    public void onSensorChanged(SensorEvent event) {
        //String message = new String();
        String attitudeData = new String();
        synchronized (this) {

//			for(double v : event.values){					
//		        s += v+ "\n";
//		    }
            switch (event.sensor.getType()) {
                case Sensor.TYPE_ACCELEROMETER:
                    getSensorData(cur_Acc, event);
                    acc_status = true;
                    break;
                case Sensor.TYPE_GYROSCOPE:
                    getSensorData(cur_Gyr, event);
                    gyr_status = true;
                    break;
                case Sensor.TYPE_MAGNETIC_FIELD:
                    getSensorData(cur_Mag, event);
                    mag_status = true;
                    break;
            }
            if (doWrite) {
                if (acc_status && gyr_status && mag_status) {
                    Quat = (attiudeThread.q).clone();
                    Euler_angle = (ARHS_Library.quat2eulers(Quat)).clone();

                    attitudeData = "roll:" + Euler_angle[0] + "\n pitch:" + Euler_angle[1] + "\n yaw:" + Euler_angle[2];
                    mDisplay.setText(attitudeData);
                    if(doConnect && tk==true && ld==false && fz ==false && (frp ==true || fyaw==true)) {
                        String msg = "";
                        msg = "{\"op\":\"publish\",\"topic\":\"" + "/rpydata" + "\",\"msg\":{" + "\"roll\":" + Euler_angle[0] + "," + "\"pitch\":" + Euler_angle[1] + "," + "\"yaw\":" + Euler_angle[2] + "" + "}}";

                        //ROSBridgeClient client1;
                        //client1 = ((RCApplication)getApplication()).getRosClient();
                        client.send(msg);
                        acc_status = false;
                        gyr_status = false;
                        mag_status = false;
                    }
                    else {
                        showTip("do not connect");
                    }
                }
            }
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        Log.d("Sensor", "onAccuracyChanged: " + sensor + ", accuracy: " + accuracy);
    }


    @Override
    public void onClick(View v) {
        if (v.getId() == R.id.Record) {
            if(doConnect==true ) {
                doWrite = true;
                attiudeThread.threadResume();         //运行姿态线程中的姿态算法
            }
            else
            {
                showTip("do not connect");
            }

        }
        if (v.getId() == R.id.Stop) {
            if(doConnect==true) {
                doWrite = false;
                attiudeThread.threadPause();          //暂定线程
            }
            else
            {
                showTip("do not connect");
            }
        }
        if (v.getId() == R.id.Connect) {
            StartSensor();
            //mDisplay.setText("Start!");
            //BluetoothConnect();
            connect("192.168.1.101", "9090");
            //connect("10.5.5.1","9090");
            if (!thread_status) {
                //线程如果没有start则启动线程
                attiudeThread.start();          //启动线程但是还没有执行姿态算法
                thread_status = true;
            }
            //mLocation.requestLocationUpdates(LocationManager.GPS_PROVIDER, 0, 0, locationListener);

        }
        if (v.getId() == R.id.Disconnect) {
            stopListening();
            doWrite = false;
            attiudeThread.threadPause();
            //BluetoothDisconnect();
            doConnect=false;
            client.disconnect();
            mLocation.removeUpdates(locationListener);

        }


        if (v.getId() == R.id.Reset) {
            reset_config();
            get_config();
        }

        if (v.getId() == R.id.MagCalibration) {
            Intent intent = new Intent();
            intent.setClass(SensorActivity.this, MagActivity.class);
            SensorActivity.this.startActivity(intent);
            SensorActivity.this.finish();
        }
        if (v.getId() == R.id.AccCalibration) {
            Intent intent = new Intent();
            intent.setClass(SensorActivity.this, AccActivity.class);
            SensorActivity.this.startActivity(intent);
            SensorActivity.this.finish();
        }
        if (v.getId() == R.id.GyroCalibration) {
            Intent intent = new Intent();
            intent.setClass(SensorActivity.this, GyroActivity.class);
            SensorActivity.this.startActivity(intent);
            SensorActivity.this.finish();
        }
        if (v.getId() == R.id.Z_up) {
            showTip("z++");
            zdata=10;
            height=(TextView) findViewById(R.id.height);
            height.setText("Go UP");

        }
        if (v.getId() == R.id.Z_down) {
            showTip("z--");
            zdata=-10;
            height=(TextView) findViewById(R.id.height);
            height.setText("Go Down");

        }
        if (v.getId() == R.id.cm_z) {
            if(doConnect && zdata!=0 && tk==true && ld==false && fz ==true && frp ==false) {
                showTip("height success!");
                String msg = "";
                msg = "{\"op\":\"publish\",\"topic\":\"" + "/zdata" + "\",\"msg\":{" + "\"z_cmd\":" + zdata +"}}";
                //ROSBridgeClient client1;
                //client1 = ((RCApplication)getApplication()).getRosClient();
                client.send(msg);
                zdata=0;
                height=(TextView) findViewById(R.id.height);
                height.setText("None");

            }
            else
            {
                showTip("fail to contorl height");
            }
        }
    }

    private class SwitchCheckedListener implements CompoundButton.OnCheckedChangeListener
    {
        @Override
        public void onCheckedChanged(CompoundButton v, boolean isChecked) {

            if(v.getId()==R.id.TO_Switch){

                if(isChecked)
                {
                    if(ld==false && fz ==false && frp ==false && fyaw==false && doConnect)
                    {
                        tk=isChecked;
                        showTip("Take OFF successfully!");
                        String msg = "";
                        msg = "{\"op\":\"publish\",\"topic\":\"" + "/switchdata" + "\",\"msg\":{"+"\"cmd\":"+0x01+"}}";
                        //ROSBridgeClient client1;
                        //client1 = ((RCApplication)getApplication()).getRosClient();
                        client.send(msg);
                    }
                    else
                    {
                        v.setChecked(false);
                        showTip("Take OFF fail!");
                    }
                }
                else
                {
                    if(fz ==false && frp==false && fyaw==false)
                    {
                        tk=isChecked;
                    }
                    else
                    {
                        v.setChecked(true);
                    }
                }

            }
            if(v.getId()==R.id.LD_Switch){

                if(isChecked)
                {
                    if(tk==false && fz ==false && frp ==false && fyaw==false  && doConnect)
                    {
                        ld=isChecked;
                        showTip("Landing successfully!");
                        String msg = "";
                        msg = "{\"op\":\"publish\",\"topic\":\"" + "/switchdata" + "\",\"msg\":{"+"\"cmd\":"+0x02+"}}";
                        //ROSBridgeClient client1;
                        //client1 = ((RCApplication)getApplication()).getRosClient();
                        client.send(msg);
                    }
                    else
                    {
                        v.setChecked(false);
                        showTip("Landing fail!");
                    }
                }
                else
                {
                    if(fz ==false && frp==false && fyaw==false)
                    {
                        ld=isChecked;
                    }
                    else
                    {
                        v.setChecked(true);
                    }
                }
            }
            if(v.getId()==R.id.freez_Switch){

                if(isChecked)
                {
                    if(tk==true && ld==false && frp ==false  && fyaw==false  && doConnect)
                    {
                        fz =isChecked;
                        showTip("free z successfully!");
                        String msg = "";
                        msg = "{\"op\":\"publish\",\"topic\":\"" + "/switchdata" + "\",\"msg\":{"+"\"cmd\":"+0x03+"}}";
                        //ROSBridgeClient client1;
                        //client1 = ((RCApplication)getApplication()).getRosClient();
                        client.send(msg);
                    }
                    else
                    {
                        v.setChecked(false);
                        showTip("Free z fail!");
                    }
                }
                else
                {

                    if(tk==true && ld==false && frp ==false  && fyaw==false  && doConnect)
                    {
                        fz =isChecked;
                        showTip("hold z successfully!");
                        String msg = "";
                        msg = "{\"op\":\"publish\",\"topic\":\"" + "/switchdata" + "\",\"msg\":{"+"\"cmd\":"+0x06+"}}";
                        //ROSBridgeClient client1;
                        //client1 = ((RCApplication)getApplication()).getRosClient();
                        client.send(msg);
                    }
                    else
                    {
                        v.setChecked(true);
                        showTip("Hold z fail!");
                    }

                }
            }
            if(v.getId()==R.id.freerp_Switch){

                if(isChecked)
                {
                    if(tk==true && ld==false && fz ==false && fyaw==false && doConnect)
                    {
                        frp =isChecked;
                        showTip("free rp successfully!");
                        String msg = "";
                        msg = "{\"op\":\"publish\",\"topic\":\"" + "/switchdata" + "\",\"msg\":{"+"\"cmd\":"+0x04+"}}";
                        //ROSBridgeClient client1;
                        //client1 = ((RCApplication)getApplication()).getRosClient();
                        client.send(msg);
                    }
                    else
                    {
                        v.setChecked(false);
                        showTip("Free rp fail!");
                    }
                }
                else
                {
                    if(tk==true && ld==false && fz ==false && fyaw==false && doConnect)
                    {
                        frp =isChecked;
                        showTip("Hold rp successfully!");
                        String msg = "";
                        msg = "{\"op\":\"publish\",\"topic\":\"" + "/switchdata" + "\",\"msg\":{"+"\"cmd\":"+0x06+"}}";
                        //ROSBridgeClient client1;
                        //client1 = ((RCApplication)getApplication()).getRosClient();
                        client.send(msg);
                    }
                    else
                    {
                        v.setChecked(true);
                        showTip("Hold rp fail!");
                    }
                }
            }
            if(v.getId()==R.id.freeyaw_Switch){

                if(isChecked)
                {
                    if(tk==true && ld==false && fz ==false && frp==false && doConnect)
                    {
                        fyaw =isChecked;
                        showTip("free yaw successfully!");
                        String msg = "";
                        msg = "{\"op\":\"publish\",\"topic\":\"" + "/switchdata" + "\",\"msg\":{"+"\"cmd\":"+0x05+"}}";
                        //ROSBridgeClient client1;
                        //client1 = ((RCApplication)getApplication()).getRosClient();
                        client.send(msg);
                    }
                    else
                    {
                        v.setChecked(false);
                        showTip("Free yaw fail!");
                    }
                }
                else
                {
                    if(tk==true && ld==false && fz ==false && frp==false && doConnect)
                    {
                        fyaw =isChecked;
                        showTip("Hold yaw successfully!");
                        String msg = "";
                        msg = "{\"op\":\"publish\",\"topic\":\"" + "/switchdata" + "\",\"msg\":{"+"\"cmd\":"+0x06+"}}";
                        //ROSBridgeClient client1;
                        //client1 = ((RCApplication)getApplication()).getRosClient();
                        client.send(msg);
                    }
                    else
                    {
                        v.setChecked(true);
                        showTip("Hold yaw fail!");
                    }
                }
            }

//            if(doConnect)
//            {
//                showTip(String.format("TakeOFF:%b,Landing:%b,HoldZ:%b,HoldXY:%b",tk,ld,fz,frp));
//                String msg = "";
//                msg = "{\"op\":\"publish\",\"topic\":\"" + "/switchdata" + "\",\"msg\":{"+"\"takeoff\":"+tk+","+"\"landing\":"+ld+"," + "\"z_hold\":"+fz+","+"\"xy_hold\":"+frp+"}}";
//                //ROSBridgeClient client1;
//                //client1 = ((RCApplication)getApplication()).getRosClient();
//                client.send(msg);
//            }
//            else
//            {
//                showTip("do not connect");
//            }

        }
    }


    private void sensorRegister(int type) {
        Sensor sensor;
        List<Sensor> list = mSensorManager.getSensorList(type);
        sensor = list.get(0);
//		mSensorManager.registerListener(this, sensor,SensorManager.SENSOR_DELAY_NORMAL);
        mSensorManager.registerListener(this, sensor, SensorManager.SENSOR_DELAY_FASTEST);
    }

    public void stopListening() {
        mSensorManager.unregisterListener(this);        //注销传感器事件监听
        mDisplay.setText("Stop!");
    }

    public void StartSensor() {

//		sensor_status = true;
        acc_status = false;
        gyr_status = false;
        mag_status = false;
        Quat[0] = 1;
        Quat[1] = 0;
        Quat[2] = 0;
        Quat[3] = 0;  //初始化四元数
        Euler_angle[0] = 0;
        Euler_angle[1] = 0;
        Euler_angle[2] = 0;

        try {
            //clean the old data in the sensor_data.txt
            File flightDataFile = new File(dir.getAbsolutePath() + File.separator + FileName_flight);
            FileOutputStream fout = new FileOutputStream(flightDataFile);
            fout.close();

            File attitudeDataFile = new File(dir.getAbsolutePath() + File.separator + FileName_attitude);
            FileOutputStream fout1 = new FileOutputStream(attitudeDataFile);
            fout1.close();
        } catch (Exception e) {
            e.printStackTrace();
        }

        sensorRegister(Sensor.TYPE_MAGNETIC_FIELD);
        sensorRegister(Sensor.TYPE_ACCELEROMETER);
        sensorRegister(Sensor.TYPE_GYROSCOPE);
    }

    private double[] getSensorData(double[] sensor_data, SensorEvent event) {

        sensor_data[0] = event.values[0];
        sensor_data[1] = -event.values[1];     //Y测量值从传感器单元坐标系变换到openpilot手机的NED坐标系
        sensor_data[2] = -event.values[2];     //Z测量值从传感器单元坐标系变换到openpilot手机的NED坐标系

        return sensor_data;
    }

//	public void sensor_data_format(double[] data,String name) {
//		
//		DecimalFormat df = new DecimalFormat("####.######");
//		Sensor_message = name +" "+ df.format(data[0])+" ";
//		Sensor_message += df.format(data[1])+" ";
//		Sensor_message += df.format(data[2])+"\n";
//		
//	}


    public void writeFileSdcard(String message, String Filename) {

        try {
            //FileOutputStream fout = openFileOutput(fileName, Context.MODE_APPEND);
            boolean append = true;
            File flightDataFile = new File(dir.getAbsolutePath() + File.separator + Filename);
            FileOutputStream fout = new FileOutputStream(flightDataFile, append);
            byte[] bytes = message.getBytes();

            fout.write(bytes);
            fout.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private String getTime() {
        Time syTime = new Time();
        String timeDisplay = new String();
        syTime.setToNow();
        timeDisplay = syTime.hour + ":" + syTime.minute + ":" + syTime.second;
        return timeDisplay;
    }

    private void BluetoothConnect(final BluetoothDevice device) {
        new Thread(new Runnable() {

            @Override
            public void run() {
                // TODO Auto-generated method stub
                Thread.currentThread().setPriority(Thread.MAX_PRIORITY);
                try {
                    clientSocket = device.createRfcommSocketToServiceRecord(MY_UUID);
                } catch (IOException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
                while (!btc_state) {
                    try {
                        Log.i("Bluetooth State", "bluetooth is being  connected!");
                        clientSocket.connect();
                        if (clientSocket.isConnected()) {
                            Log.i("Bluetooth State", "bluetooth has been connected!");
                            btc_state = true;
                            br = new BufferedReader(new InputStreamReader(clientSocket
                                    .getInputStream()));
//							if(!btr_status){
                            bluetooth_readThread = new Bluetooth_read(br, BtrHandler);
                            bluetooth_readThread.start();           //启动还没有执行实际的功能段代码
//								btr_status = true;
//							}							
                            bluetooth_readThread.threadResume();


                        }
                    } catch (IOException e) {
                        // TODO Auto-generated catch block
                        e.printStackTrace();
                    }

                }

            }
        }).start();
    }

    private void BluetoothDisconnect() {
        new Thread(new Runnable() {

            @Override
            public void run() {
                // TODO Auto-generated method stub
                while (btc_state) {
                    try {
                        bluetooth_readThread.threadStop();
                        clientSocket.close();
                    } catch (IOException e) {
                        // TODO Auto-generated catch block
                        e.printStackTrace();
                    }
                    if (!clientSocket.isConnected()) {
                        btc_state = false;

                    }

                }
            }
        }).start();
    }

    private Handler TimerHandler = new Handler() {
        public void handleMessage(Message msg) {
            String s = (String) msg.obj;
            if (s.equals("Record")) {
                record_cn++;
                Countview.setText("Count: " + record_cn);
            }
            super.handleMessage(msg);
        }
    };

    public Handler BtrHandler = new Handler() {
        public void handleMessage(Message msg) {
//			super.handleMessage(msg);	
            String s = (String) msg.obj;
            if (s.equals("Start 2Hz")) {
                if (!isTimerStart) {
                    //Log.i("DEBUG", "aaaaaaaaaaaaaa");
                    timer.scheduleAtFixedRate(task, 0, 4);
                    //Log.i("DEBUG", "ssssssssssssss");
                    isTimerStart = true;

                }
                record_state = true;
                Log.i("Handle Message", "Triger Start 2HZ");
            } else if (s.equals("Start 4Hz")) {
                if (!isTimerStart) {
                    timer.scheduleAtFixedRate(task, 0, 4);
                    isTimerStart = true;
                }
                record_state = true;
                Log.i("Handle Message", "Triger Start 4HZ");
            } else if (s.equals("Stop")) {
                record_state = false;
//				isTimerStart = false;
                Log.i("Handle Message", "Triger Stop");
            }
//				BtrHandler.getLooper();	
            super.handleMessage(msg);

        }

    };


    public void onPause() {
        super.onPause();

    }

    public void onStop() {
        super.onStop();
        stopListening();
        timer.cancel();
        BluetoothDisconnect();
        attiudeThread.threadStop();
        mLocation.removeUpdates(locationListener);
        if (bluetooth_readThread != null) {
            bluetooth_readThread.threadStop();
        }

    }

    private void ensureDiscoverable() {
        Log.d("Bluetooth search", "ensure discoverable");
        if (bluetoothAdapter.getScanMode() !=
                BluetoothAdapter.SCAN_MODE_CONNECTABLE_DISCOVERABLE) {
            Intent discoverableIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_DISCOVERABLE);
            discoverableIntent.putExtra(BluetoothAdapter.EXTRA_DISCOVERABLE_DURATION, 300);
            startActivity(discoverableIntent);
        }
    }

    public void onActivityResult(int requestCode, int resultCode, Intent data) {
        Log.d("Activity Back", "onActivityResult " + resultCode);
        switch (requestCode) {
            case REQUEST_CONNECT_DEVICE:
                // When DeviceListActivity returns with a device to connect
                if (resultCode == Activity.RESULT_OK) {
                    // Get the device MAC address
                    String address = data.getExtras()
                            .getString(DeviceListActivity.EXTRA_DEVICE_ADDRESS);
                    // Get the BLuetoothDevice object
//	                btdevice = bluetoothAdapter.getRemoteDevice(address);
                    BluetoothDevice device = bluetoothAdapter.getRemoteDevice(address);
                    BluetoothConnect(device);
//	                BluetoothDevice device = bluetoothAdapter.getRemoteDevice(address);
                    // Attempt to connect to the device
//	                mChatService.connect(device);
                }
                break;
//	        case REQUEST_ENABLE_BT:
//	            // When the request to enable Bluetooth returns
//	            if (resultCode == Activity.RESULT_OK) {
//	                // Bluetooth is now enabled, so set up a chat session
//	                setupChat();
//	            } else {
//	                // User did not enable Bluetooth or an error occured
//	                Log.d(TAG, "BT not enabled");
//	                Toast.makeText(this, R.string.bt_not_enabled_leaving, Toast.LENGTH_SHORT).show();
//	                finish();
//	            }
        }
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.option_menu, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
            case R.id.scan:
                // Launch the DeviceListActivity to see devices and do scan
                Intent serverIntent = new Intent(this, DeviceListActivity.class);
                startActivityForResult(serverIntent, REQUEST_CONNECT_DEVICE);
                return true;
            case R.id.discoverable:
                // Ensure this device is discoverable by others
                ensureDiscoverable();
                return true;
        }
        return false;
    }

    ROSBridgeClient client;

    private void connect(String ip, String port) {
        client = new ROSBridgeClient("ws://" + ip + ":" + port);
        boolean conneSucc = client.connect(new ROSClient.ConnectionStatusListener() {
            @Override
            public void onConnect() {
                client.setDebug(true);
                ((RCApplication) getApplication()).setRosClient(client);
                showTip("Connect ROS success");
                doConnect=true;
//                mDisplay.setText("Connect ROS success");
                //Log.d(TAG, "Connect ROS success");
                //startActivity(new Intent(ConnectActivity.this, TopicActivity.class));
            }

            @Override
            public void onDisconnect(boolean normal, String reason, int code) {
                showTip("ROS disconnect");
//                mDisplay.setText("ROS disconnect");
                //Log.d(TAG, "ROS disconnect");
            }

            @Override
            public void onError(Exception ex) {
                ex.printStackTrace();
                showTip("ROS communication error");
//                mDisplay.setText("ROS communication error");
                //Log.d(TAG, "ROS communication error");
            }
        });
    }
    private void showTip(final String tip) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(SensorActivity.this, tip,Toast.LENGTH_SHORT).show();
            }
        });
    }

}
