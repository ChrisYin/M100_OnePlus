package com.zthit.AHRS;

import android.app.Activity;
import android.content.Intent;
import android.hardware.SensorEventListener;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;


public class GyroActivity extends Activity implements View.OnClickListener {

    public Button mBack,mSave,mCalibration;
    public TextView Magview;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_gyro);
        mBack = (Button) findViewById(R.id.Back);
        mBack.setOnClickListener(this);
        Magview = (TextView)findViewById(R.id.GyroView);
        mSave = (Button) findViewById(R.id.Save);
        mSave.setOnClickListener(this);
        mCalibration = (Button) findViewById(R.id.GyroCalibrate);
        mCalibration.setOnClickListener(this);
    }
    @Override
    public void onClick(View v) {
        if(v.getId() == R.id.Back){

            Intent intent = new Intent();
            intent.setClass(GyroActivity.this, SensorActivity.class);
            GyroActivity.this.startActivity(intent);
            GyroActivity.this.finish();
        }
        if(v.getId() == R.id.GyroCalibrate){
            Magview.setText("Calibration Successfully !");
        }
        if(v.getId() == R.id.Save){
            Magview.setText("Saving data Successfully !");
        }
    }

}
