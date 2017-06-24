package com.zthit.AHRS;

import java.io.BufferedReader;
import java.io.IOException;

import android.os.Handler;
import android.os.Message;
import android.util.Log;

public class Bluetooth_read extends Thread{
	private boolean isRun = true;
    private boolean isWait = true;
    boolean isStart = false;
    private BufferedReader br;
    public Handler BtrHandler;
	public Bluetooth_read(BufferedReader br,Handler BtrHandler) {
		this.br = br;
		this.BtrHandler = BtrHandler;
	}
	
	public void run() {
		super.run();
		this.setName("Bluetooth Read");	
		Message msg = new Message();
		while(isRun){
			try {
				synchronized (this) {
					while(isWait){			
						try {
							wait();
						} catch (InterruptedException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
					} 
				}
				if(!isStart){				
//					Message msg = new Message();
					msg.obj = "No Triger";
					BtrHandler.sendMessage(msg);
					isStart = true;
//					msg.recycle();
					
				}
				else{
//					Message msg = new Message();
					String res = br.readLine();
					Log.i("Bluetooth read",res);
					if(res.length()>0){ 
						msg.obj = res;
						BtrHandler.sendMessage(msg);					
					}
				}
				
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			msg = Message.obtain();         //释放message队列资源
		}
	}
	
	public void threadStop() {
		isRun = false;
		isStart = false;
	}
	
	//线程暂定
		public void threadPause() {
			isWait = true;
			isStart = false;
		}
		
		//线程唤醒
		public synchronized void threadResume() {
			isWait = false;
			notify();
		}
}
