package com.zthit.AHRS;

import java.text.DecimalFormat;

public class TcpProtocol {
	//姿态数据传输协议，58901为数据传输的头，用来表征传输的是一组新的数据
	//便于服务器端判断，接受处理
	public static String Quat_format(double[] q) {
		DecimalFormat df = new DecimalFormat("#0.00");
		String s = "58901";
		
		for(int i=0;i<q.length;i++){
			if(q[i]>=0){
				s += "+" + df.format(q[i]);
			}
			else {
				s += df.format(q[i]);
			}			
		}
		return s;
		
	}
}	
