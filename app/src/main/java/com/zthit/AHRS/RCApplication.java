package com.zthit.AHRS;
import android.app.Application;
import com.jilk.ros.rosbridge.ROSBridgeClient;
/**
 * Created by chris on 17-6-14.
 */

public class RCApplication extends Application{
    ROSBridgeClient client;

    @Override
    public void onCreate() {
        super.onCreate();
    }

    @Override
    public void onTerminate() {
        if(client != null)
            client.disconnect();
        super.onTerminate();
    }

    public ROSBridgeClient getRosClient() {
        return client;
    }

    public void setRosClient(ROSBridgeClient client) {
        this.client = client;
    }
}
