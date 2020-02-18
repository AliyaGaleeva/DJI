package test.lab.vision;

import android.app.Application;
import android.content.Context;
import android.util.Log;

import com.secneo.sdk.Helper;

import org.opencv.android.OpenCVLoader;
//import com.squareup.leakcanary.LeakCanary;

public class MApplication extends Application {

    private DJIApplication demoApplication;
    @Override
    protected void attachBaseContext(Context paramContext) {
        super.attachBaseContext(paramContext);
        Helper.install(MApplication.this);
        if (demoApplication == null) {
            demoApplication = new DJIApplication();
            demoApplication.setContext(this);
        }
    }

    @Override
    public void onCreate() {
        super.onCreate();
//        if (LeakCanary.isInAnalyzerProcess(this)) {
//            // This process is dedicated to LeakCanary for heap analysis.
//            // You should not init your app in this process.
//            return;
//        }
//        LeakCanary.install(this);
////         Normal app init code...
//        System.loadLibrary("opencv_java4");
        if (!OpenCVLoader.initDebug())
            Log.e("OpenCv", "Unable to load OpenCV");
        else
            Log.d("OpenCv", "OpenCV loaded");
        demoApplication.onCreate();
    }

}
