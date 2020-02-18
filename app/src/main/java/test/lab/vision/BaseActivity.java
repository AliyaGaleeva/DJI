package test.lab.vision;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.graphics.Bitmap;
import android.graphics.Matrix;
import android.graphics.SurfaceTexture;
import android.os.Bundle;
import android.os.Handler;
import android.support.v4.app.FragmentActivity;
import android.util.Log;
import android.view.TextureView;
import android.view.TextureView.SurfaceTextureListener;
import android.view.View;
import android.widget.TextView;
import android.widget.Toast;

import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.nio.ByteBuffer;
import java.util.ArrayList;

import dji.common.product.Model;
import dji.sdk.base.BaseProduct;
import dji.sdk.camera.Camera;
import dji.sdk.camera.VideoFeeder;
import dji.sdk.codec.DJICodecManager;
import dji.sdk.products.Aircraft;

public class BaseActivity extends FragmentActivity implements SurfaceTextureListener,  DJICodecManager.YuvDataCallback{

    private static final String TAG = MainActivity.class.getName();
    protected VideoFeeder.VideoDataListener mReceivedVideoDataListener = null;
    protected DJICodecManager mCodecManager = null;
    private BaseProduct mProduct;

    //To store index chosen in PopupNumberPicker listener
    protected static int[] INDEX_CHOSEN = {-1, -1, -1};

    protected TextView mConnectStatusTextView;

    protected TextureView mVideoSurface = null;

    Thread mThread;
    static Bitmap[] mBitmaps = new Bitmap[1];
    static Mat[] mMat = new Mat[1];
    boolean recognitionIsLocked = false; // когда объект пойман нужно выключить распознавание чтобы освободить процессор. сейчас этот флаг не используется

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
         
        IntentFilter filter = new IntentFilter();  
        filter.addAction(DJIApplication.FLAG_CONNECTION_CHANGE);
        registerReceiver(mReceiver, filter);

        mVideoSurface = (TextureView) findViewById(R.id.video_previewer_surface); // дрон drone
        mConnectStatusTextView = (TextView) findViewById(R.id.ConnectStatusTextView);

        if (null != mVideoSurface) {
            mVideoSurface.setSurfaceTextureListener(this);
        }

        // The callback for receiving the raw H264 video data for camera live view
        mReceivedVideoDataListener = new VideoFeeder.VideoDataListener() {

            @Override
            public void onReceive(byte[] videoBuffer, int size) {
//                Log.d("frame_update", "frame updated 2 ");
                if(mCodecManager != null){
                    mCodecManager.sendDataToDecoder(videoBuffer, size);
                }
            }
        };
    }


    protected BroadcastReceiver mReceiver = new BroadcastReceiver() {

        @Override
        public void onReceive(Context context, Intent intent) {
            updateTitleBar();
            onProductChange();
        }
        
    };
    
    protected void onProductChange() {
        initPreviewer();
    }
    
    @Override
    protected void onStart() {
        super.onStart();

    }

    @Override
    protected void onResume() {
        Log.e(TAG, "onResume");
        super.onResume();
        updateTitleBar();
        initPreviewer();
        onProductChange();

        if(mVideoSurface == null) {
            Log.e(TAG, "mVideoSurface is null");
        }
    }
    
    @Override
    protected void onPause() {
        Log.e(TAG, "onPause");
        uninitPreviewer();
        super.onPause();
    }

    @Override
    protected void onStop() {
        Log.e(TAG, "onStop");
        super.onStop();
    }

    @Override
    protected void onDestroy() {
        Log.e(TAG, "onDestroy");
        unregisterReceiver(mReceiver);
        uninitPreviewer();
        super.onDestroy();
    }
    
    private void showToast(final String msg) {
        runOnUiThread(new Runnable() {
            public void run() {
                Toast.makeText(BaseActivity.this, msg, Toast.LENGTH_SHORT).show();
            }
        });
    }
    
    private void initPreviewer() {
        try {
            mProduct = DJIApplication.getProductInstance();
        } catch (Exception exception) {
            mProduct = null;
        }
        
        if (mProduct == null || !mProduct.isConnected()) {
            Log.d(TAG, "Disconnect");
        } else {
            if (null != mVideoSurface) {
                mVideoSurface.setSurfaceTextureListener(this);
            }

            if (!mProduct.getModel().equals(Model.UNKNOWN_AIRCRAFT)) {
                VideoFeeder.getInstance().getPrimaryVideoFeed().addVideoDataListener(mReceivedVideoDataListener);
            }
        }
    }

    private void uninitPreviewer() {
        Camera camera = DJIApplication.getCameraInstance();
        if (camera != null){
            // Reset the callback
            VideoFeeder.getInstance().getPrimaryVideoFeed().removeVideoDataListener(mReceivedVideoDataListener);
        }
    }
    
    /**
     * @param surface
     * @param width
     * @param height
     * @see SurfaceTextureListener#onSurfaceTextureAvailable(SurfaceTexture,
     *      int, int)
     */
    @Override
    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
        if (mCodecManager == null) {
            mCodecManager = new DJICodecManager(this, surface, width, height);
//            mCodecManager.enabledYuvData(true);
//            mCodecManager.setYuvDataCallback(this);
        }
    }

    /**
     * @param surface
     * @param width
     * @param height
     * @see SurfaceTextureListener#onSurfaceTextureSizeChanged(SurfaceTexture,
     *      int, int)
     */
    @Override
    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {

    }

    /**
     * @param surface
     * @return
     * @see SurfaceTextureListener#onSurfaceTextureDestroyed(SurfaceTexture)
     */
    @Override
    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
        if (mCodecManager != null)
            mCodecManager.cleanSurface();
        return false;
    }

    /**
     * @param surface
     * @see SurfaceTextureListener#onSurfaceTextureUpdated(SurfaceTexture)
     */
    long frameCounter = 0;
    @Override
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
        // TODO uncomment to start recognition processing

        Log.d("frame_update", "frame updated frameCounter = " + frameCounter );
        frameCounter ++;
        if (frameCounter % 13 != 0){ // разрешает каждый 13-й кадр и задерживает все остальные
            return;
        }
        frameCounter = 0;
        mThread = new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    mBitmaps[0] = mVideoSurface.getBitmap(); // получаем изображение камеры дрона и конвертируем его в объект типа Mat для OpenCV
                    mBitmaps[0] = getResizedBitmap(mBitmaps[0], 1280, 720);
                    mMat[0] = new Mat();
                    Utils.bitmapToMat(mBitmaps[0], mMat[0]);
                    handler.sendEmptyMessage(1); // отправляем сообщение что данные готовы
                }catch (OutOfMemoryError e){

                }
            }
        });
        mThread.start();
        Log.d("frame_update", "frame updated 1 mThread.start()" );
    }

    public Bitmap getResizedBitmap(Bitmap bitmap, int newWidth, int newHeight) {
        if (bitmap == null) return null;
        int width = bitmap.getWidth();
        int height = bitmap.getHeight();
        float scaleWidth = ((float) newWidth) / width;
        float scaleHeight = ((float) newHeight) / height;
        Matrix matrix = new Matrix();
        matrix.postScale(scaleWidth, scaleHeight);
        return Bitmap.createBitmap(bitmap, 0, 0, width, height, matrix, false);
    }


    private void updateTitleBar() {
        if(mConnectStatusTextView == null) return;
        boolean ret = false;
        BaseProduct product = DJIApplication.getProductInstance();
        if (product != null) {
            if(product.isConnected()) {
                mConnectStatusTextView.setText(DJIApplication.getProductInstance().getModel().getDisplayName() + " Connected");
                ret = true;
            } else {
                if(product instanceof Aircraft) {
                    Aircraft aircraft = (Aircraft)product;
                    if(aircraft.getRemoteController() != null) {
                    }
                    if(aircraft.getRemoteController() != null && aircraft.getRemoteController().isConnected()) {
                        mConnectStatusTextView.setText("only RC Connected");
                        ret = true;
                    }
                }
            }
        }
        
        if(!ret) {
//            mConnectStatusTextView.setText("Disconnected");
        }
    }

    /**
     * @Description : RETURN BTN RESPONSE FUNCTION
     * @author : andy.zhao
     * @param view
     * @return : void
     */
    public void onReturn(View view) {
        this.finish();
    }
    
    public void resetIndex() {
        INDEX_CHOSEN = new int[3];
        INDEX_CHOSEN[0] = -1;
        INDEX_CHOSEN[1] = -1;
        INDEX_CHOSEN[2] = -1;
    }
    
    public ArrayList<String> makeListHelper(Object[] o) {
        ArrayList<String> list = new ArrayList<String>();
        for (int i = 0; i < o.length - 1; i++) {
            list.add(o[i].toString());
        }
        return list;
    }


    protected void getBitmapFromStream(Mat mat){ // вызываем одноименный метод в TrackingActivity
    }

    private Handler handler;
    {
        handler = new Handler() {
            public void handleMessage(android.os.Message msg) { // получаем сообщение и вызываем метод для дальнейшей обработки
                getBitmapFromStream(mMat[0]);
            }
        };
    }

    @Override
    public void onYuvDataReceived(ByteBuffer byteBuffer, int i, int i1, int i2) {

    }
}
