package test.lab.vision;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.graphics.Color;
import android.graphics.RectF;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.os.CountDownTimer;
import android.os.Handler;
import android.support.annotation.NonNull;
import android.util.Log;
import android.view.SurfaceView;
import android.view.TextureView;
import android.view.TextureView.SurfaceTextureListener;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.Window;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.FrameLayout;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.RelativeLayout;
import android.widget.SlidingDrawer;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;

import com.felhr.usbserial.UsbSerialDevice;
import com.felhr.usbserial.UsbSerialInterface;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.io.UnsupportedEncodingException;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import dji.common.error.DJIError;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.flightcontroller.FlightOrientationMode;
import dji.common.flightcontroller.virtualstick.FlightControlData;
import dji.common.flightcontroller.virtualstick.FlightCoordinateSystem;
import dji.common.flightcontroller.virtualstick.RollPitchControlMode;
import dji.common.flightcontroller.virtualstick.VerticalControlMode;
import dji.common.flightcontroller.virtualstick.YawControlMode;
import dji.common.gimbal.GimbalState;
import dji.common.gimbal.Rotation;
import dji.common.gimbal.RotationMode;
import dji.common.mission.activetrack.ActiveTrackMission;
import dji.common.mission.activetrack.ActiveTrackMissionEvent;
import dji.common.mission.activetrack.ActiveTrackMode;
import dji.common.mission.activetrack.ActiveTrackState;
import dji.common.mission.activetrack.ActiveTrackTargetState;
import dji.common.mission.activetrack.ActiveTrackTrackingState;
import dji.common.util.CommonCallbacks.CompletionCallback;
import dji.log.DJILog;
import dji.midware.media.DJIVideoDataRecver;
import dji.sdk.base.BaseProduct;
import dji.sdk.flightcontroller.FlightAssistant;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.gimbal.Gimbal;
import dji.sdk.mission.MissionControl;
import dji.sdk.mission.activetrack.ActiveTrackMissionOperatorListener;
import dji.sdk.mission.activetrack.ActiveTrackOperator;
import dji.sdk.products.Aircraft;
import dji.sdk.sdkmanager.DJISDKManager;

public class TrackingActivity extends BaseActivity implements CvCameraViewListener2, SurfaceTextureListener, OnClickListener,/* OnTouchListener,*/ ActiveTrackMissionOperatorListener, VisionProcessCoordinatesCallBack {

    //********************************************************** OpenCV Segment ************************************************************

    public final static int MAX_FRAME_WIDTH = 1280; // ширина экрана
    public final static int MAX_FRAME_HEIGHT = 720;//960; // высота экрана
    public static int MAXIMUM_PERMITTED_ALTITUDE = 100; // максимальная разрешенная высота полета дрона
    public final static float DRONE_MISSION_HEIGHT = 2.3f; // высота текущего полета дрона в метрах
    private static final String TAG = "TrackingActivity";
    private final String MISSION_TYPE_SEARCH = "search_marker"; // тип миссии
    private final String MISSION_TYPE_LEND = "lend_aircraft"; // тип миссии
    private final int TARGET_CONFIRMATION_NUMBER = 1;

    private String ACTION_USB_PERMISSION = "me.ngargi.engr100_vision.USB_PERMISSION";
    private String mErrorInformation;
    private String mErrorReason;
    private String MISSION_TYPE = "MISSION_NOT_ACTIVE"; // миссия первоначально в статусе "неактивна"
    private String MISSION_STATE = "IDLE"; // в эту переменную попадает текущий статус миссии по которому мы отслеживаем потерю метки во время полета дрона на этапе приземления

    private Scalar DRAW_COLOR;

    private UsbSerialDevice serialPort;
    private UsbManager usbManager;
    private UsbDevice device;
    private UsbDeviceConnection connection;

    private RectF mRectF;

    private Button btn_source;
    private Button btn_start_yaw;
    private Button btn_stop_yaw;
    private Button btn_reset_controller;
    private Button btn_set_stick_mode;
    private Button btn_forward;
    private Button btn_stop;
    private Button btn_right;
    private Button btn_stop_lending;
    private Button btn_a;
    private Button btn_4;
    private Button btn_5;
    private Button btn_6;
    private Button btn_7;

    private CameraBridgeViewBase mOpenCvCameraView;

    private boolean sourceFromCamera = true;
    private boolean missionIsActive = false; // блокируем передачу координат повторно. передаем только один раз
    private boolean targetConfirmationIsLocked = false; // блокируем повторное подтверждение, пока не пришел колбак предыдущего подтверждения
    private boolean mCanRepeatRecognition = true; // пока false метод не запустится
    private boolean imageIsLocked = true; // пока дрон не взлетел на нужную высоту, блокируем распознавание
    private boolean autoLendingActivated = false;
    private boolean markerIsDetected = false; // показывает был ли распознан маркер
    private boolean mUseCoordinate = false; // если false то заменяет значения в параметрах на 0; дрон останавливается.

    private FrameLayout java_camera_view_layout;

    private Mat mDroneMat; // изображение полученное с камеры дрона
    private Mat mGray = new Mat();
    private Mat mIds = new Mat();
    private Mat mRgba;

    private List<Mat> allCorners = new ArrayList<>();
    private List<Mat> rejected = new ArrayList<>();
    private List<Marker> mMarkersList = new ArrayList<>();

    private FlightController mFlightController;

    private int DRONE_FLIGHT_TIME = 0; // время полета дрона в миллисекундах
    private int DRONE_FREEZE_TIME = 5000; // время неподвижного состояния дрона в миллисекундах, стояло 7000, но нужно подобрать практически
    private int mMarkerID = -1; // идентификатор маркера пока пустой
    private int mMissionIndex = -1; // индекс структуры с параметрами движения в массиве
    private int confirmationAttemptCounter = 0;
    private int lendingStepIndex = 0; // индекс массива с названием шага посадки
    private int counterOfSearchCircles = 0;
    private int mOpenCVRectangleWidth;
    private int mOpenCVRectangleHeight;
    private int center_x_color = 0;
    private int center_y_color = 0;
    private int pitch_color = 0;
    private int targetConfirmationCounter = 0;

    private float mPitch = 0f; // движение вперед
    private float mRoll = 0f; // движение в сторону
    private float mYaw = 0f; // поворот
    private float mThrottle = DRONE_MISSION_HEIGHT; // высота полета при следовании за маркером
    private float mFlightSpeed = 0.7f; // скорость полета дрона в м.сек*/
    private float mTrackingRectangleWidth;
    private float mTrackingRectangleHeight;
    private float mUltrasonicHeight = 0f; // высота с ультразвукового датчика
    private float mGPSHeight = 0; // высота с другого датчика
    private float mCameraPitch = 0f; // угол наклона камеры
    private float lendingHeight = DRONE_MISSION_HEIGHT; // задумана как высота для приземления. на это значение должно осуществляться снижение. но пока это просто хардкод.
    private float center_X = 0f; // центр метки автономного слежения дрона на экране
    private float center_Y = 0f; // центр метки автономного слежения дрона на экране
    private float ROBOT_PRELANDING_HEIGHT = 1.0f; // высота робота плюс 0.5 метра

    private Timer mSendVirtualStickDataTimer; // таймер генератор 10 Герц
    private CountDownTimer mMissionDataUpdateTimer; // таймер обновления координаты (1 раз в DRONE_FREEZE_TIME секунд)
    private CountDownTimer mFlightTimer; // таймер для запуска/остановки полета
    private Timer screenUpdateTimer; // таймер для обновления данных на экране 3 герца

    private List<MissionCoordinate> mMissionCoordinateList; // массив координат
    private SendVirtualStickDataTask mSendVirtualStickDataTask;
    private UpdateScreenTask mUpdateScreenTask;

    private TextureView video_previewer_surface;

    private ImageView detection_status_view;
    private ImageView mission_status_view;

    private TextView tracking_status_info;
    private TextView confirm_target_info;
    private TextView coordinates_info;
    private TextView selected_rectangle_info; // размеры распознанного маркера
    private TextView movement_direction;
    private TextView missionEventStateInfo; // статус ActiveTrackMissionEvent
    private TextView controller_state_text_view; // (ultrasonicHeight + altitudeHeight);
    private TextView flight_info_text_view;
    private TextView text;
    private TextView gimbal_state_text_view; // текущий угол наклона камеры
    private TextView center_x_text_view; // центр по оси Х
    private TextView center_y_text_view; // центр по оси У
    private TextView pitch_text_view;

    private Aircraft mAircraft;

    private Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_ARUCO_ORIGINAL);
    private DetectorParameters detectorParameters = DetectorParameters.create();
    private Handler mHandler = new Handler();
    final boolean[] takeOffSuccess = {false}; // флаг успешного взлета


    UsbSerialInterface.UsbReadCallback mCallback = new UsbSerialInterface.UsbReadCallback() { //Defining a Callback which triggers whenever data is read.
        @Override
        public void onReceivedData(byte[] arg0) {
            String data = null;
            try {
                data = new String(arg0, "UTF-8");
                data.concat("/n");
                final String d = data;
//                runOnUiThread(() -> text.append(d));

            } catch (UnsupportedEncodingException e) {
                e.printStackTrace();
            }
        }
    };


    public TrackingActivity() {
        Log.i(TAG, "Instantiated new " + this.getClass());
    }

    /**
     * Called when the activity is first created.
     */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        Log.i(TAG, "called onCreate");
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        setContentView(R.layout.activity_tracking_test);
        super.onCreate(savedInstanceState);

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        initUI();
        text = findViewById(R.id.textView);
        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.color_blob_detection_activity_surface_view);
        mOpenCvCameraView.setMaxFrameSize(MAX_FRAME_WIDTH, MAX_FRAME_HEIGHT);
        mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
        mOpenCvCameraView.setCvCameraViewListener(this);
        btn_source = findViewById(R.id.btn_source);
        btn_source.setOnClickListener(this);
        java_camera_view_layout = findViewById(R.id.java_camera_view_layout);
        video_previewer_surface = findViewById(R.id.video_previewer_surface);
        detection_status_view = findViewById(R.id.detection_status_view);
        mission_status_view = findViewById(R.id.mission_status_view);
        serialPort = null;
        usbManager = (UsbManager) getSystemService(Context.USB_SERVICE);
        IntentFilter filter = new IntentFilter();
        filter.addAction(ACTION_USB_PERMISSION);
        filter.addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED);
        filter.addAction(UsbManager.ACTION_USB_DEVICE_DETACHED);
        registerReceiver(broadcastReceiver, filter);
        flight_info_text_view = findViewById(R.id.flight_info_text_view);
        btn_start_yaw = findViewById(R.id.btn_start_yaw);
        btn_start_yaw.setOnClickListener(this);
        btn_stop_yaw = findViewById(R.id.btn_stop_yaw);
        btn_stop_yaw.setOnClickListener(this);
        btn_reset_controller = findViewById(R.id.btn_reset_controller);
        btn_reset_controller.setOnClickListener(this);
        btn_set_stick_mode = findViewById(R.id.btn_set_stick_mode);
        btn_set_stick_mode.setOnClickListener(this);
        tracking_status_info = findViewById(R.id.tracking_status_info);
        confirm_target_info = findViewById(R.id.confirm_target_info);
        coordinates_info = findViewById(R.id.coordinates_info);
        selected_rectangle_info = findViewById(R.id.selected_rectangle_info);
        movement_direction = findViewById(R.id.movement_direction);
        btn_forward = findViewById(R.id.btn_forward);
        btn_forward.setOnClickListener(this);
        btn_stop = findViewById(R.id.btn_stop);
        btn_stop.setOnClickListener(this);
        btn_right = findViewById(R.id.btn_right);
        btn_right.setOnClickListener(this);
        missionEventStateInfo = findViewById(R.id.mission_event_text_view);
        pitch_text_view = findViewById(R.id.pitch_text_view);
        gimbal_state_text_view = findViewById(R.id.gimbal_state_text_view);
        controller_state_text_view = findViewById(R.id.controller_state_text_view);
        btn_stop_lending = findViewById(R.id.btn_stop_lending);
        btn_stop_lending.setOnClickListener(this);
        center_x_text_view = findViewById(R.id.center_x_text_view);
        center_y_text_view = findViewById(R.id.center_y_text_view);

        btn_a = findViewById(R.id.btn_a);
        btn_4 = findViewById(R.id.btn_4);
        btn_5 = findViewById(R.id.btn_5);
        btn_6 = findViewById(R.id.btn_6);
        btn_7 = findViewById(R.id.btn_7);
        btn_a.setOnClickListener(this);
        btn_4.setOnClickListener(this);
        btn_5.setOnClickListener(this);
        btn_6.setOnClickListener(this);
        btn_7.setOnClickListener(this);

        activateScreenUpdateTimer();

    }

    private Aircraft getAircraft(){
        BaseProduct product = DJIApplication.getProductInstance();
        if(product instanceof Aircraft) {
            return (Aircraft) product;
        }
        return null;
    }

    private CountDownTimer createMissionDataUpdateTimer() {
        CountDownTimer countDownTimer = new CountDownTimer(3600000, DRONE_FREEZE_TIME) {
            @Override
            public void onTick(long l) {
                if (MISSION_TYPE.equals(MISSION_TYPE_SEARCH)) { // миссия поиска
                    if (mMissionIndex < mMissionCoordinateList.size() - 1) {
                        float angle = 0f;
                        if (counterOfSearchCircles == 0) { // первый круг поиска
                            angle = -55f;
                            rotateGimbalToAbsoluteAngle(angle);
                        }else if(counterOfSearchCircles == 1){ // второй круг поиска
                            angle = -30f;
                            rotateGimbalToAbsoluteAngle(angle);
                        }else {
                            if (!markerIsDetected) {
                                mMissionCoordinateList = null;// если прошли два круга и ничего не нашли, останавливаем поиск. дрон зависает в воздухе
                                cancelMissionDataUpdateAndFlightTimers(); //
                                destroyVirtualStickDataTimer();
                                resetController(); // переводим в режим ручного управления
                            }
                        }

                        mUseCoordinate = true; // взять координату из массива
                        mMissionIndex += 1; // индекс координаты в массиве
                        if (mMissionCoordinateList != null && mMissionIndex == mMissionCoordinateList.size() - 1){ // увеличиваем индекс после прохода полного круга
                            counterOfSearchCircles += 1;
                        }
                        if (mFlightTimer != null) {
                            mFlightTimer.start(); // запустить таймер времени полета
                        }
                    } else {
                        mMissionIndex = -1;
                    }
                }else if (MISSION_TYPE.equals(MISSION_TYPE_LEND)){ // миссия посадка

                }
            }

            @Override
            public void onFinish() {
                if (MISSION_TYPE.equals(MISSION_TYPE_SEARCH)) { // миссия поиска
                }else if (MISSION_TYPE.equals(MISSION_TYPE_LEND)){ // миссия посадка

                }
            }
        };
        return countDownTimer;
    }

    private CountDownTimer createFlightTimer(){
        CountDownTimer countDownTimer = new CountDownTimer(DRONE_FLIGHT_TIME, DRONE_FLIGHT_TIME) {
            @Override
            public void onTick(long l) {
                if (MISSION_TYPE.equals(MISSION_TYPE_SEARCH)) { // миссия поиска
                }else if (MISSION_TYPE.equals(MISSION_TYPE_LEND)){ // миссия посадка

                }
            }

            @Override
            public void onFinish() {
                if (MISSION_TYPE.equals(MISSION_TYPE_SEARCH)) { // миссия поиска
//                    mUseCoordinate = false; // обнулить координату чтобы остановить движение
                }else if (MISSION_TYPE.equals(MISSION_TYPE_LEND)){ // миссия посадка
                    if (mMissionCoordinateList != null) {
                        mMissionCoordinateList.clear(); // удаляем задание из списка после его выполнения
                        mMissionCoordinateList = null;
                    }
                }
            }
        };
        return countDownTimer;
    }

    @Override
    public void onPause() {
        super.onPause();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
        unregisterReceiver(broadcastReceiver);
    }

    @Override
    public void onResume() {
        super.onResume();
        IntentFilter filter = new IntentFilter();
        filter.addAction(ACTION_USB_PERMISSION);
        filter.addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED);
        filter.addAction(UsbManager.ACTION_USB_DEVICE_DETACHED);
        registerReceiver(broadcastReceiver, filter);
        initMissionManager();
        mOpenCvCameraView.enableView();
        mOpenCvCameraView.enableFpsMeter();
    }


    public void onDestroy() {
        destroyVirtualStickDataTimer();
        try {
            DJIVideoDataRecver.getInstance().setVideoDataListener(false, null);
        } catch (Exception e) {
            e.printStackTrace();
        }

        if (mActiveTrackOperator != null) {
            mActiveTrackOperator.removeListener(this);
        }

        if (mCodecManager != null) {
            mCodecManager.destroyCodec();
        }
        super.onDestroy();
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
        screenUpdateTimer.cancel();
        screenUpdateTimer.purge();
        screenUpdateTimer = null;

        mMissionCoordinateList = null;
        cancelMissionDataUpdateAndFlightTimers();
        destroyVirtualStickDataTimer(); // выключить SendVirtualStickDataTask
    }

    private void destroyVirtualStickDataTimer(){
        if (null != mSendVirtualStickDataTimer) {
            mSendVirtualStickDataTask.cancel();
            mSendVirtualStickDataTask = null;
            mSendVirtualStickDataTimer.cancel();
            mSendVirtualStickDataTimer.purge();
            mSendVirtualStickDataTimer = null;
        }
    }

    @Override
    protected void onStop() {
        super.onStop();
    }

    public void onCameraViewStarted(int width, int height) {
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        DRAW_COLOR = new Scalar(255, 0, 0, 255);
    }

    public void onCameraViewStopped() {
        if (mRgba != null) {
            mRgba.release();
        }
    }


    private final BroadcastReceiver broadcastReceiver = new BroadcastReceiver() { //Broadcast Receiver to automatically start and stop the Serial connection.
        @Override
        public void onReceive(Context context, Intent intent) {
            if (intent.getAction().equals(ACTION_USB_PERMISSION)) {
                boolean granted =
                        intent.getExtras().getBoolean(UsbManager.EXTRA_PERMISSION_GRANTED);
                if (granted) {
                    connection = usbManager.openDevice(device);
                    serialPort = UsbSerialDevice.createUsbSerialDevice(device, connection);
                    if (serialPort != null) {
                        if (serialPort.open()) { //Set Serial Connection Parameters.
//                            setUiEnabled(true); //Enable Buttons in UI
                            serialPort.setBaudRate(9600);
                            serialPort.setDataBits(UsbSerialInterface.DATA_BITS_8);
                            serialPort.setStopBits(UsbSerialInterface.STOP_BITS_1);
                            serialPort.setParity(UsbSerialInterface.PARITY_NONE);
                            serialPort.setFlowControl(UsbSerialInterface.FLOW_CONTROL_OFF);
                            serialPort.read(mCallback); //
                            text.append("Serial connection established!");
//                            runOnUiThread(new updateUIThread("Serial connection established!"));
//                            tvAppend(textView,"Serial Connection Opened!\n");

                        } else {
                            text.append("port not open");
                        }
                    } else {
                        text.append("port is null");
                    }
                } else {
                    text.append("no perm");
                }
            } else if (intent.getAction().equals(UsbManager.ACTION_USB_DEVICE_ATTACHED)) {
//                checkUSB(findViewById(R.id.usb_button));
            } else if (intent.getAction().equals(UsbManager.ACTION_USB_DEVICE_DETACHED)) {
//                onClickStop(stopButton);
            }
        }
    };


    public Mat onCameraFrame(CvCameraViewFrame inputFrame) { // на каждый такт обновления экрана запускаем распознавание
        if (!sourceFromCamera) {
            if (!imageIsLocked) {
                detectMarker(mDroneMat, null);
            }
            return mDroneMat;
        }
        mRgba = inputFrame.rgba();
        mGray = inputFrame.gray();
        detectMarker(mRgba, mGray);

        return mRgba;
    }

    private void detectMarker(Mat rgba, Mat gray){
        if (rgba == null){
            return;
        }
        allCorners.clear();
        rejected.clear();
        mMarkersList.clear();
        if (gray == null){
            gray = new Mat();
            Imgproc.cvtColor(rgba, gray, Imgproc.COLOR_BGRA2GRAY);
        }
        Aruco.detectMarkers(gray, dictionary, allCorners, mIds, detectorParameters, rejected);
        gray.release();
        if (!mIds.empty()) {
            for (int i = 0; i < mIds.rows(); i++) {
                Mat markerCorners = allCorners.get(i);
                int ID = (int) mIds.get(i, 0)[0];
                Marker marker = new Marker(markerCorners, ID);
                marker.draw(rgba);
                mMarkersList.add(marker);
            }
        }
        if (mMarkerID == -1){ // если маркер еще не распознавался
            if (mMarkersList.size() == 1) {
                mMarkerID = mMarkersList.get(0).getID(); // запоминаем маркер, который показали
            }
        }

        Marker marker = findMarkerWithId(mMarkerID); // ищем среди распознанных маркеров наш маркер
        getCoordinatesAndUpdate(marker); // получам из маркера координаты и передаем их дрону
    }

    private Marker findMarkerWithId(int id){
        for (Marker marker : mMarkersList){
            if (marker.getID() == id){ // ищем маркер с сохраненным ID
                return marker;
            }
        }
        return null;
    }

    private void getCoordinatesAndUpdate(Marker marker){
        if (marker == null){
            onCoordinateUpdate(null, 0f, 0f); // нет маркера - нет координат
        }else {
            float[] coordinates = new float[4];
            Point[] corner = marker.getMarkerCorners();
            View parent = (View) mSendRectIV.getParent();
            float screenWidth =  (float) parent.getWidth();
            float screenHeight = (float) parent.getHeight();
            double heading = marker.getHeading(); // показывает где находится перед маркера

            float left = 0; // left
            float top = 0; // top
            float right = 0; // right
            float bottom = 0; // bottom

            if (heading > -3.14 && heading < -1.56) {
                left = (float) corner[0].x * screenWidth / (float) MAX_FRAME_WIDTH;// left
                top = (float) corner[1].y * screenHeight / (float) MAX_FRAME_HEIGHT; // top
                right = (float) corner[2].x * screenWidth / (float) MAX_FRAME_WIDTH; // right
                bottom = (float) corner[3].y * screenHeight / (float) MAX_FRAME_HEIGHT; // bottom
            }else if (heading < 3.1415 && heading > 1.57) {
                left = (float) corner[3].x * screenWidth / (float) MAX_FRAME_WIDTH; // left
                top = (float) corner[0].y * screenHeight / (float) MAX_FRAME_HEIGHT; // top
                right = (float) corner[1].x * screenWidth / (float) MAX_FRAME_WIDTH; // right
                bottom = (float) corner[2].y * screenHeight / (float) MAX_FRAME_HEIGHT;//* heightCoef; // bottom
            }else if (heading < 0 && heading >= -1.56) {
                left = (float) corner[1].x * screenWidth / (float) MAX_FRAME_WIDTH;//widthCoef; // left
                top = (float) corner[2].y * screenHeight / (float) MAX_FRAME_HEIGHT;//*  heightCoef; // top
                right = (float) corner[3].x * screenWidth / (float) MAX_FRAME_WIDTH;//widthCoef; // right
                bottom = (float) corner[0].y * screenHeight / (float) MAX_FRAME_HEIGHT;//* heightCoef; // bottom
            }else if (heading >= 0 && heading <= 1.57){
                left = (float) corner[2].x * screenWidth / (float) MAX_FRAME_WIDTH;//widthCoef; // left
                top = (float) corner[3].y * screenHeight / (float) MAX_FRAME_HEIGHT;//*  heightCoef; // top
                right = (float) corner[0].x * screenWidth / (float) MAX_FRAME_WIDTH;//widthCoef; // right
                bottom = (float) corner[1].y * screenHeight / (float) MAX_FRAME_HEIGHT;//* heightCoef; // bottom
            }
//
            coordinates[0] = left - 15; // left увеличиваем размер маркера на 30 пикселей чтобы цель фиксировалась увереннее
            coordinates[1] = top - 15; // top
            coordinates[2] = right + 15; // right
            coordinates[3] = bottom + 15; // bottom

            int width = (int) Math.abs(right - left);
            int height = (int) Math.abs(bottom - top);
            if (width > 10 && height > 10) { // проверка на случай ошибочного распознавания. артефакты имеют маленький размер
                onCoordinateUpdate(coordinates, screenWidth, screenHeight);
            }else{
                onCoordinateUpdate(null, 0f, 0f); // размеры слишком маленькие. координат нет
            }
        }
    }

    @Override
    public void getBitmapFromStream(Mat mat1) { // сюда приходит изображение дрона, которое мы дальше распознаем
        if (!sourceFromCamera) {
            mDroneMat = mat1;
            mRgba = mDroneMat;
        }
    }

    @Override
    public void onCoordinateUpdate(float[] coordinates, float width, float height) { // сюда приходят обновленные координаты маркера
        int l = 0;
        int t = 0;
        int r = 0;
        int b = 0;
        if (coordinates != null) {
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    detection_status_view.setVisibility(View.VISIBLE);
                }
            });
        } else {
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    detection_status_view.setVisibility(View.GONE);
                }
            });
        }
        if (missionIsActive) {
            return;
        }

        if (coordinates != null) {
//            View parent = (View) mSendRectIV.getParent();
//            float widthCoef = parent.getWidth() / (float) MAX_FRAME_WIDTH;
//            float heightCoef = parent.getHeight() / (float) MAX_FRAME_HEIGHT;
            mRectF = new RectF();
            mRectF.left = coordinates[0] / width;
            mRectF.top = coordinates[1] / height;
            mRectF.right = coordinates[2] / width;
            mRectF.bottom = coordinates[3] / height;
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    detection_status_view.setVisibility(View.VISIBLE);
                    coordinates_info.setText(mRectF.toString());
                }
            });

            isDrawingRect = false;

            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    mSendRectIV.setVisibility(View.VISIBLE);
                }
            });

            if (!missionIsActive) {
                t = (int) coordinates[1];
                l = (int) coordinates[0];
                r = (int) coordinates[2];
                b = (int) coordinates[3];
                mSendRectIV.setX(l);
                mSendRectIV.setY(t);
                mSendRectIV.getLayoutParams().width = r - l;
                mSendRectIV.getLayoutParams().height = b - t;
                mOpenCVRectangleWidth = r - l ;
                mOpenCVRectangleHeight = b - t;

                Marker marker = null;
                if (mMarkersList != null && !mMarkersList.isEmpty()) { // проверяем где находится центр распознанного маркера
                    marker = mMarkersList.get(0);
                }
                if (marker == null){
                    return;
                }
                double pointX = (r + l) / 2; //centerPoint.x;
                double pointY = (t + b) / 2; //centerPoint.y;
                if (MISSION_TYPE.equals(MISSION_TYPE_SEARCH)) { // если активна миссия поиска
                    if (100d / width * pointX > 33d && 100d / width * pointX < 66d) { // позволяем передачу координат только если они в допустимых пределах
                        missionIsActive = true;
                        activateMission(); // запускаем на дроне встроенную функцию следования за объектом
                    }
                }else{ // если маркер распознался сразу, без миссии поиска
                    missionIsActive = true;
                    activateMission(); // запускаем на дроне встроенную функцию следования за объектом
                }
            }
        } else {
            missionIsActive = false;
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    detection_status_view.setVisibility(View.GONE);
                }
            });
        }

        int finalL = l; // это данные только для отображения. больше нигде не используются.
        int finalT = t;
        int finalR = r;
        int finalB = b;
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mSendRectIV.setVisibility(View.VISIBLE);
                mSendRectIV.requestLayout();
                selected_rectangle_info.setText(" l = " + finalL + " t = " + finalT + " r = " + finalR + " b = " + finalB);
            }
        });
    }


    @Override
    public void onYuvDataReceived(ByteBuffer byteBuffer, int i, int i1, int i2) {

    }

//**************************************************** DRONE SEGMENT ***********************************************************

    private static final int MAIN_CAMERA_INDEX = 0;
    private static final int INVAVID_INDEX = -1;
    private static final int MOVE_OFFSET = 20;

    private Switch mAutoSensingSw;
    private Switch mQuickShotSw;
    private ImageButton mPushDrawerIb;
    private SlidingDrawer mPushInfoSd;
    private ImageButton mStopBtn;
    private ImageView mTrackingImage;
    private RelativeLayout mBgLayout;
    private TextView mPushInfoTv;
    private Switch mPushBackSw;
    private Switch mGestureModeSw;
    private ImageView mSendRectIV;
    private Button mConfigBtn;
    private Button mConfirmBtn;
    private Button mRejectBtn;

    private Button btnTakeOff;
    private Button confirmLand;
    private Button autoLand;
    private Button btn_land;
    //
    private ActiveTrackOperator mActiveTrackOperator;
    private ActiveTrackMission mActiveTrackMission;
    private int trackingIndex = INVAVID_INDEX;
    private boolean isDrawingRect = false;

    /**
     * Toast
     *
     * @param string
     */
    private void setResultToToast(final String string) {
        TrackingActivity.this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(TrackingActivity.this, string, Toast.LENGTH_SHORT).show();
            }
        });
    }

    /**
     * Push Status to TextView
     *
     * @param string
     */
    private void setResultToText(final String string) {
        if (mPushInfoTv == null) {
            setResultToToast("Push info tv has not be init...");
        }
        TrackingActivity.this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mPushInfoTv.setText(string);
            }
        });
    }

    /**
     * InitUI
     */
    private void initUI() {
        mPushDrawerIb = (ImageButton) findViewById(R.id.tracking_drawer_control_ib);
        mPushInfoSd = (SlidingDrawer) findViewById(R.id.tracking_drawer_sd);
        mPushInfoTv = (TextView) findViewById(R.id.tracking_push_tv);
        mBgLayout = (RelativeLayout) findViewById(R.id.tracking_bg_layout);
        mSendRectIV = (ImageView) findViewById(R.id.tracking_send_rect_iv);
        mTrackingImage = (ImageView) findViewById(R.id.tracking_rst_rect_iv);
        mConfirmBtn = (Button) findViewById(R.id.confirm_btn);
        mStopBtn = (ImageButton) findViewById(R.id.tracking_stop_btn);
        mRejectBtn = (Button) findViewById(R.id.reject_btn);
        mConfigBtn = (Button) findViewById(R.id.recommended_configuration_btn);
        mAutoSensingSw = (Switch) findViewById(R.id.set_multitracking_enabled);
        mQuickShotSw = (Switch) findViewById(R.id.set_multiquickshot_enabled);
        mPushBackSw = (Switch) findViewById(R.id.tracking_pull_back_tb);
        mGestureModeSw = (Switch) findViewById(R.id.tracking_in_gesture_mode);

        btnTakeOff = (Button) findViewById(R.id.btn_take_off);
        confirmLand = (Button) findViewById(R.id.btn_confirm_land);
        autoLand = (Button) findViewById(R.id.btn_auto_land);
        btn_land = (Button) findViewById(R.id.btn_land);
        autoLand.setOnClickListener(this);
        confirmLand.setOnClickListener(this);
        btnTakeOff.setOnClickListener(this);
        btn_land.setOnClickListener(this);

        mAutoSensingSw.setChecked(false);
        mGestureModeSw.setChecked(false);
        mQuickShotSw.setChecked(false);
        mPushBackSw.setChecked(false);

        mConfirmBtn.setOnClickListener(this);
        mStopBtn.setOnClickListener(this);
        mRejectBtn.setOnClickListener(this);
        mConfigBtn.setOnClickListener(this);
        mPushDrawerIb.setOnClickListener(this);
    }

    /**
     * Init Mission parameter
     */
    private void initMissionManager() {
        mActiveTrackOperator = MissionControl.getInstance().getActiveTrackOperator();
        if (mActiveTrackOperator == null) {
            return;
        }

        mActiveTrackOperator.addListener(this);
        if (mAutoSensingSw == null || mActiveTrackOperator == null) {
            return;
        }
    }

    /**
     * @Description : RETURN BTN RESPONSE FUNCTION
     */
    @Override
    public void onReturn(View view) {
        DJILog.d(TAG, "onReturn");
        DJISDKManager.getInstance().getMissionControl().destroy();
        this.finish();
    }


    private void activateMission() {
        String activationStepName = "activate_mission";
        mActiveTrackMission = new ActiveTrackMission(mRectF, ActiveTrackMode.TRACE);
//        mActiveTrackOperator.canStartTracking()
        if (mActiveTrackOperator != null) {
            mActiveTrackOperator.startTracking(mActiveTrackMission, new CompletionCallback() {
                @Override
                public void onResult(DJIError error) {
                    analiseResult(activationStepName, error, mErrorInformation, mErrorReason, mTrackingRectangleWidth, mTrackingRectangleHeight);

                    if (error == null) { // здесь только отображаем визуальные данные
                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                tracking_status_info.setText("Start Tracking: success ");
                            }
                        });
                    } else {
                        missionIsActive = false;
                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                tracking_status_info.setText("Start Tracking: error " + error.getDescription());
                            }
                        });
                    }
                }
            });
        }
    }

    private void analiseResult(String stepName, DJIError error, String information, String reason, float cameraRectangleWidth, float cameraRectangleHeight){
        if (cameraRectangleWidth == 0f || cameraRectangleHeight == 0f){
            missionIsActive = false;
            repeatRecognition(); // повторяем процесс распознавания метки и передачи координат дрону
            return;
        }
        switch (stepName){
            case "activate_mission":
                if (error == null){
                    cancelMissionDataUpdateAndFlightTimers();
                    mMissionCoordinateList = null;
                    destroyVirtualStickDataTimer();
                    resetController();
                    markerIsDetected = true;
                    mHandler.removeCallbacksAndMessages(null);
                    confirmTarget(); // подтверждаем цель
                    return;
                }else{
                    missionIsActive = false;
                    // ничего не делаем. ждем.
                    return;
                }
            case "confirm_target":
                if (error == null){
                    targetConfirmationIsLocked = true; // если дрон принял метку оставляем флаг активным
                    stopMarkerSearch();
                    return;         // дрон успешно зафиксировал цель. больше делать нечего.
                }else{
                    targetConfirmationIsLocked = false;
                    // подумать что делать дальше в зависимости от полученной ошибки
                }
                break;
        }

        float widthCoef = cameraRectangleWidth * (float) MAX_FRAME_WIDTH / (float) mOpenCVRectangleWidth;
        float heightCoef = cameraRectangleHeight * (float) MAX_FRAME_HEIGHT / (float) mOpenCVRectangleHeight;
        if (widthCoef > 1.2f){ // цель определена но неправильно. слишком широкий прямоугольник
            missionIsActive = false;
            repeatRecognition(); // повторяем процесс распознавания метки и передачи координат дрону
            return;
        }
        if (heightCoef > 1.2f){ // цель определена но неправильно. слишком высокий прямоугольник
            missionIsActive = false;
            repeatRecognition();  // повторяем процесс распознавания метки и передачи координат дрону
            return;
        }
    }

    private void repeatRecognition(){
        if (mCanRepeatRecognition) {
            mCanRepeatRecognition = false;
            new Handler(getMainLooper()).postDelayed(new Runnable() {
                @Override
                public void run() {
                    stopTracking(); // повторяем процесс распознавания метки и передачи координат дрону
                }
            }, 3000); // с задержкой (время нужно для (нацеливания) разворота камеры на метку, иначе метка плохо центрируется на центре маркера)
        }
    }

    private void confirmTarget() {
        String activationStepName = "confirm_target";

        if (targetConfirmationIsLocked){
            return;
        }
        confirmationAttemptCounter += 1;
        targetConfirmationIsLocked = true;
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                confirm_target_info.setText("confirming target");
            }
        });
        mActiveTrackOperator.acceptConfirmation(new CompletionCallback() {
            @Override
            public void onResult(DJIError error) {
                analiseResult(activationStepName, error, mErrorInformation, mErrorReason, mTrackingRectangleWidth, mTrackingRectangleHeight);

                if (error != null) { // здесь только отображаем визуальные данные
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            mission_status_view.setColorFilter(Color.parseColor("#ff0000"));
//                            Toast.makeText(TrackingActivity.this, "confirm target error", Toast.LENGTH_SHORT).show();
                            confirm_target_info.setText("confirm target error " + confirmationAttemptCounter + " " + error.getDescription());
                        }
                    });
                } else {
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            mission_status_view.setColorFilter(Color.parseColor("#00ff00"));
                            Toast.makeText(TrackingActivity.this, "confirm target success", Toast.LENGTH_SHORT).show();
                            confirm_target_info.setText("confirm target success " + confirmationAttemptCounter);
                        }
                    });
                }
            }
        });
    }

    @Override
    public void onClick(View v) {

        switch (v.getId()) {
            case R.id.btn_source:
                sourceFromCamera = !sourceFromCamera;
                java_camera_view_layout.setVisibility(sourceFromCamera ? View.VISIBLE : View.GONE);
                video_previewer_surface.setVisibility(!sourceFromCamera ? View.VISIBLE : View.GONE);
                return;
            case R.id.btn_a:
                dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_ARUCO_ORIGINAL);
                mMarkerID = -1;
                return;
            case R.id.btn_4:
                dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_4X4_1000);
                mMarkerID = -1;
                return;
            case R.id.btn_5:
                dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_1000);
                mMarkerID = -1;
                return;
            case R.id.btn_6:
                dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_6X6_1000);
                mMarkerID = -1;
                return;
            case R.id.btn_7:
                dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_7X7_1000);
                mMarkerID = -1;
                return;
        }

        if (DJISampleApplication.getAircraftInstance() == null){
            return;
        }
        mFlightController = DJISampleApplication.getAircraftInstance().getFlightController();
        if (mFlightController == null) {
            return;
        }

        mFlightController.setFlightOrientationMode(FlightOrientationMode.AIRCRAFT_HEADING, null);
        mFlightController.setMaxFlightHeight(MAXIMUM_PERMITTED_ALTITUDE, null);

        switch (v.getId()) {
            case R.id.btn_take_off: // кнопка взлета
                autoLendingActivated = false;
                recognitionIsLocked = false;
                targetConfirmationIsLocked = false;
                missionIsActive = false;
                mCanRepeatRecognition = true;
                trackingIndex = INVAVID_INDEX;
                markerIsDetected = false;
                counterOfSearchCircles = 0;
                imageIsLocked = true;
                center_X = 0f;
                center_Y = 0f;
                mCameraPitch = -90;
                MISSION_TYPE = "MISSION_NOT_ACTIVE";
                center_x_color = 0;
                center_y_color = 0;
                pitch_color = 0;
                mThrottle = DRONE_MISSION_HEIGHT;
                lendingHeight = DRONE_MISSION_HEIGHT;
                mHandler.removeCallbacksAndMessages(null);
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        mission_status_view.setColorFilter(Color.parseColor("#0000ff"));
                        gimbal_state_text_view.setTextColor(Color.parseColor("#ff0000"));
                        pitch_text_view.setTextColor(Color.parseColor("#ff0000"));
                        center_x_text_view.setTextColor(Color.parseColor("#ff0000"));
                        center_y_text_view.setTextColor(Color.parseColor("#ff0000"));
                        confirmLand.setTextColor(Color.parseColor("#ffffff"));
                        detection_status_view.setVisibility(View.GONE);
                        pitch_text_view.requestLayout();

                    }
                });
                Log.d("take_off_log", "take off click");
                cancelMissionDataUpdateAndFlightTimers();
                destroyVirtualStickDataTimer();
                startTakeOff();
                return;
            case R.id.btn_auto_land: // кнопка автоматической посадки
                autoLendingActivated = false;
                MISSION_TYPE = MISSION_TYPE_LEND;
                mMissionIndex = -1;
                cancelMissionDataUpdateAndFlightTimers();
                activateStickModeForLending();
                lendingStepIndex = 0;
                updateLandingTask();
                activateSendVirtualStickDataTimer();
                return;
            case R.id.btn_confirm_land:
                mFlightController.confirmLanding(new CompletionCallback() {
                    @Override
                    public void onResult(DJIError djiError) {
//                        DialogUtils.showDialogBasedOnError(TrackingActivity.this, djiError);
                    }
                });
                return;
            case R.id.btn_land:
                mFlightController.startLanding(new CompletionCallback() {
                    @Override
                    public void onResult(DJIError djiError) {
//                        DialogUtils.showDialogBasedOnError(TrackingActivity.this, djiError);
                    }
                });
                return;
            default:
                break;
        }

        if (mActiveTrackOperator == null) {
            return;
        }
        switch (v.getId()) {
            case R.id.tracking_stop_btn: // кнопка прекращения слежения
                stopTracking();
                break;
//            case R.id.reject_btn: // кнопка отмены объекта
//                trackingIndex = INVAVID_INDEX;
////                mActiveTrackOperator.canStartTracking()
//                mActiveTrackOperator.rejectConfirmation(new CompletionCallback() {
//
//                    @Override
//                    public void onResult(DJIError error) {
//                        if (error == null) {
//                            missionIsActive = false;
//                        }
//                        setResultToToast(error == null ? "Reject Confirm Success!" : error.getDescription());
//                    }
//                });
//                runOnUiThread(new Runnable() {
//                    @Override
//                    public void run() {
//                        mStopBtn.setVisibility(View.VISIBLE);
//                        mRejectBtn.setVisibility(View.VISIBLE);
////                        mConfirmBtn.setVisibility(View.INVISIBLE);
//                    }
//                });
//                break;
//            case R.id.tracking_drawer_control_ib:
//                if (mPushInfoSd.isOpened()) {
//                    mPushInfoSd.animateClose();
//                } else {
//                    mPushInfoSd.animateOpen();
//                }
//                break;

            default:
                break;
        }
    }

    private void startTakeOff(){
        takeOffSuccess[0] = false; // флаг успешного взлета
        mFlightController.startTakeoff(new CompletionCallback() {
            @Override
            public void onResult(DJIError djiError) {
                if (djiError == null){
                    new Handler().postDelayed(new Runnable() {
                        @Override
                        public void run() {
                            Log.d("take_off_log", "take off success");
                            rotateGimbalToAbsoluteAngle(-90); // устанавливаем камеру под углом 90 градусов через 2 сек после взлета иначе нет класса Aircraft  и мы не можем получить gimbal
                            setGimbalAndFlightControllerCallbacks();
                            takeOffSuccess[0] = true;
                        }
                    }, 2000); // время задержки
                }else{
                    Log.d("take_off_log", "take off error");
                }
                setResultToToast("Take off " + (djiError == null ?  " Success" : djiError.getDescription()));
            }
        });
        new Handler().postDelayed(new Runnable() {
            @Override
            public void run() {
                if (takeOffSuccess[0]) {
                    activateStickModeForSearch(); // активируем режим автономного полета
                }
            }
        }, 6000); // время задержки перед активацией. если время меньше 7000, то режим StickModeForSearch не включится
    }

    private void startSearchMission(){
        DRONE_FLIGHT_TIME = 300; // время в течении которого команда активна. сейчас второй таймер нигде не используется и работает вхолостую
        MISSION_TYPE = MISSION_TYPE_SEARCH; // устанавливаем тип миссии
        createRotationSearchCoordinates(); // создаем маршрут поиска
        mMissionIndex = -1; // обнуляем индекс координаты поиска
        cancelMissionDataUpdateAndFlightTimers(); // активируем необходимые таймеры
        activateSendVirtualStickDataTimer();
        setMissionDataUpdateAndFlightTimers();
    }

    private void activateStickModeForSearch(){
        mMissionIndex = -1;
        cancelMissionDataUpdateAndFlightTimers();
        activateSendVirtualStickDataTimer();
        mFlightController.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
        mFlightController.setYawControlMode(YawControlMode.ANGLE);
        mFlightController.setVerticalControlMode(VerticalControlMode.POSITION);
        mFlightController.setRollPitchCoordinateSystem(FlightCoordinateSystem.BODY);
        mFlightController.setVirtualStickModeEnabled(true, null);
        mFlightController.setVirtualStickAdvancedModeEnabled(true);
        FlightAssistant flightAssistant = mFlightController.getFlightAssistant();
        if (flightAssistant != null) {
            flightAssistant.setVisionAssistedPositioningEnabled(true, null);
            flightAssistant.setActiveObstacleAvoidanceEnabled(true, null);
            flightAssistant.setCollisionAvoidanceEnabled(false, null);
            flightAssistant.setVisionAssistedPositioningEnabled(true, null);
        }else{
            setResultToToast("flightAssistant = null");
        }
        mHandler.postDelayed(new Runnable() { // запускаем таймер на случай если метка не будет найдена
            @Override
            public void run() {
                activateSearchMission();
            }
        }, 25000);
    }

    private void activateStickModeForLending(){
        mMissionIndex = -1;
        cancelMissionDataUpdateAndFlightTimers();
        activateSendVirtualStickDataTimer();
        mFlightController.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
        mFlightController.setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
        mFlightController.setVerticalControlMode(VerticalControlMode.POSITION);
        mFlightController.setRollPitchCoordinateSystem(FlightCoordinateSystem.BODY);
        mFlightController.setVirtualStickModeEnabled(true, null);
        mFlightController.setVirtualStickAdvancedModeEnabled(true);
        FlightAssistant flightAssistant = mFlightController.getFlightAssistant();
        if (flightAssistant != null) {
            flightAssistant.setVisionAssistedPositioningEnabled(true, null);
            flightAssistant.setActiveObstacleAvoidanceEnabled(true, null);
            flightAssistant.setCollisionAvoidanceEnabled(false, null);
            flightAssistant.setVisionAssistedPositioningEnabled(true, null);
        }else{
            setResultToToast("flightAssistant = null");
        }
    }

    private void activateSearchMission(){
        if (markerIsDetected){
            return;
        }
        startSearchMission();
    }

    private void cancelMissionDataUpdateAndFlightTimers(){
        if (mMissionDataUpdateTimer != null){
            mMissionDataUpdateTimer.cancel();
            mMissionDataUpdateTimer = null;
        }
        if (mFlightTimer != null){
            mFlightTimer.cancel();
            mFlightTimer = null;
        }
    }

    private void activateSendVirtualStickDataTimer(){
        if (null == mSendVirtualStickDataTimer) {
            mSendVirtualStickDataTask = new SendVirtualStickDataTask();
            mSendVirtualStickDataTimer = new Timer();
            mSendVirtualStickDataTimer.schedule(mSendVirtualStickDataTask, 100, 100);
        }
    }

    private void activateScreenUpdateTimer(){
        if (screenUpdateTimer == null){
            screenUpdateTimer = new Timer();
            mUpdateScreenTask = new UpdateScreenTask();
            screenUpdateTimer.schedule(mUpdateScreenTask, 20, 250);
        }
    }

    private void setMissionDataUpdateAndFlightTimers() {
        mFlightTimer = createFlightTimer();
        mMissionDataUpdateTimer = createMissionDataUpdateTimer();
        mMissionDataUpdateTimer.start();
    }

    private void stopTracking(){
        MISSION_TYPE = "MISSION_NOT_ACTIVE";
        resetTrackingData();
        resetTrackingIndicator();
        mActiveTrackOperator.stopTracking(new CompletionCallback() {
            @Override
            public void onResult(DJIError error) {
                setResultToToast(error == null ? "Stop Tracking Success!" : error.getDescription());
            }
        });
    }

    private void resetTrackingData(){
        trackingIndex = INVAVID_INDEX;
        recognitionIsLocked = false;
        targetConfirmationIsLocked = false;
        missionIsActive = false;
        mCanRepeatRecognition = true;
    }

    private void resetTrackingIndicator(){
        TrackingActivity.this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mission_status_view.setColorFilter(Color.parseColor("#0000ff"));
            }
        });
    }

    private void resetController(){
        mMissionIndex = -1;
        mUseCoordinate = false;
        cancelMissionDataUpdateAndFlightTimers();
        mFlightController.setRollPitchControlMode(RollPitchControlMode.VELOCITY);
        mFlightController.setYawControlMode(YawControlMode.ANGULAR_VELOCITY);
        mFlightController.setVerticalControlMode(VerticalControlMode.VELOCITY);
        mFlightController.setVirtualStickModeEnabled(false, null);
        mFlightController.setVirtualStickAdvancedModeEnabled(false);
        mFlightController.setMaxFlightRadiusLimitationEnabled(false, null);
        mFlightController.setMaxFlightRadius(1000, null);
        mFlightController.setHomeLocationUsingAircraftCurrentLocation(null);
        mFlightController.setMultipleFlightModeEnabled(false, null);

        FlightAssistant flightAssistant = mFlightController.getFlightAssistant();
        if (flightAssistant != null) {
            flightAssistant.setActiveObstacleAvoidanceEnabled(true, null);
            flightAssistant.setCollisionAvoidanceEnabled(false, null);
            flightAssistant.setVisionAssistedPositioningEnabled(true, null);
        }
    }

    @Override
    public void onUpdate(ActiveTrackMissionEvent event) {
//        String activationStepName = "mission_event";
        StringBuffer sb = new StringBuffer();
        String errorInformation = (event.getError() == null ? "null" : event.getError().getDescription()) + "\n";
        String currentState = event.getCurrentState() == null ? "null" : event.getCurrentState().getName();
        MISSION_STATE = currentState;
        String previousState = event.getPreviousState() == null ? "null" : event.getPreviousState().getName();
//        DJIUtils.addLineToSB(sb, "CurrentState: ", currentState);
        DJIUtils.addLineToSB(sb, "PreviousState: ", previousState);
        DJIUtils.addLineToSB(sb, "Error:", errorInformation);

        mErrorInformation = errorInformation;

        ActiveTrackTrackingState trackingState = event.getTrackingState();
        if (trackingState != null) {
            RectF trackingRect = trackingState.getTargetRect();
            if (trackingRect != null) {
                center_X = trackingRect.centerX();
                center_Y = trackingRect.centerY();
//                DJIUtils.addLineToSB(sb, "Rect center x: ", trackingRect.centerX());
//                DJIUtils.addLineToSB(sb, "Rect center y: ", trackingRect.centerY());
//                DJIUtils.addLineToSB(sb, "Rect Width: ", trackingRect.width());
//                DJIUtils.addLineToSB(sb, "Rect Height: ", trackingRect.height());
                DJIUtils.addLineToSB(sb, "left: ", trackingRect.left);
                DJIUtils.addLineToSB(sb, "top: ", trackingRect.top);
                DJIUtils.addLineToSB(sb, "right: ", trackingRect.right);
                DJIUtils.addLineToSB(sb, "bottom: ", trackingRect.bottom);
                DJIUtils.addLineToSB(sb, "Reason", trackingState.getReason().name());
//                DJIUtils.addLineToSB(sb, "Target Index: ", trackingState.getTargetIndex());
//                DJIUtils.addLineToSB(sb, "Target Type", trackingState.getType().name());
//                DJIUtils.addLineToSB(sb, "Target State", trackingState.getState().name());
//                mErrorReason = trackingState.getReason().name();
                mTrackingRectangleWidth = trackingRect.width();
                mTrackingRectangleHeight = trackingRect.height();
                setResultToText(sb.toString());
            }
        }

        updateActiveTrackRect(mTrackingImage, event);
        String text = sb.toString();
        String text_2 = "";

        ActiveTrackState state = event.getCurrentState();
        if (state == ActiveTrackState.FINDING_TRACKED_TARGET ||
                state == ActiveTrackState.AIRCRAFT_FOLLOWING ||
                state == ActiveTrackState.ONLY_CAMERA_FOLLOWING ||
                state == ActiveTrackState.CANNOT_CONFIRM ||
                state == ActiveTrackState.WAITING_FOR_CONFIRMATION ||
                state == ActiveTrackState.PERFORMING_QUICK_SHOT) {

                if (state == ActiveTrackState.FINDING_TRACKED_TARGET) {
                    text_2 = "FINDING_TRACKED_TARGET";
                } else if (state == ActiveTrackState.AIRCRAFT_FOLLOWING) {
                    text_2 = "AIRCRAFT_FOLLOWING";
                } else if (state == ActiveTrackState.ONLY_CAMERA_FOLLOWING) {
                    text_2 = "ONLY_CAMERA_FOLLOWING";
                }else if (state == ActiveTrackState.CANNOT_CONFIRM){
                    text_2 = "CANNOT_CONFIRM" ;
                }else if (state == ActiveTrackState.WAITING_FOR_CONFIRMATION){
                    text_2 = "WAITING_FOR_CONFIRMATION";
                }else if (state == ActiveTrackState.PERFORMING_QUICK_SHOT){
                    text_2 = "PERFORMING_QUICK_SHOT";
                }else{
                    text_2 = "UNKNOWN_STATE";
                }


            TrackingActivity.this.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    mStopBtn.setVisibility(View.VISIBLE);
                    mStopBtn.setClickable(true);
//                    mConfirmBtn.setVisibility(View.VISIBLE);
//                    mConfirmBtn.setClickable(true);
//                    mRejectBtn.setVisibility(View.VISIBLE);
//                    mRejectBtn.setClickable(true);
//                    mConfigBtn.setVisibility(View.INVISIBLE);
                }
            });
        } else {
            TrackingActivity.this.runOnUiThread(new Runnable() {

                @Override
                public void run() {
//                    mStopBtn.setVisibility(View.INVISIBLE);
//                    mStopBtn.setClickable(false);
//                    mConfirmBtn.setVisibility(View.INVISIBLE);
//                    mConfirmBtn.setClickable(false);
//                    mRejectBtn.setVisibility(View.INVISIBLE);
//                    mRejectBtn.setClickable(false);
////                    mConfigBtn.setVisibility(View.VISIBLE);
//                    mTrackingImage.setVisibility(View.INVISIBLE);
                }
            });
        }
        showStateInfo(text_2 + text);
    }

    private void showStateInfo(String text){
        TrackingActivity.this.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                missionEventStateInfo.setText(text); // статус ActiveTrackMissionEvent
            }
        });
    }

    /**
     * Update ActiveTrack Rect
     *
     * @param iv
     * @param event
     */
    private void updateActiveTrackRect(final ImageView iv, final ActiveTrackMissionEvent event) {
        if (iv == null || event == null) {
            return;
        }
        View parent = (View) iv.getParent();

        ActiveTrackTrackingState trackingState = event.getTrackingState();
        if (trackingState != null) {
            RectF trackingRect = event.getTrackingState().getTargetRect();
            final int l = (int) ((trackingRect.centerX() - trackingRect.width() / 2) * parent.getWidth());
            final int t = (int) ((trackingRect.centerY() - trackingRect.height() / 2) * parent.getHeight());
            final int r = (int) ((trackingRect.centerX() + trackingRect.width() / 2) * parent.getWidth());
            final int b = (int) ((trackingRect.centerY() + trackingRect.height() / 2) * parent.getHeight());

            TrackingActivity.this.runOnUiThread(new Runnable() {
                @Override
                public void run() {

                    ActiveTrackTargetState targetState = event.getTrackingState().getState();

                    if ((targetState == ActiveTrackTargetState.CANNOT_CONFIRM)
                            || (targetState == ActiveTrackTargetState.UNKNOWN)) {
                        iv.setImageResource(R.drawable.visual_track_cannotconfirm);
                    } else if (targetState == ActiveTrackTargetState.WAITING_FOR_CONFIRMATION) {
                        iv.setImageResource(R.drawable.visual_track_needconfirm);
//                        confirmTarget();
                    } else if (targetState == ActiveTrackTargetState.TRACKING_WITH_LOW_CONFIDENCE) {
                        iv.setImageResource(R.drawable.visual_track_lowconfidence);
                    } else if (targetState == ActiveTrackTargetState.TRACKING_WITH_HIGH_CONFIDENCE) {
                        iv.setImageResource(R.drawable.visual_track_highconfidence);
                    }
                    iv.setVisibility(View.VISIBLE);
                    iv.setX(l);
                    iv.setY(t);
                    iv.getLayoutParams().width = r - l;
                    iv.getLayoutParams().height = b - t;
                    iv.requestLayout();
                }
            });
        }
    }


    @Override
    public void onBackPressed() {
        super.onBackPressed();
        finish();
    }


    class UpdateScreenTask extends TimerTask {
        @Override
        public void run() {
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    if (mMissionCoordinateList != null){
                        if (mMissionIndex > -1) {
                            if (!mMissionCoordinateList.isEmpty()){
                                movement_direction.setText("направление " + mMissionIndex + " " + mMissionCoordinateList.get(mMissionIndex).getCourse() + " " + confirmationAttemptCounter);
                            }
                        }else{
                            String course = "";
                            if (!mMissionCoordinateList.isEmpty()){
                                course = mMissionCoordinateList.get(0).getCourse();
                            }
                            movement_direction.setText("направление " + mMissionIndex + " " + MISSION_TYPE + " " + course);
                        }
                    }
                    center_x_text_view.setText(" center_X = " + center_X);
                    center_x_text_view.setTextColor(center_x_color == 0 ? Color.parseColor("#ff0000") : Color.parseColor("#00ff00"));
                    center_y_text_view.setText(" center_Y = " + center_Y);
                    center_y_text_view.setTextColor(center_y_color == 0 ? Color.parseColor("#ff0000") : Color.parseColor("#00ff00"));
                    pitch_text_view.setText(" CameraPitch = " + mCameraPitch);
                    pitch_text_view.setTextColor(pitch_color == 0 ? Color.parseColor("#ff0000") : Color.parseColor("#00ff00"));
                    pitch_text_view.requestLayout();
                }
            });
        }
    }

    class SendVirtualStickDataTask extends TimerTask {

        @Override
        public void run() {
            updateMissionCoordinate();
            if (mFlightController != null) {
                mFlightController.sendVirtualStickFlightControlData(new FlightControlData(mPitch, mRoll, mYaw, mThrottle), null);
            }
        }
    }

    private void updateMissionCoordinate(){
        if (MISSION_TYPE.equals(MISSION_TYPE_SEARCH)) { // миссия поиска
            if (mMissionCoordinateList != null && mMissionIndex < mMissionCoordinateList.size() && mMissionIndex > -1 && mUseCoordinate) {
                mPitch = mMissionCoordinateList.get(mMissionIndex).getPitch();
                mRoll = mMissionCoordinateList.get(mMissionIndex).getRoll();
                mYaw = mMissionCoordinateList.get(mMissionIndex).getYaw();
                mThrottle = mMissionCoordinateList.get(mMissionIndex).getThrottle();
            } else {
                mPitch = 0f;
                mRoll = 0f;
                mYaw = 0f;
                mThrottle = DRONE_MISSION_HEIGHT;
            }
        }else if (MISSION_TYPE.equals(MISSION_TYPE_LEND)) { // миссия посадка
            if (targetIsLost()){
                destroyVirtualStickDataTimer(); // останавливаем полет если цель потеряна во время посадки
                resetController();
                return;
            }
            updateLandingTask();
            if (mMissionCoordinateList != null) {
                mPitch = mMissionCoordinateList.get(0).getPitch(); // берем всегда 0 элемент т.к. список всегда имеет длину 1
                mRoll = mMissionCoordinateList.get(0).getRoll();
                mYaw = mMissionCoordinateList.get(0).getYaw();
                mThrottle = mMissionCoordinateList.get(0).getThrottle();
            }else{
                mPitch = 0f; // останавливаем полет
                mRoll = 0f;
                mYaw = 0f;
                mThrottle = lendingHeight;
            }
        }
    }

    private boolean targetIsLost(){
        return "FINDING_TRACKED_TARGET".equals(MISSION_STATE);
    }

    private void stopMarkerSearch(){
        mHandler.removeCallbacksAndMessages(null);
        mMissionIndex = -1;
        mUseCoordinate = false;  // остановить полет
        cancelMissionDataUpdateAndFlightTimers();
        destroyVirtualStickDataTimer();
        resetController();
    }

    private boolean rotateGimbalToAbsoluteAngle(float angle){ // значения менются в отрицательную сторону "0" = прямо "-90" = вниз.
        mAircraft = getAircraft();
        Rotation.Builder builder =  new Rotation.Builder()
                .pitch((float) (Math.abs(angle) * - 1))
                .yaw(Rotation.NO_ROTATION)
                .roll(Rotation.NO_ROTATION)
                .time(0.5)
                .mode(RotationMode.ABSOLUTE_ANGLE);//RELATIVE_ANGLE

        Rotation rotation = builder.build();

        if (mAircraft == null || rotation == null){
            return false;
        }
        Gimbal gimbal = mAircraft.getGimbal();
        gimbal.rotate(rotation, null);
        return true;
    }

    private void setGimbalAndFlightControllerCallbacks(){
        mAircraft = getAircraft();
        if (mAircraft == null){
            return;
        }

        Gimbal gimbal = mAircraft.getGimbal();
        if (gimbal != null) {
            gimbal.setStateCallback(new GimbalState.Callback() {
                @Override
                public void onUpdate(@NonNull GimbalState gimbalState) {
                    mCameraPitch = gimbalState.getAttitudeInDegrees().getPitch();
                }
            });
        }

        FlightController flightController = mAircraft.getFlightController();
        if (flightController != null) {
            flightController.setStateCallback(new FlightControllerState.Callback() {
                @Override
                public void onUpdate(@NonNull FlightControllerState flightControllerState) {
                    mUltrasonicHeight = flightControllerState.getUltrasonicHeightInMeters();
                    mGPSHeight = flightControllerState.getAircraftLocation().getAltitude();
                    if (mGPSHeight >= 2f){ // когда высота стала больше 2 метров, включаем распознавание
                        imageIsLocked = false;
                    }
                    TrackingActivity.this.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            String ultrasonicHeight = "ultrasonic alt. = " + String.valueOf(flightControllerState.getUltrasonicHeightInMeters() + "\n");
                            String altitudeHeight = "GPS alt. = " + String.valueOf(flightControllerState.getAircraftLocation().getAltitude());
// ************************                           flightControllerState.
                            controller_state_text_view.setText(ultrasonicHeight + altitudeHeight);
                        }
                    });
                }
            });
        }
    }


    private void updateLandingTask(){
        switch (lendingStepIndex){
            case 0: // подлететь к цели
                mMissionCoordinateList = new ArrayList<>();
                MissionCoordinate missionCoordinate = new MissionCoordinate();
                createMoveDroneForwardTask(missionCoordinate);
                createMoveDroneSidewaysTask();
                createMoveDroneBackTask();
                break;
            case 1: // проверить позицию на соответствие нужным параметрам
                if (validateLendingCoordinates()){ // если проверка успешная
                    lendingStepIndex = 0;
                    TrackingActivity.this.runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            confirmLand.setTextColor(Color.parseColor("#00ff00"));
                        }
                    });
                    if (!autoLendingActivated) { // остановить процедуру перемещения и запустить приземление
                        autoLendingActivated = true;
                        destroyVirtualStickDataTimer(); // выключить SendVirtualStickDataTask
                        mMissionCoordinateList = null;
//                        resetController(); // вернуть контроллер в начальное состояние
                        mFlightController.startLanding(null);
                    }
                }else{
                    lendingStepIndex = 0; // рассчитать координаты заново
                }
                break;
        }
    }

    private boolean validateLendingCoordinates(){
        if (Math.abs(mCameraPitch) < 82f){
            center_x_color = 0;
            center_y_color = 0;
            pitch_color = 0;
            return false;
        }else{
            pitch_color = 1;
        }

        if (center_X >= 0.48f && center_X <= 0.52f){
            center_x_color = 1;
        }else{
            center_x_color = 0;
            return false;
        }
        if (center_Y >= 0.47f && center_Y <= 0.65f){
            center_y_color = 1;
        }else{
            center_y_color = 0;
            return false;
        }

        if (mGPSHeight - 0.15f > ROBOT_PRELANDING_HEIGHT){ // пока не снизились до нужной высоты автоматическое приземление не запускается.
            return false;
        }

        return true;
    }

    private void createMoveDroneForwardTask(MissionCoordinate missionCoordinate){
        missionCoordinate.setYaw(0f);
        missionCoordinate.setPitch(0f);
        missionCoordinate.setThrottle(lendingHeight);
        missionCoordinate.setCourse("FORWARD");
        if (Math.abs(mCameraPitch) < 50){
            missionCoordinate.setRoll(mFlightSpeed); // вперед
        }else if(Math.abs(mCameraPitch) >= 50 && Math.abs(mCameraPitch) < 55){ //скорость движения вперед на разных дистанциях подлета
            missionCoordinate.setRoll(0.3f);
        }else if(Math.abs(mCameraPitch) >= 55 && Math.abs(mCameraPitch) < 60){
            missionCoordinate.setRoll(0.25f);
        }else if(Math.abs(mCameraPitch) >= 60 && Math.abs(mCameraPitch) < 65){
            missionCoordinate.setRoll(0.2f);
        }else if(Math.abs(mCameraPitch) >= 65 && Math.abs(mCameraPitch) < 70){
            missionCoordinate.setRoll(0.15f);
        }else if(Math.abs(mCameraPitch) >= 70 && Math.abs(mCameraPitch) < 75){
            missionCoordinate.setRoll(0.15f);
        }else if(Math.abs(mCameraPitch) >= 75 && Math.abs(mCameraPitch) < 82){
            missionCoordinate.setRoll(0.07f);
        }else if(Math.abs(mCameraPitch) >= 82/* && Math.abs(mCameraPitch) < 90*/){
            missionCoordinate.setRoll(0f); // останавливаем движние вперед
            lendingHeight = ROBOT_PRELANDING_HEIGHT; // снижаем дрон до подготовительной к посадке высоты
            missionCoordinate.setThrottle(lendingHeight);
            missionCoordinate.setCourse("STOP");
            lendingStepIndex = 1; // мы в нужной позиции. проверяем валидность координаты.
        }
        mMissionCoordinateList.add(missionCoordinate);
    }

    private void createMoveDroneSidewaysTask(){
        MissionCoordinate missionCoordinate = mMissionCoordinateList.get(0);
        float pitch = 0f;
        if (center_X < 0.42f){
            pitch = -0.08f;
        }else if (center_X >= 0.42f && center_X < 0.46f){ // скорость и направление движения в стороны в зависимости от смещения в сторону от центра
            pitch = -0.06f;
        }else if (center_X >= 0.46f && center_X < 0.48f){
            pitch = -0.04f;
        }else if (center_X >= 0.48f && center_X < 0.49f){
            pitch = -0.02f;
        }else if (center_X >= 0.49f && center_X < 0.51f){
            pitch = -0.01f;
        }else if (center_X >= 0.51f && center_X < 0.49f){
            pitch = 0f;
        }else if(center_X > 0.51f && center_X <= 0.52f){
            pitch = 0.01f;
        }else if(center_X > 0.52f && center_X <= 0.54f){
            pitch = 0.02f;
        }else if(center_X > 0.54f && center_X <= 0.56f){
            pitch = 0.04f;
        }else if(center_X > 0.56f && center_X <= 0.58f){
            pitch = 0.06f;
        }else if(center_X > 0.58f){
            pitch = 0.08f;
        }

        missionCoordinate.setPitch(pitch);
    }

    private void createMoveDroneBackTask(){ // если пролетели метку, даем задний ход
        if (Math.abs(mCameraPitch) < 88){
            return;
        }
        MissionCoordinate missionCoordinate = mMissionCoordinateList.get(0);
        float roll = 0f;
        if (center_Y > 0.7f) {
            roll = -0.15f;
        }else if(center_Y <= 0.7f && center_Y > 0.6f){
            roll = -0.1f;
        }else if(center_Y <= 0.6f && center_Y < 0.51f){
            roll = -0.06f;
        }else if(center_Y <= 0.51f && center_Y >= 0.49f){
            roll = 0f;
        }

        missionCoordinate.setRoll(roll);
    }

    private void createRotationSearchCoordinates(){ // массив с координатами поворота
        mMissionCoordinateList = null;
        mMissionCoordinateList = new ArrayList<>();
        float height = DRONE_MISSION_HEIGHT;
        raiseDroneToHeight(height, 0f);
        rotateDroneToAngle(10f, height);
        rotateDroneToAngle(20f, height);
        rotateDroneToAngle(30f, height);
        rotateDroneToAngle(40f, height);
        rotateDroneToAngle(50f, height);
        rotateDroneToAngle(60f, height);
        rotateDroneToAngle(70f, height);
        rotateDroneToAngle(80f, height);
        rotateDroneToAngle(90f, height);
        rotateDroneToAngle(100f, height);
        rotateDroneToAngle(110f, height);
        rotateDroneToAngle(120f, height);
        rotateDroneToAngle(130f, height);
        rotateDroneToAngle(140f, height);
        rotateDroneToAngle(150f, height);
        rotateDroneToAngle(160f, height);
        rotateDroneToAngle(170f, height);
        rotateDroneToAngle(180f, height);
        rotateDroneToAngle(-170f, height);
        rotateDroneToAngle(-160f, height);
        rotateDroneToAngle(-150f, height);
        rotateDroneToAngle(-140f, height);
        rotateDroneToAngle(-130f, height);
        rotateDroneToAngle(-120f, height);
        rotateDroneToAngle(-110f, height);
        rotateDroneToAngle(-100f, height);
        rotateDroneToAngle(-90f, height);
        rotateDroneToAngle(-80f, height);
        rotateDroneToAngle(-70f, height);
        rotateDroneToAngle(-60f, height);
        rotateDroneToAngle(-50f, height);
        rotateDroneToAngle(-40f, height);
        rotateDroneToAngle(-30f, height);
        rotateDroneToAngle(-20f, height);
        rotateDroneToAngle(-10f, height);
    }

    private void raiseDroneToHeight(float height, float angle){
        MissionCoordinate missionCoordinate = new MissionCoordinate();
        missionCoordinate.setCourse("ROTATE");
        missionCoordinate.setThrottle(height);
        missionCoordinate.setRoll(0.0f);
        missionCoordinate.setPitch(0.0f);
        missionCoordinate.setYaw(angle);
        mMissionCoordinateList.add(missionCoordinate);
    }

    private void rotateDroneToAngle(float angle, float height){
        MissionCoordinate missionCoordinate = new MissionCoordinate();
        missionCoordinate.setCourse("ROTATE");
        missionCoordinate.setThrottle(height);
        missionCoordinate.setRoll(0.0f);
        missionCoordinate.setPitch(0.0f);
        missionCoordinate.setYaw(angle);
        mMissionCoordinateList.add(missionCoordinate);
    }
}
