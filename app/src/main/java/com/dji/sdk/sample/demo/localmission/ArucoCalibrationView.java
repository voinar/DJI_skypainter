package com.dji.sdk.sample.demo.localmission;

import android.app.Activity;
import android.app.ProgressDialog;
import android.app.Service;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.SurfaceTexture;
import android.media.MediaFormat;
import android.view.LayoutInflater;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.TextureView;
import android.view.View;
import android.widget.ImageView;
import android.widget.RelativeLayout;
import android.widget.Toast;

import androidx.annotation.NonNull;

import com.dji.sdk.sample.R;
import com.dji.videostreamdecodingsample.media.DJIVideoStreamDecoder;
import com.dji.sdk.sample.internal.controller.DJISampleApplication;
import com.dji.sdk.sample.internal.utils.ModuleVerificationUtil;
import com.dji.sdk.sample.internal.utils.ToastUtils;
import com.dji.sdk.sample.internal.view.PresentableView;
import com.dji.videostreamdecodingsample.media.NativeHelper;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.nio.ByteBuffer;
import java.util.Arrays;

import ch.ethz.cea.dca.CalibrationResult;
import ch.ethz.cea.dca.CameraCalibrator;
import dji.common.camera.SettingsDefinitions;
import dji.common.camera.SystemState;
import dji.common.error.DJIError;
import dji.common.product.Model;
import dji.common.util.CommonCallbacks;
import dji.sdk.base.BaseProduct;
import dji.sdk.camera.Camera;
import dji.sdk.camera.VideoFeeder;
import dji.sdk.codec.DJICodecManager;
import dji.sdk.media.MediaFile;
import dji.sdk.media.MediaFileInfo;
import dji.thirdparty.afinal.core.AsyncTask;

import static com.google.android.gms.internal.zzahn.runOnUiThread;

public class ArucoCalibrationView extends RelativeLayout
        implements
        View.OnClickListener,
        PresentableView,
        CameraCalibrator.OnAddFrameListener,
        DJICodecManager.YuvDataCallback {

    private CameraCalibrator calibrator;

    private Camera camera;

    private Camera mCamera;

    // Live in-app video feed

    private TextureView videostreamPreviewTtView;
    private SurfaceView videostreamPreviewSf;
    private SurfaceHolder videostreamPreviewSh;
    private ImageView previewImageview;
    private DJICodecManager mCodecManager;

    protected VideoFeeder.VideoDataListener mReceivedVideoDataListener = null;

    private int videoViewWidth;
    private int videoViewHeight;

    private long lastupdate;

    private int count;

    private int surfaceCounter = 0;

    private boolean openCVLoaded = false;

    private Context context;



    private enum DemoType { USE_TEXTURE_VIEW, USE_SURFACE_VIEW, USE_SURFACE_VIEW_DEMO_DECODER}
    private static DemoType demoType = DemoType.USE_TEXTURE_VIEW;

    private VideoFeeder.VideoFeed standardVideoFeeder;

    private SurfaceHolder.Callback surfaceCallback;


    private BaseLoaderCallback loaderCallback;

    private Bitmap bm;

    public ArucoCalibrationView(Context context) {
        super(context);
        init(context);

        loaderCallback = new BaseLoaderCallback(context){
            @Override
            public void onManagerConnected(int status){
                if(status == LoaderCallbackInterface.SUCCESS){
                    openCVLoaded = true;
                }
                else {
                    super.onManagerConnected(status);
                }
            }
        };

    }

    private void init(Context context) {
        LayoutInflater layoutInflater = (LayoutInflater) context.getSystemService(Service.LAYOUT_INFLATER_SERVICE);
        layoutInflater.inflate(R.layout.view_aruco_calibration, this, true);

        initUI();

        initSurfaceOrTextureView(context, this);

        this.context = context;
    }

    private void initUI() {

        findViewById(R.id.btn_aruco_capture).setOnClickListener(this);
        findViewById(R.id.btn_aruco_calibrate).setOnClickListener(this);

        videostreamPreviewTtView = (TextureView) findViewById(R.id.livestream_preview_ttv);

        videostreamPreviewSf = (SurfaceView) findViewById(R.id.livestream_preview_sf);
        previewImageview = findViewById(R.id.preview_imageview);
    }

    @Override
    protected void onAttachedToWindow() {
        super.onAttachedToWindow();
        setUpListeners();

        if(OpenCVLoader.initDebug()) {
            loaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
        else {
            //Toast.makeText(this, getString(R.string.error_native_lib), Toast.LENGTH_LONG).show();
        }

        calibrator = new CameraCalibrator(360, 180);
        calibrator.setOnAddFrameListener(this);

        notifyStatusChange();
    }


    @Override
    protected void onDetachedFromWindow() {
        super.onDetachedFromWindow();

        //mCodecManager.enabledYuvData(false);
        //mCodecManager.setYuvDataCallback(null);

        //rgb.release();
        calibrator.release();

        if (VideoFeeder.getInstance().getPrimaryVideoFeed() != null) {
            VideoFeeder.getInstance().getPrimaryVideoFeed().removeVideoDataListener(mReceivedVideoDataListener);
        }
    }



    private void setUpListeners() {
        // Receive : Camera

        if (ModuleVerificationUtil.isCameraModuleAvailable()) {
            camera = DJISampleApplication.getAircraftInstance().getCamera();
            if (ModuleVerificationUtil.isMatrice300RTK() || ModuleVerificationUtil.isMavicAir2()) {
                camera.setFlatMode(SettingsDefinitions.FlatCameraMode.PHOTO_SINGLE, djiError -> ToastUtils.setResultToToast("setFlatMode to PHOTO_SINGLE"));
            } else {
                camera.setMode(SettingsDefinitions.CameraMode.SHOOT_PHOTO, djiError -> ToastUtils.setResultToToast("setMode to shoot_PHOTO"));
            }

            camera.setNewGeneratedMediaFileInfoCallback(new MediaFile.NewFileInfoCallback() {
                @Override
                public void onNewFileInfo(@NonNull MediaFileInfo mediaFileInfo) {
                    ToastUtils.setResultToToast("New photo generated");
                }
            });
        }

        camera.setSystemStateCallback(new SystemState.Callback() {
            @Override
            public void onUpdate(@NonNull SystemState systemState) {
                if ((!systemState.isShootingSinglePhoto()) && (!systemState.isStoringPhoto())) {
                }
            }
        });
    }


    @Override
    public int getDescription() {
        return R.string.aruco_calibration_view_name;
    }

    @NonNull
    @Override
    public String getHint() {
        return this.getClass().getSimpleName() + ".java";
    }

    @Override
    public void onClick(View v) {
        switch(v.getId()) {
            case R.id.btn_aruco_capture:
                calibrator.addFrame();
                break;

            case R.id.btn_aruco_calibrate:
                if(!calibrator.canCalibrate()){
                    Toast.makeText(context, "Add More Frames", Toast.LENGTH_SHORT).show();
                }

                new AsyncTask<Void,Void,Void>(){
                    private double error=0;
                    private ProgressDialog progress;

                    @Override
                    protected void onPreExecute(){
                        progress = new ProgressDialog(context);
                        progress.setTitle("Calibrating");
                        progress.setMessage("Please Wait");
                        progress.setCancelable(false);
                        progress.setIndeterminate(true);
                        progress.show();
                    }

                    @Override
                    protected Void doInBackground(Void... arg0){
                        error = calibrator.calibrate();
                        return null;
                    }

                    @Override
                    protected void onPostExecute(Void result){
                        progress.dismiss();
                        calibrator.clear();

                        Activity activity = (Activity) context;

                        CalibrationResult.save(
                                activity,
                                calibrator.getCameraMatrix(),
                                calibrator.getDistorsionCoefficients()
                        );

                        String resultMessage = "Calibrated with error : " + error;
                        Toast.makeText(context, resultMessage, Toast.LENGTH_SHORT).show();

                    }
                }.execute();
                break;
        }
    }

    @Override
    public void onAddFrame(boolean added) {
        // Succes or failure feedback if sufficient tags are in frame.
    }


    @Override
    public void onYuvDataReceived(MediaFormat format, final ByteBuffer yuvFrame, int dataSize, final int width, final int height) {
        //DJILog.d(TAG, "onYuvDataReceived " + dataSize);

        System.out.println("YUV Received");
        if (count++ % 30 == 0 && yuvFrame != null && openCVLoaded) {
            final byte[] bytes = new byte[dataSize];
            yuvFrame.get(bytes);
            //DJILog.d(TAG, "onYuvDataReceived2 " + dataSize);
            AsyncTask.execute(new Runnable() {
                @Override
                public void run() {

                    System.out.println("Bytes : " + bytes.length);
                    System.out.println("Size : " + width + " , " + height);
                    System.out.println(format);
                    // two samples here, it may has other color format.
                    int colorFormat = format.getInteger(MediaFormat.KEY_COLOR_FORMAT);
                    //byte[] data = new byte[frame.capacity()];
                    //((ByteBuffer) frame.duplicate().clear()).get(data);

                    final byte[] bytesLuminance = Arrays.copyOfRange(bytes,0,(width * height));

                    Mat matGRAY =  new Mat(width, height, CvType.CV_8UC1);
                    matGRAY.put(0,0,bytesLuminance);

                    //calibrator.render(rgb,matGRAY);

                    //Utils.matToBitmap(rgb,bm);
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            previewImageview.setImageBitmap(bm);
                        }
                    });

                }
            });
        }
    }




    private void initSurfaceOrTextureView(Context context, DJICodecManager.YuvDataCallback yuvCallback){
        //bm = Bitmap.createBitmap(1280,720,Bitmap.Config.ARGB_8888);
        bm = Bitmap.createBitmap(990,495,Bitmap.Config.ARGB_8888);
        switch (demoType) {
            case USE_SURFACE_VIEW:
                initPreviewerSurfaceView(context, yuvCallback);
                break;
            case USE_SURFACE_VIEW_DEMO_DECODER:
                initPreviewerTextureView(context,yuvCallback);
                initPreviewerSurfaceView(context,yuvCallback);
                break;
            case USE_TEXTURE_VIEW:
                initPreviewerTextureView(context, yuvCallback);
                break;
        }
    }

    /**
     * Init a surface view for the DJIVideoStreamDecoder
     */
    private void initPreviewerSurfaceView(Context context, DJICodecManager.YuvDataCallback yuvCallback) {
        videostreamPreviewSh = videostreamPreviewSf.getHolder();
        surfaceCallback = new SurfaceHolder.Callback() {
            @Override
            public void surfaceCreated(SurfaceHolder holder) {
                videoViewWidth = videostreamPreviewSf.getWidth();
                videoViewHeight = videostreamPreviewSf.getHeight();
                System.out.println("Width : " + videoViewWidth + ", Height : " + videoViewHeight);
                switch (demoType) {
                    case USE_SURFACE_VIEW:
                        if (mCodecManager == null) {
                            mCodecManager = new DJICodecManager(context, holder, videoViewWidth,
                                    videoViewHeight);
                        }
                        //Code originally from handleYUVClick
                        //mCodecManager.enabledYuvData(true);
                        //mCodecManager.setYuvDataCallback(yuvCallback);

                        break;
                    case USE_SURFACE_VIEW_DEMO_DECODER:
                        // This demo might not work well on P3C and OSMO.
                        NativeHelper.getInstance().init();
                        DJIVideoStreamDecoder.getInstance().init(context, holder.getSurface());
                        DJIVideoStreamDecoder.getInstance().resume();

                        DJIVideoStreamDecoder.getInstance().changeSurface(null);
                        DJIVideoStreamDecoder.getInstance().setYuvDataListener(yuvCallback);
                        break;
                }

            }

            @Override
            public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
                videoViewWidth = width;
                videoViewHeight = height;
                switch (demoType) {
                    case USE_SURFACE_VIEW:


                        //mCodecManager.onSurfaceSizeChanged(videoViewWidth, videoViewHeight, 0);
                        break;
                    case USE_SURFACE_VIEW_DEMO_DECODER:
                        DJIVideoStreamDecoder.getInstance().changeSurface(holder.getSurface());
                        break;
                }

            }

            @Override
            public void surfaceDestroyed(SurfaceHolder holder) {
                switch (demoType) {
                    case USE_SURFACE_VIEW:
                        if (mCodecManager != null) {
                            mCodecManager.cleanSurface();
                            mCodecManager.destroyCodec();
                            mCodecManager = null;
                        }
                        break;
                    case USE_SURFACE_VIEW_DEMO_DECODER:
                        DJIVideoStreamDecoder.getInstance().stop();
                        NativeHelper.getInstance().release();
                        break;
                }

            }
        };



        videostreamPreviewSh.addCallback(surfaceCallback);
    }

    private void initPreviewerTextureView(Context context, DJICodecManager.YuvDataCallback yuvCallback) {
        videostreamPreviewTtView.setSurfaceTextureListener(new TextureView.SurfaceTextureListener() {
            @Override
            public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
                //Log.d(TAG, "real onSurfaceTextureAvailable");
                videoViewWidth = width;
                videoViewHeight = height;
                //Log.d(TAG, "real onSurfaceTextureAvailable: width " + videoViewWidth + " height " + videoViewHeight);
                if (mCodecManager == null) {
                    mCodecManager = new DJICodecManager(context, surface, width, height);
                    //For M300RTK, you need to actively request an I frame.
                    mCodecManager.resetKeyFrame();
                }
            }

            @Override
            public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
                videoViewWidth = width;
                videoViewHeight = height;
                //Log.d(TAG, "real onSurfaceTextureAvailable2: width " + videoViewWidth + " height " + videoViewHeight);
            }

            @Override
            public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
                if (mCodecManager != null) {
                    mCodecManager.cleanSurface();
                }
                return false;
            }

            @Override
            public void onSurfaceTextureUpdated(SurfaceTexture surface) {
                Bitmap image = videostreamPreviewTtView.getBitmap();
                handleImageData(image);
            }
        });
    }

    private void notifyStatusChange() {

        final BaseProduct product = VideoDecodingApplication.getProductInstance();

        // The callback for receiving the raw H264 video data for camera live view
        mReceivedVideoDataListener = new VideoFeeder.VideoDataListener() {

            @Override
            public void onReceive(byte[] videoBuffer, int size) {
                if (System.currentTimeMillis() - lastupdate > 1000) {
                    lastupdate = System.currentTimeMillis();
                }
                switch (demoType) {
                    case USE_SURFACE_VIEW:
                        if (mCodecManager != null) {
                            mCodecManager.sendDataToDecoder(videoBuffer, size);
                            /*
                            runOnUiThread(new Runnable() {
                                @Override
                                public void run() {
                                    surfaceCounter += 1;
                                    if (surfaceCounter % 30 == 0) {
                                        mCodecManager.enabledYuvData(true);
                                        mCodecManager.setYuvDataCallback(yuvCallback);
                                    }
                                    else {
                                        mCodecManager.enabledYuvData(false);
                                        mCodecManager.setYuvDataCallback(null);
                                    }
                                }
                            });
                            */
                        }

                        break;
                    case USE_SURFACE_VIEW_DEMO_DECODER:
                        /**
                         we use standardVideoFeeder to pass the transcoded video data to DJIVideoStreamDecoder, and then display it
                         * on surfaceView
                         */
                        DJIVideoStreamDecoder.getInstance().parse(videoBuffer, size);
                        break;

                    case USE_TEXTURE_VIEW:
                        if (mCodecManager != null) {
                            mCodecManager.sendDataToDecoder(videoBuffer, size);
                        }
                        break;
                }

            }
        };

        if (null == product || !product.isConnected()) {
            mCamera = null;
        } else {
            if (!product.getModel().equals(Model.UNKNOWN_AIRCRAFT)) {
                mCamera = product.getCamera();
                if (mCamera != null) {
                    if (mCamera.isFlatCameraModeSupported()) {
                        mCamera.setFlatMode(SettingsDefinitions.FlatCameraMode.PHOTO_SINGLE, new CommonCallbacks.CompletionCallback() {
                            @Override
                            public void onResult(DJIError djiError) {
                                if(djiError!=null){
                                }
                            }
                        });
                    } else {
                        mCamera.setMode(SettingsDefinitions.CameraMode.SHOOT_PHOTO, new CommonCallbacks.CompletionCallback() {
                            @Override
                            public void onResult(DJIError djiError) {
                                if (djiError != null) {
                                }
                            }
                        });
                    }
                }

                //When calibration is needed or the fetch key frame is required by SDK, should use the provideTranscodedVideoFeed
                //to receive the transcoded video feed from main camera.
                if (demoType == DemoType.USE_SURFACE_VIEW_DEMO_DECODER && isTranscodedVideoFeedNeeded()) {
                    standardVideoFeeder = VideoFeeder.getInstance().provideTranscodedVideoFeed();
                    standardVideoFeeder.addVideoDataListener(mReceivedVideoDataListener);
                    return;
                }
                if (VideoFeeder.getInstance().getPrimaryVideoFeed() != null) {
                    VideoFeeder.getInstance().getPrimaryVideoFeed().addVideoDataListener(mReceivedVideoDataListener);
                }

            }
        }
    }

    private boolean isTranscodedVideoFeedNeeded() {
        if (VideoFeeder.getInstance() == null) {
            return false;
        }

        return VideoFeeder.getInstance().isFetchKeyFrameNeeded() || VideoFeeder.getInstance()
                .isLensDistortionCalibrationNeeded();
    }

    private void handleImageData(Bitmap image) {
        //rgb = new Mat(720,1280, CvType.CV_8UC3);

        //System.out.println("Image Width : " + image.getWidth());
        //System.out.println("Image Height : " + image.getHeight());

        Mat matRGB = new Mat(image.getHeight(), image.getWidth(), CvType.CV_8UC3);
        Utils.bitmapToMat(image,matRGB);
        Imgproc.cvtColor(matRGB,matRGB,Imgproc.COLOR_RGBA2RGB);
        Mat matGRAY =  new Mat(image.getHeight(), image.getWidth(), CvType.CV_8UC1);
        Imgproc.cvtColor(matRGB,matGRAY,Imgproc.COLOR_RGB2GRAY);
        calibrator.render(matRGB,matGRAY);

        Utils.matToBitmap(matRGB,bm);
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                previewImageview.setImageBitmap(bm);
            }
        });

    }



}
