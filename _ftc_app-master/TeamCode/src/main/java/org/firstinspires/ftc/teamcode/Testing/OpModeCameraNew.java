package org.firstinspires.ftc.teamcode.Testing;

import android.app.Activity;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.CaptureResult;
import android.hardware.camera2.TotalCaptureResult;
import android.media.ImageReader;
import android.os.Bundle;
import android.os.HandlerThread;
import android.support.annotation.NonNull;
import android.util.Size;
import android.view.Surface;
import android.view.SurfaceView;
import android.view.TextureView.SurfaceTextureListener;
import android.view.SurfaceHolder;
import android.view.TextureView;
import android.view.View;
import android.view.ViewGroup;
import android.os.Looper;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.vuforia.ar.pl.Camera2_Preview;

import org.firstinspires.ftc.teamcode.R;

import android.nfc.Tag;
import android.app.Activity;
import android.graphics.ImageFormat;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureFailure;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.TotalCaptureResult;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.Image;
import android.media.ImageReader;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;
import android.util.Size;
import android.util.Log;
import android.view.Surface;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Created by RoboticsUser on 9/29/2016.
 */
public class OpModeCameraNew extends Activity{

    HandlerThread camBackgroundThread;
    Handler camBackgroundHandler;
    Handler camForegroundHandler;
    SurfaceView camSurfaceView;
    ImageReader camCaptureBuffer;
    CameraManager camMan;
    CameraCaptureSession camCaptureSession;

    CameraDevice camera;

    static Size chooseBigEnoughSize(Size[] choices, int width, int height) {
        List<Size> bigEnough = new ArrayList<Size>();
        for (Size option : choices){
            if(option.getWidth() >= width && option.getHeight() >= height) {
                bigEnough.add(option);
            }
        }
        //picks the smallest size
        if(bigEnough.size() > 0){
            return Collections.min(bigEnough, new CompareSizesByArea());
        }
        else{
            return choices[0];
        }
    }

    static class CompareSizesByArea implements Comparator<Size> {
        @Override
        public int compare(Size lhs, Size rhs) {
            // cast to make sure multiplication doesn't overflow
            return Long.signum((long) lhs.getWidth() * lhs.getHeight() -
                    (long) rhs.getWidth() * rhs.getHeight());
        }
    }

    @Override
    protected void onResume() {
        super.onResume();

        camBackgroundThread = new HandlerThread("background");
        camBackgroundThread.start();
        camBackgroundHandler = new Handler(camBackgroundThread.getLooper());
        camForegroundHandler = new Handler(getMainLooper());

        camMan = (CameraManager) getSystemService(Context.CAMERA_SERVICE);

        View layout = getLayoutInflater().inflate(R.layout.mainactivity, null);
        camSurfaceView = (SurfaceView) layout.findViewById(R.id.mainSurfaceView);
        camSurfaceView.getHolder().addCallback(camSurfaceHolderCallback);
        setContentView(camSurfaceView);
    }

    protected void onPause() {
        super.onPause();

        try{
            camSurfaceView.getHolder().setFixedSize(0,0);

            if(camCaptureSession != null){
                camCaptureSession.close();
                camCaptureSession = null;
            }
        } finally {
            if(camera != null) {
                camera.close();
                camera = null;
            }
        }
        camBackgroundThread.quitSafely();
        try{
            camBackgroundThread.join();
        } catch (InterruptedException e) {

        }
        if(camCaptureBuffer != null) camCaptureBuffer.close();
    }

    final SurfaceHolder.Callback  camSurfaceHolderCallback = new SurfaceHolder.Callback() {
        private String camId;
        private boolean camGotSecondCallback;

        @Override
        public void surfaceCreated(SurfaceHolder holder){
            Log.i(TAG, "Surface created");
            camId = null;
            camGotSecondCallback = false;
        }

        @Override
        public void surfaceDestroyed(SurfaceHolder holder){
            Log.i(TAG, "Surface destroyed");
            holder.removeCallback(this);
        }

        @Override
        public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
            if (camId == null){
                try{
                    for (String cameraId : camMan.getCameraIdList()){
                        CameraCharacteristics cameraCharacteristics =
                                camMan.getCameraCharacteristics(cameraId);
                        if(cameraCharacteristics.get(CameraCharacteristics.LENS_FACING) ==
                                CameraCharacteristics.LENS_FACING_BACK){
                            Log.i(TAG, "Found back-facing camera");
                            StreamConfigurationMap info = cameraCharacteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);

                            Size largestSize = Collections.max(
                                    Arrays.asList(info.getOutputSizes(ImageFormat.JPEG)),
                                    new CompareSizesByArea());
                            Log.i(TAG, "CaptureSize:" + largestSize);
                            camCaptureBuffer = ImageReader.newInstance(largestSize.getWidth(),
                                    largestSize.getHeight(), ImageFormat.JPEG, 2);
                            camCaptureBuffer.setOnImageAvailableListener(camImageCaptureListener, camBackgroundHandler);

                            Log.i(TAG, "SurfaceView Size:" + camSurfaceView.getWidth() + 'x' + camSurfaceView.getHeight());
                            Size optimalSize = chooseBigEnoughSize(info.getOutputSizes(SurfaceHolder.class), width, height);

                            Log.i(TAG, "PreviewSize:" + optimalSize);
                            SurfaceHolder surfaceHolder = camSurfaceView.getHolder();
                            surfaceHolder.setFixedSize(optimalSize.getWidth(), optimalSize.getHeight());

                            camId = cameraId;
                            return;
                        }
                    }
                } catch (CameraAccessException e){
                    Log.e(TAG, "Unable to list cameras", e);
                }
                Log.e(TAG, "found no back-facing cameras")
            } else if(camera != null){
                Log.e(TAG, "camera has not been closed previously");
                return;
            }
            try{
                camMan.openCamera(camId, camS);
            }
        }
    }

    public void startCamera(){

    }

    public void stopCamera(){

    }


    public Bitmap convertYuvImageToRgb(YuvImage yuvImage, int width, int height, int downSample) {
        Bitmap rgbImage;
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        yuvImage.compressToJpeg(new Rect(0, 0, width, height), 0, out);
        byte[] imageBytes = out.toByteArray();

        BitmapFactory.Options opt;
        opt = new BitmapFactory.Options();
        opt.inSampleSize = downSample;

        rgbImage = BitmapFactory.decodeByteArray(imageBytes, 0, imageBytes.length, opt);
        return rgbImage;
    }


    @Override
    public void init(){
        startCamera();

    }
    @Override
    public void loop(){

    }
    @Override
    public void stop(){
        stopCamera();
    }
}
