package org.firstinspires.ftc.robotcontroller.Testing;

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
import android.hardware.camera2.CameraDevice.StateCallback;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.view.ViewGroup;
import java.io.IOException;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CameraMetadata;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.CaptureResult;
import android.hardware.camera2.TotalCaptureResult;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.vuforia.ar.pl.Camera2_Preview;

import java.io.ByteArrayOutputStream;

/**
 * Created by RoboticsUser on 9/29/2016.
 */
public class OpModeCameraNew extends OpMode {

    public CameraManager camMan;
    public Camera2_Preview camPre;
    public CameraDevice camera;


    private int mState = STATE_PREVIEW;

    private static final int STATE_PREVIEW = 0;

    private static final int STATE_WAITING_LOCK = 1;

    private static final int STATE_WAITING_PRECAPTURE = 2;

    private static final int STATE_WAITING_NON_PRECAPTURE = 3;

    private static final int STATE_PICTURE_TAKEN = 4;

    private static final int MAX_PREVIEW_WIDTH =

    private static final int MAX_PREVIEW_HEIGHT =



    public boolean imageReady = false;
    String cameraFId = getFrontFacingCameraId(camMan);

    public String getFrontFacingCameraId(CameraManager camManI) {
        try {
            String cameraId;
            int cameraOrientation;
            CameraCharacteristics characteristics;
            for (int i = 0; i < camManI.getCameraIdList().length; i++) {
                cameraId = camManI.getCameraIdList()[i];
                characteristics = camManI.getCameraCharacteristics(cameraId);
                cameraOrientation = characteristics.get(CameraCharacteristics.LENS_FACING);
                if (cameraOrientation == CameraCharacteristics.LENS_FACING_FRONT) {
                    return cameraId;
                }

            }
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
        return null;
    }

    public void surfaceCreated(SurfaceHolder holder) {
        // The Surface has been created, acquire the camera and tell it where
        // to draw.
        try {
            if (camera != null) {
                camera.setPreviewDisplay(holder);
            }
        } catch (IOException exception) {
            Log.e("CameraBasic", "IOException caused by setPreviewDisplay()", exception);
        }
    }

    private CameraCaptureSession.CaptureCallback cCallback = new CameraCaptureSession.CaptureCallback(){
        private void process(CaptureResult result) {
            switch (mState) {
                case STATE_PREVIEW: {
                    // We have nothing to do when the camera preview is working normally.
                    break;
                }
                case STATE_WAITING_LOCK: {
                    Integer afState = result.get(CaptureResult.CONTROL_AF_STATE);
                    if (afState == null) {
                        captureStillPicture();
                    } else if (CaptureResult.CONTROL_AF_STATE_FOCUSED_LOCKED == afState ||
                            CaptureResult.CONTROL_AF_STATE_NOT_FOCUSED_LOCKED == afState) {
                        // CONTROL_AE_STATE can be null on some devices
                        Integer aeState = result.get(CaptureResult.CONTROL_AE_STATE);
                        if (aeState == null ||
                                aeState == CaptureResult.CONTROL_AE_STATE_CONVERGED) {
                            mState = STATE_PICTURE_TAKEN;
                            captureStillPicture();
                        } else {
                            runPrecaptureSequence();
                        }
                    }
                    break;
                }
                case STATE_WAITING_PRECAPTURE: {
                    // CONTROL_AE_STATE can be null on some devices
                    Integer aeState = result.get(CaptureResult.CONTROL_AE_STATE);
                    if (aeState == null ||
                            aeState == CaptureResult.CONTROL_AE_STATE_PRECAPTURE ||
                            aeState == CaptureRequest.CONTROL_AE_STATE_FLASH_REQUIRED) {
                        mState = STATE_WAITING_NON_PRECAPTURE;
                    }
                    break;
                }
                case STATE_WAITING_NON_PRECAPTURE: {
                    // CONTROL_AE_STATE can be null on some devices
                    Integer aeState = result.get(CaptureResult.CONTROL_AE_STATE);
                    if (aeState == null || aeState != CaptureResult.CONTROL_AE_STATE_PRECAPTURE) {
                        mState = STATE_PICTURE_TAKEN;
                        captureStillPicture();
                    }
                    break;
                }
            }
        }

    };

    /*public void surfaceDestroyed(SurfaceHolder holder) {
        // Surface will be destroyed when we return, so stop the preview.
        if (camera != null) {
            camera.stopPreview();
        }
    }*/



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
