package org.firstinspires.ftc.teamcode.Testing;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Camera;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.util.Log;

//import com.qualcomm.ftcrobotcontroller.FtcRobotControllerActivity;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.vuforia.ar.pl.Camera2_Preview;

import java.io.ByteArrayOutputStream;

public class OpModeCamera extends OpMode {

    public Camera camera;
    public CameraPreview preview;

    public int width;
    public int height;
    public YuvImage yuvImage = null;

    volatile private boolean imageReady = false;

    private int looped = 0;
    private String data;
    private int ds = 1; // downsampling parameter

    public Camera2_Preview previewCallback = new Camera2_Preview() {
        public void onPreviewFrame(byte[] data, Camera camera) {
            try {
                Camera parameters = camera.getParameters();
                width = parameters.getPreviewSize().width;
                height = parameters.getPreviewSize().height;
                yuvImage = new YuvImage(data, ImageFormat.NV21, width, height, null);
                imageReady = true;
                looped += 1;
            } catch (Exception e) {

            }
        }
    };

    public void setCameraDownsampling(int downSampling) {
        ds = downSampling;
    }

    public boolean imageReady() {
        return imageReady;
    }

    public boolean isCameraAvailable() {
        int cameraId = -1;
        Camera cam = null;
        int numberOfCameras = Camera.getNumberOfCameras();
        for (int i = 0; i < numberOfCameras; i++) {
            Camera.CameraInfo info = new Camera.CameraInfo();
            Camera.getCameraInfo(i, info);
            if (info.facing == Camera.CameraInfo.CAMERA_FACING_BACK) { // Camera.CameraInfo.CAMERA_FACING_FRONT or BACK
                cameraId = i;
                break;
            }
        }
        try {
            cam = Camera.open(cameraId);
        } catch (Exception e) {
            Log.e("Error", "Camera Not Available!");
            return false;
        }
        if(cam != null) {
            cam.release();
        }
        cam = null;
        return true;
    }

    private void openCamera(int width, int height){

    }

    public int red(int pixel) {
        return (pixel >> 16) & 0xff;
    }

    public int green(int pixel) {
        return (pixel >> 8) & 0xff;
    }

    public int blue(int pixel) {
        return pixel & 0xff;
    }

    public int gray(int pixel) {
        return (red(pixel) + green(pixel) + blue(pixel));
    }

    public int highestColor(int red, int blue) {
        int value;
        if((red + 3) > blue)
        {
            value = 0;
        }
        else if((blue + 3) > red)
        {
            value = 2;
        }
        else
        {
            value = 1;
        }
        return value;
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

    public void startCamera() {

    }

    public void stopCamera() {

    }

    /*
     * Code to run when the op mode is first enabled goes here
     */
    @Override
    public void init() {
        startCamera();
    }


    @Override
    public void loop() {

    }


    @Override
    public void stop() {
        stopCamera();

    }
    public int normalizePixels(int value) {
        value /= 100000;
        return value;
    }
}