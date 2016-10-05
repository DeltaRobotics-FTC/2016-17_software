package org.firstinspires.ftc.teamcode.Testing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import android.graphics.Bitmap;
import for_camera_opmodes.OpModeCamera;
/**
 * Created by RoboticsUser on 12/29/2015.
 */
@Autonomous(name = "Camera_Testing_Op", group = "SensorTesting")

public class Camera_Testing extends OpModeCamera{
    private int looped = 0;
    private int ds2 = 2;

    public void init() {
        setCameraDownsampling(2);
        super.init();
    }

    public void loop() {
        long startTime = System.currentTimeMillis();

        if (imageReady()) {
            int redValueLeft = -76800;
            int blueValueLeft = -76800;
            int greenValueLeft = -76800;
            int redValueRight = -76800;
            int blueValueRight = -76800;
            int greenValueRight = -76800;

            Bitmap rgbImage;
            rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds2);
            for (int x = 0; x < 240; x++) {
                for (int y = 121; y < 320; y++) {
                    int pixelL = rgbImage.getPixel(x, y);
                    redValueLeft += red(pixelL);
                    blueValueLeft += blue(pixelL);
                    greenValueLeft += green(pixelL);
                }
            }
            for (int a = 0; a < 240; a++) {
                for (int b = 0; b < 160; b++) {
                    int pixelR = rgbImage.getPixel(a,b);
                    redValueRight += red(pixelR);
                    blueValueRight += blue(pixelR);
                    greenValueRight += green(pixelR);
                }
            }
            redValueLeft = normalizePixels(redValueLeft);
            blueValueLeft = normalizePixels(blueValueLeft);
            greenValueLeft = normalizePixels(greenValueLeft);
            redValueRight = normalizePixels(redValueRight);
            blueValueRight = normalizePixels(blueValueRight);
            greenValueRight = normalizePixels(greenValueRight);
            int colorLeft = highestColor(redValueLeft, greenValueLeft, blueValueLeft);
            int colorRight = highestColor(redValueRight, greenValueRight, blueValueRight);
            String colorStringLeft = "";
            String colorStringRight = "";
            switch (colorLeft) {
                case 0:
                    colorStringLeft = "RED";
                    break;
                case 1:
                    colorStringLeft = "Color Undetected.";
                    break;
                case 2:
                    colorStringLeft = "BLUE";
            }
            switch (colorRight) {
                case 0:
                    colorStringRight = "RED";
                    break;
                case 1:
                    colorStringRight = "Color Undetected.";
                    break;
                case 2:
                    colorStringRight = "BLUE";
            }

            telemetry.addData("Color Left", colorStringLeft);
            telemetry.addData("Left Red", redValueLeft);
            telemetry.addData("Left Blue", blueValueLeft);
            telemetry.addData("Left Green", greenValueLeft);
            telemetry.addData("Color Right", colorStringRight);
            telemetry.addData("Right Red", redValueRight);
            telemetry.addData("Right Blue", blueValueRight);
            telemetry.addData("Right Green", greenValueRight);
        }
    }

    public void stop() {
        super.stop();
    }
}
