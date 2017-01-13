package org.firstinspires.ftc.teamcode.Testing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import android.graphics.Bitmap;
import android.graphics.Color;

import java.lang.reflect.Array;

import for_camera_opmodes.OpModeCamera;
/**
 * Created by RoboticsUser on 12/29/2015.
 */
@Autonomous(name = "Camera_Testing_Op", group = "SensorTesting")

public class Camera_Testing extends OpModeCamera{
    private int looped = 0;
    private int ds1 = 1;
    private boolean flag = true;
    int white = 50;

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
            rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds1);

            //Put results to phone with red/blue/green


            //Evaluating left side of screen/beacon
            for (int x = 0; x < 240; x++) {
                for (int y = 0; y < 640; y++) {
                    int pixelL = rgbImage.getPixel(x, y);
                    redValueLeft += red(pixelL);
                    blueValueLeft += blue(pixelL);
                    greenValueLeft += green(pixelL);
                }
            }

            //Evaluating right side of screen/beacon
            for (int a = 121; a < 240; a++) {
                for (int b = 90; b < 230; b++) {
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
                    colorStringLeft = "GREEN";
                    break;
                case 2:
                    colorStringLeft = "BLUE";
            }
            switch (colorRight) {
                case 0:
                    colorStringRight = "RED";
                    break;
                case 1:
                    colorStringRight = "GREEN";
                    break;
                case 2:
                    colorStringRight = "BLUE";
            }
            double returns[] = findLineAngle(rgbImage, 480, 640);

            for (int x = 0; x < 480; x++) {
                for (int y = 0; y < 640; y++) {
                    if(x == returns[1] && y == returns[11])
                    {
                        rgbImage.setPixel(x,y, Color.rgb(255,0,0));
                    }
                }
            }
            SaveImage(rgbImage);

            /*telemetry.addData("Color Left", colorStringLeft);
            telemetry.addData("Left Red", redValueLeft);
            telemetry.addData("Left Blue", blueValueLeft);
            telemetry.addData("Left Green", greenValueLeft);
            telemetry.addData("Color Right", colorStringRight);
            telemetry.addData("Right Red", redValueRight);
            telemetry.addData("Right Blue", blueValueRight);
            telemetry.addData("Right Green", greenValueRight);*/

            telemetry.addData("Theta", returns[0]);
            telemetry.addData("BottomBeginX", returns[1]);
            telemetry.addData("BottomEndX", returns[2]);
            telemetry.addData("TopBeginX", returns[3]);
            telemetry.addData("TopEndX", returns[4]);
            telemetry.addData("BottomX", returns[5]);
            telemetry.addData("TopX", returns[6]);
        }
    }

    private double[] findLineAngle(Bitmap image, int pixelWidth, int pixelHeight)
    {
        int topX = 1;
        int topY = 1;
        int topBeginX = 0;
        int topEndX = 0;
        boolean line = true;
        int bottomX = 2;
        int bottomY = 2;
        int bottomBeginX = 0;
        int bottomEndX = 0;
        double theta;
        boolean bottomFlag = false;
        boolean topFlag = false;
        double[] returnArray = new double[15];

        boolean[] bottomPixels;
        bottomPixels = new boolean[pixelWidth];

        boolean[] topPixels;
        topPixels = new boolean[pixelWidth];

        //Get bottom line center
        for (int x = 0; x < pixelWidth; x++)
        {
            int pixelPlaceholder = image.getPixel(x,(pixelHeight - 3));
            if(red(pixelPlaceholder) > white && blue(pixelPlaceholder) > white && green(pixelPlaceholder) > white )
            {
                bottomPixels[x] = true;
            }
            else
            {
                bottomPixels[x] = false;
            }
        }

        for (int x = 0; x < pixelWidth; x++)
        {
            if(!bottomFlag)
            {
                if (bottomPixels[x])
                {
                    bottomBeginX = x;
                    bottomFlag = true;
                }
            }
            else
            {
                if(!bottomPixels[x])
                {
                    bottomEndX = x;
                    x = pixelWidth;
                }
            }
        }
        //Average the center of the line
        bottomX = (bottomEndX - bottomBeginX);
        bottomY = (pixelHeight - 3);
        //Point of topLine
        int y;
        for(y = 639; line; y--)
        {
            //Create and analyze an array for each y-value
            for(int x = 0; x < pixelWidth; x++)
            {
                int pixelPlaceholder = image.getPixel(x,y);
                if(red(pixelPlaceholder) > white && blue(pixelPlaceholder) > white && green(pixelPlaceholder) > white )
                {
                    topPixels[x] = true;
                }
                else
                {
                    topPixels[x] = false;
                }
            }
            for(int x = 0; x < pixelWidth; x++)
            {
                line = false;
                if(topPixels[x])
                {
                    line = true;
                }
            }
        }
        for (int x = 0; x < pixelWidth; x++)
        {
            if(!topFlag)
            {
                if (topPixels[x])
                {
                    topBeginX = x;
                    topFlag = true;
                }
            }
            else
            {
                if(!topPixels[x])
                {
                    topEndX = x;
                    x = pixelWidth;
                }
            }
        }
        //Average the center of the line
        topX = (topEndX - topBeginX);
        topY = y;

        //Analyze the image and find the two points

        int opposite = bottomX-topX;
        int adjacent = bottomY-topY;
        theta = Math.atan(opposite/adjacent);
        theta = Math.toDegrees(theta);
        returnArray[0] = theta;

        returnArray[1] = bottomBeginX;
        returnArray[2] = bottomEndX;
        returnArray[3] = topBeginX;
        returnArray[4] = topEndX;
        returnArray[5] = bottomX;
        returnArray[6] = topX;

        //returnArray[7] = bottomBeginY;
        //returnArray[8] = bottomEndY;
        //returnArray[9] = topBeginY;
        //returnArray[10] = topEndY;
        returnArray[11] = bottomY;
        returnArray[12] = topY;
        return returnArray;
    }

    public void stop() {

        super.stop();
    }
}
