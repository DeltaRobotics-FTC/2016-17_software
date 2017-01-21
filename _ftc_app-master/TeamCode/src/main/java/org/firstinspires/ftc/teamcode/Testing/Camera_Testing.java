package org.firstinspires.ftc.teamcode.Testing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import android.graphics.Bitmap;
import android.graphics.Color;

import java.lang.reflect.Array;

import for_camera_opmodes.OpModeCamera;
/**
 * Created by RoboticsUser on 12/29/2015.
 */
//@Autonomous(name = "Camera_Testing_Op", group = "SensorTesting")

public class Camera_Testing extends OpModeCamera{
    private int looped = 0;
    private int ds1 = 1;
    private boolean flag = true;
    int white = 120;
    double topX;
    double leftBoundary = 100;
    double rightBoundary = 100;
    double center = 320;
    String adjustString = "";

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

            double returns[] = findLineAngle(rgbImage, 480, 640);

            for (int x = 0; x < 480; x++) {
                for (int y = 0; y < 640; y++) {
                    /*if(x == returns[1])
                    {
                        rgbImage.setPixel(x,y, Color.rgb(0,255,255));
                    }
                    if(x == returns[2])
                    {
                        rgbImage.setPixel(x,y, Color.rgb(0,255,0));
                    }
                    if(x == returns[3])
                    {
                        rgbImage.setPixel(x,y, Color.rgb(255,0,255));
                    }
                    if(x == returns[4])
                    {
                        rgbImage.setPixel(x,y, Color.rgb(255,255,0));
                    }*/
                    if(y == returns[11])
                    {
                        rgbImage.setPixel(x,y, Color.rgb(255,0,0));
                    }
                    if(y == returns[12])
                    {
                        rgbImage.setPixel(x,y, Color.rgb(0,0,255));
                    }
                    if(x == returns[5])
                    {
                        rgbImage.setPixel(x,y, Color.rgb(255,0,0));
                    }
                    if(x == returns[6])
                    {
                        rgbImage.setPixel(x,y, Color.rgb(0,0,255));
                    }
                }
            }
            SaveImage(rgbImage);

            if((topX - center) < -10)
            {
                adjustString = "move left!";
            }
            else if((topX - center) > 10)
            {
                adjustString = "move right!";
            }
            else
            {
                adjustString = "go now!";
            }
            telemetry.addData("Theta", returns[0]);
            telemetry.addData("", adjustString);

            telemetry.addData("Center", topX - center);
            //Put results to phone with red/blue/green
            topX = returns[6];
            if(topX - 100 < 0)
            {
                leftBoundary = topX;
            }
            if(topX + 100 > 480)
            {
                rightBoundary = (480 - topX);
            }

            //Evaluating left side of screen/beacon
            for (int x = ((int)(topX - leftBoundary)); x < topX; x++) {
                for (int y = 160; y < 320; y++) {
                    int pixelL = rgbImage.getPixel(x, y);
                    redValueLeft += red(pixelL);
                    blueValueLeft += blue(pixelL);
                    greenValueLeft += green(pixelL);
                }
            }

            //Evaluating right side of screen/beacon
            for (int a = (int)topX; a < ((int)(topX + rightBoundary)); a++) {
                for (int b = 160; b < 320; b++) {
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
            telemetry.addData("Left Color", colorStringLeft);
            telemetry.addData("Right Color", colorStringRight);

        }
    }

    private double[] findLineAngle(Bitmap image, int pixelWidth, int pixelHeight)
    {
        double topX;
        double topY;
        int topBeginX = 0;
        int topEndX = 0;
        boolean line = true;
        double bottomX;
        double bottomY;
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
        int pixelPlaceholder;

        //Get bottom line center
        for (int x = 0; x < pixelWidth; x++)
        {
            pixelPlaceholder = image.getPixel(x,(pixelHeight - 3));
            if((Color.red(pixelPlaceholder) > white) && (Color.blue(pixelPlaceholder) > white) && (Color.green(pixelPlaceholder) > white))
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
        bottomX = (bottomBeginX + ((bottomEndX - bottomBeginX) / 2));
        bottomY = (pixelHeight - 3);
        //Point of topLine
        int y;
        for(y = (pixelHeight-3); line; y--)
        {
            //Create and analyze an array for each y-value
            for(int x = 0; x < pixelWidth; x++)
            {
                pixelPlaceholder = image.getPixel(x,y);
                if((Color.red(pixelPlaceholder) > white) && (Color.blue(pixelPlaceholder) > white) && (Color.green(pixelPlaceholder) > white))
                {
                    topPixels[x] = true;
                }
                else
                {
                    topPixels[x] = false;
                }
            }
            line = false;
            for(int x = 0; x < pixelWidth; x++)
            {
                if(topPixels[x])
                {
                    line = true;
                }
            }
        }
        if(y > 635)
        {
            y = 630;
        }

        for(int x = 0; x < pixelWidth; x++)
        {
            pixelPlaceholder = image.getPixel(x,(y+5));
            if((Color.red(pixelPlaceholder) > white) && (Color.blue(pixelPlaceholder) > white) && (Color.green(pixelPlaceholder) > white))
            {
                topPixels[x] = true;
            }
            else
            {
                topPixels[x] = false;
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
        topX = (topBeginX + ((topEndX - topBeginX) / 2));
        topY = y;

        //Analyze the image and find the two points

        double opposite = center-topX;
        double adjacent = topY-bottomY;
        theta = Math.atan(opposite/adjacent);
        theta = Math.toDegrees(theta);
        returnArray[0] = theta;

        returnArray[1] = bottomBeginX;
        returnArray[2] = bottomEndX;
        returnArray[3] = topBeginX;
        returnArray[4] = topEndX;
        returnArray[5] = bottomX;
        returnArray[6] = topX;
        returnArray[11] = bottomY;
        returnArray[12] = topY;
        return returnArray;
    }

    public void stop() {

        super.stop();
    }
}
