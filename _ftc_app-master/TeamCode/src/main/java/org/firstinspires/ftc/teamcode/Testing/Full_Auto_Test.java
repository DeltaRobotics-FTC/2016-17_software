package org.firstinspires.ftc.teamcode.Testing;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import for_camera_opmodes.OpModeCamera;

/**
 * Created by RoboticsUser on 1/26/2017.
 */
@Autonomous (name = "Full_Auto_Test", group = "")
public class Full_Auto_Test extends OpModeCamera {
    DcMotor motorL;
    DcMotor motorLF;
    DcMotor motorR;
    DcMotor motorRF;
    Servo bBP;
    ColorSensor colorSensorL;
    ColorSensor colorSensorR;
    OpticalDistanceSensor ODS;
    DcMotor launcherWheel;
    Servo popper;
    DcMotor collector;

    String beaconColorLeft = "ERROR";
    String beaconColorRight = "ERROR";
    String[] beaconColors = new String[2];
    double[] positionRobot = new double[20];
    int center = 320;
    double theta = 112358;
    double topX = 132134;
    int ds1 = 1;
    double leftBoundary = 0;
    double rightBoundary = 0;

    int x = 0;
    int count = 0;
    int rev = 0;
    boolean test = true;
    boolean test1 = true;
    boolean testloop = true;
    boolean test3 = true;
    double constant = 0;
    int lastE = 0;
    int encoderCount;
    int cps = 0;
    long a2 = 0;
    long c2 = 0;
    long c = 0;
    long a = 0;
    double popperUp = 0.9;
    double popperDown = 0.7;
    int avg = 0;
    int p = 5;
    //p-Previous was 20
    int t = 100;
    //t-Previous was 10

    double launcherPower = 0;

    enum States {Drive1, Turn1, Drive2, Turn2, StopRobot}

    States state;

    public void init() {
        motorL = hardwareMap.dcMotor.get("motorL");
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorR = hardwareMap.dcMotor.get("motorR");
        motorRF = hardwareMap.dcMotor.get("motorRF");
        bBP = hardwareMap.servo.get("bBP");
        colorSensorL = hardwareMap.colorSensor.get("colorSensorL");
        colorSensorR = hardwareMap.colorSensor.get("colorSensorR");
        ODS = hardwareMap.opticalDistanceSensor.get("ODS");
        launcherWheel = hardwareMap.dcMotor.get("launcherWheel");
        popper = hardwareMap.servo.get("popper");
        collector = hardwareMap.dcMotor.get("collector");

        colorSensorR.setI2cAddress(I2cAddr.create7bit(0x1e));
        colorSensorL.setI2cAddress(I2cAddr.create7bit(0x26));
        colorSensorL.enableLed(true);
        colorSensorR.enableLed(true);

        setCameraDownsampling(2);
        super.init();

        state = States.Drive1;
    }

    public void loop() {
        if (testloop) {
            constant = System.currentTimeMillis();
            testloop = false;
        }

        telemetry.addData("State", state);
        if(System.currentTimeMillis() - constant > t)
        {
            rev++; //Making sure that the loop keeps running
            constant = System.currentTimeMillis(); //Resetting current time for calculating time difference
            encoderCount = launcherWheel.getCurrentPosition() - lastE; //Change in encoder counts
            lastE = launcherWheel.getCurrentPosition(); //Resetting the encoder position for calculating difference
            cps = encoderCount * (1000/t);
            //****For Averaging****//
            avg = ((avg * (p - 1) + cps) / p);

        }

        switch (state) {
            case Drive1:
                if (motorL.getCurrentPosition() > -300) {
                    motorLF.setPower(-.30);
                    motorL.setPower(-.30);
                    motorRF.setPower(.30);
                    motorR.setPower(.30);
                    telemetry.addData("LeftPower", -.30);
                    telemetry.addData("RightPower", .30);
                } else {
                    motorL.setPower(0.0);
                    motorLF.setPower(0.0);
                    motorR.setPower(0.0);
                    motorRF.setPower(0.0);
                    motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    state = States.Turn1;
                }
                break;
            case Turn1:
                motorLF.setPower(1);
                motorL.setPower(1);
                motorRF.setPower(1);
                motorR.setPower(1);
                telemetry.addData("L", motorL.getPower());
                telemetry.addData("R", motorR.getPower());
                telemetry.addData("LF", motorLF.getPower());
                telemetry.addData("RF", motorRF.getPower());
                if (motorL.getCurrentPosition() < 140)
                {
                    break;
                }
                else
                {
                    motorL.setPower(0.0);
                    motorLF.setPower(0.0);
                    motorR.setPower(0.0);
                    motorRF.setPower(0.0);
                    motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    state = States.StopRobot;
                    break;
                }
            case StopRobot:
                motorL.setPower(0.0);
                motorLF.setPower(0.0);
                motorR.setPower(0.0);
                motorRF.setPower(0.0);
        }
    }

    private String[] readBeacon() {
        String[] endCondition = new String[2];
        if (imageReady()) {
            int redValueLeft = -76800;
            int blueValueLeft = -76800;
            int greenValueLeft = -76800;
            int redValueRight = -76800;
            int blueValueRight = -76800;
            int greenValueRight = -76800;

            Bitmap rgbImage;

            rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds1);

            if((topX - 100 < 0 ))
            {
                leftBoundary = 0;
            }
            else
            {
                leftBoundary = topX - 100;
            }

            if((topX + 100 > 480))
            {
                rightBoundary = 480;
            }
            else
            {
                rightBoundary = topX + 100;
            }
            //Evaluating left side of screen/beacon
            for (int x = (int)leftBoundary; x < (topX - 20); x++) {
                for (int y = 120; y < 270; y++) {
                    int pixelL = rgbImage.getPixel(x, y);
                    redValueLeft += red(pixelL);
                    blueValueLeft += blue(pixelL);
                    greenValueLeft += green(pixelL);
                }
            }
            //Evaluating right side of screen/beacon
            for (int a = (int)(topX + 20); a < rightBoundary; a++) {
                for (int b = 120; b < 270; b++) {
                    int pixelR = rgbImage.getPixel(a, b);
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
            for (int x = 0; x < 480; x++) {
                for (int y = 0; y < 640; y++) {

                    if (x == leftBoundary) {
                        rgbImage.setPixel(x, y, Color.rgb(255, 0, 0));
                    }
                    if (x == rightBoundary) {
                        rgbImage.setPixel(x, y, Color.rgb(255, 0, 0));
                    }
                    if (x == topX - 20) {
                        rgbImage.setPixel(x, y, Color.rgb(0, 0, 255));
                    }
                    if (x == topX + 20) {
                        rgbImage.setPixel(x, y, Color.rgb(0, 0, 255));
                    }
                    if (y == 120 || y == 270) {
                        rgbImage.setPixel(x, y, Color.rgb(0, 255, 0));
                    }
                }
            }
            SaveImage(rgbImage);
            endCondition[0] = colorStringLeft;
            endCondition[1] = colorStringRight;

        }
        return endCondition;
    }

    private double[] positionRobot(Bitmap rgbImage) {
        int topX;
        int topY;
        int topBeginX = 0;
        int topEndX = 0;
        boolean line = true;
        int bottomX;
        int bottomY;
        int bottomBeginX = 0;
        int bottomEndX = 0;
        double theta;
        boolean bottomFlag = false;
        boolean topFlag = false;
        int white = 120;
        double[] returnArray = new double[15];

        int pixelWidth = 480;
        int pixelHeight = 640;

        boolean[] bottomPixels;
        bottomPixels = new boolean[pixelWidth];

        boolean[] topPixels;
        topPixels = new boolean[pixelWidth];
        int pixelPlaceholder;

        //Get bottom line center
        for (int x = 0; x < pixelWidth; x++) {
            pixelPlaceholder = rgbImage.getPixel(x, (pixelHeight - 3));
            if ((Color.red(pixelPlaceholder) > white) && (Color.blue(pixelPlaceholder) > white) && (Color.green(pixelPlaceholder) > white)) {
                bottomPixels[x] = true;
            } else {
                bottomPixels[x] = false;
            }
        }

        for (int x = 0; x < pixelWidth; x++) {
            if (!bottomFlag) {
                if (bottomPixels[x]) {
                    bottomBeginX = x;
                    bottomFlag = true;
                }
            } else {
                if (!bottomPixels[x]) {
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
        for (y = (pixelHeight - 3); line; y--) {
            //Create and analyze an array for each y-value
            for (int x = 0; x < pixelWidth; x++) {
                pixelPlaceholder = rgbImage.getPixel(x, y);
                if ((Color.red(pixelPlaceholder) > white) && (Color.blue(pixelPlaceholder) > white) && (Color.green(pixelPlaceholder) > white)) {
                    topPixels[x] = true;
                } else {
                    topPixels[x] = false;
                }
            }
            line = false;
            for (int x = 0; x < pixelWidth; x++) {
                if (topPixels[x]) {
                    line = true;
                }
            }
        }
        if (y > 635) {
            y = 630;
        }
        for (int x = 0; x < pixelWidth; x++) {
            pixelPlaceholder = rgbImage.getPixel(x, (y + 5));
            if ((Color.red(pixelPlaceholder) > white) && (Color.blue(pixelPlaceholder) > white) && (Color.green(pixelPlaceholder) > white)) {
                topPixels[x] = true;
            } else {
                topPixels[x] = false;
            }
        }

        for (int x = 0; x < pixelWidth; x++) {
            if (!topFlag) {
                if (topPixels[x]) {
                    topBeginX = x;
                    topFlag = true;
                }
            } else {
                if (!topPixels[x]) {
                    topEndX = x;
                    x = pixelWidth;
                }
            }
        }
        //Average the center of the line
        topX = (topBeginX + ((topEndX - topBeginX) / 2));
        topY = y;

        //Analyze the image and find the two points

        double opposite = center - topX;
        double adjacent = topY - bottomY;
        theta = Math.atan(opposite / adjacent);
        theta = Math.toDegrees(theta);
        returnArray[0] = theta;
        returnArray[1] = topX;
        returnArray[2] = topY;
        returnArray[3] = bottomX;
        returnArray[4] = bottomY;

        returnArray[5] = bottomX;
        returnArray[6] = topX;
        returnArray[11] = bottomY;
        returnArray[12] = topY;

        return returnArray;
    }

    private static void sleep(int time) // In milliseconds
    {
        double constant = System.currentTimeMillis();
        double current = System.currentTimeMillis();
        while ((current - constant) <= time) {
            current = System.currentTimeMillis();
        }
    }
}

