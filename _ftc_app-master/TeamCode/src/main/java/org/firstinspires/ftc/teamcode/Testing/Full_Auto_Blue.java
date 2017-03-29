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
@Autonomous (name = "Full_Auto_Blue", group = "")
public class Full_Auto_Blue extends OpModeCamera {
    DcMotor motorL;
    DcMotor motorLF;
    DcMotor motorR;
    DcMotor motorRF;
    Servo bBP;
    Servo boot;
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
    //**********************
    //DO NOT CHANGE THIS!!!!
    //**********************
    int center = 240;
    //**********************
    //DO NOT CHANGE THIS!!!!
    //**********************
    double theta = 112358;
    double topX = 132134;
    int ds1 = 1;
    double leftBoundary = 0;
    double rightBoundary = 0;

    int positionNum = 0;
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
    long launchA2 = 0;
    long launchC2 = 0;
    long launchC = 0;
    long launchA = 0;
    double popperUp = 0.69;
    double popperDown = 0.94;
    int avg = 0;
    int p = 5;
    int iterationCount;
    //p-Previous was 20
    int t = 100;
    //t-Previous was 10
    int motorLCurrentEnc = 0;
    int motorRCurrentEnc = 0;
    int encA = 700;
    int encB = 250;
    int encC = 1700;
    int encD = 325;
    int encE = 300;
    int encF = 750;
    int encG = 100;
    //encD previously -150


    int drive1Telemetry = 0;
    int turn1Telemetry = 0;
    int longForwardTelemetry = 0;
    int TurntoWhiteLineTelemetry = 0;

    double distance;
    double launcherPower = 0;

    enum States {Drive1, Turn1, ReadBeacon, ShortBackward, LongForward, PositionRobot, ShortForward, TurnToWhiteLine, TurnPastWhiteLine, ForwardToBeacon, StopRobot, SHOOT, SHOOT2, ShootTurn}

    States state;

    public void init() {
        motorL = hardwareMap.dcMotor.get("motorL");
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorR = hardwareMap.dcMotor.get("motorR");
        motorRF = hardwareMap.dcMotor.get("motorRF");
        bBP = hardwareMap.servo.get("bBP");
        boot = hardwareMap.servo.get("boot");
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


        //boot.setPosition(0.1);
        popper.setPosition(popperDown);
        bBP.setPosition(.01);


        setCameraDownsampling(2);
        super.init();

        state = States.Drive1;
    }

    public void loop() {
        if (testloop) {
            constant = System.currentTimeMillis();
            testloop = false;
        }
        distance = ODS.getRawLightDetected();
        telemetry.addData("ODS", distance);
        telemetry.addData("State", state);
        if(System.currentTimeMillis() - constant > t)
        {
            rev++; //Making sure that the loop keeps running
            constant = System.currentTimeMillis(); //Resetting current time for calculating time difference
            encoderCount = launcherWheel.getCurrentPosition() - lastE; //Change in encoder counts
            lastE = launcherWheel.getCurrentPosition(); //Resetting the encoder position for calculating difference
            cps = encoderCount * (1000/t);

            avg = ((avg * (p - 1) + cps) / p);

        }
        telemetry.addData("Average", avg);

        switch (state) {

            case Drive1:
                while(motorR.getCurrentPosition() < motorRCurrentEnc + encA)
                {
                    motorLF.setPower(-.30);
                    motorL.setPower(-.30);
                    motorRF.setPower(.30);
                    motorR.setPower(.30);
                }
                telemetry.addData("Drive1", motorR.getCurrentPosition());
                motorL.setPower(0.0);
                motorLF.setPower(0.0);
                motorR.setPower(0.0);
                motorRF.setPower(0.0);
                motorLCurrentEnc = motorL.getCurrentPosition();
                motorRCurrentEnc = motorR.getCurrentPosition();
                sleep(250);
                state = States.Turn1;
                break;

            case Turn1:
                while(motorR.getCurrentPosition() < motorRCurrentEnc + encB)
                {
                    motorLF.setPower(.65);
                    motorL.setPower(.65);
                    motorRF.setPower(.65);
                    motorR.setPower(.65);
                }
                telemetry.addData("Turn1", motorL.getCurrentPosition());
                motorL.setPower(0.0);
                motorLF.setPower(0.0);
                motorR.setPower(0.0);
                motorRF.setPower(0.0);
                motorLCurrentEnc = motorL.getCurrentPosition();
                motorRCurrentEnc = motorR.getCurrentPosition();
                sleep(250);
                state = States.LongForward;
                break;

            case LongForward:
                while(motorR.getCurrentPosition() < motorRCurrentEnc + encC)
                {
                    motorLF.setPower(-.35);
                    motorL.setPower(-.35);
                    motorRF.setPower(.35);
                    motorR.setPower(.35);
                }
                telemetry.addData("LongForward", motorR.getCurrentPosition());
                motorL.setPower(0.0);
                motorLF.setPower(0.0);
                motorR.setPower(0.0);
                motorRF.setPower(0.0);
                motorLCurrentEnc = motorL.getCurrentPosition();
                motorRCurrentEnc = motorR.getCurrentPosition();
                sleep(250);
                state = States.TurnToWhiteLine;
                break;

            case TurnToWhiteLine:
                while(motorR.getCurrentPosition() < motorRCurrentEnc + encD)
                {
                    motorLF.setPower(.65);
                    motorL.setPower(.65);
                    motorRF.setPower(.65);
                    motorR.setPower(.65);
                }
                telemetry.addData("TurnToWhiteLine", motorL.getCurrentPosition());
                motorL.setPower(0.0);
                motorLF.setPower(0.0);
                motorR.setPower(0.0);
                motorRF.setPower(0.0);
                motorLCurrentEnc = motorL.getCurrentPosition();
                motorRCurrentEnc = motorR.getCurrentPosition();
                sleep(250);
                state = States.PositionRobot;
                break;

            case PositionRobot:
                Bitmap rgbImage;
                rgbImage = convertYuvImageToRgb(yuvImage, width, height, 1);
                positionRobot = positionRobot(rgbImage);
                theta = positionRobot[0];
                topX = positionRobot[1];
                telemetry.addData("Theta", theta);
                telemetry.addData("Pivoting", topX - center);
                motorLF.setPower(0);
                motorL.setPower(0);
                motorRF.setPower(0);
                motorR.setPower(0);

                    /*for (int x = 0; x < 480; x++) {
                        for (int y = 0; y < 640; y++) {

                            if (y == positionRobot[11]) {
                                rgbImage.setPixel(x, y, Color.rgb(255, 0, 0));
                            }
                            if (y == positionRobot[12]) {
                                rgbImage.setPixel(x, y, Color.rgb(0, 0, 255));
                            }
                            if (x == positionRobot[5]) {
                                rgbImage.setPixel(x, y, Color.rgb(255, 0, 0));
                            }
                            if (x == positionRobot[6]) {
                                rgbImage.setPixel(x, y, Color.rgb(0, 0, 255));
                            }
                        }
                    }
                    SaveImage(rgbImage);*/
                if ((topX - center) < -25) {
                    telemetry.addData("Pivot Fast", "Left!");
                    motorL.setPower(-.45);
                    motorLF.setPower(-.45);
                    motorR.setPower(-.45);
                    motorRF.setPower(-.45);
                    sleep(400);
                    break;

                } else if ((topX - center) > 25) {
                    telemetry.addData("Pivot Fast", "Right!");
                    motorL.setPower(.45);
                    motorLF.setPower(.45);
                    motorR.setPower(.45);
                    motorRF.setPower(.45);
                    sleep(400);
                    break;
                }
                else {
                    telemetry.addData("Pivot", (topX - center));
                    motorL.setPower(0.0);
                    motorLF.setPower(0.0);
                    motorR.setPower(0.0);
                    motorRF.setPower(0.0);
                    if(positionNum != 1)
                    {
                        positionNum = 1;
                        motorLCurrentEnc = motorL.getCurrentPosition();
                        motorRCurrentEnc = motorR.getCurrentPosition();
                        state = States.ShortForward;
                    }
                    else
                    {
                        state = States.ReadBeacon;
                    }
                    break;
                }
                    /*
                    telemetry.addData("Theta", positionRobot[0]);
                    telemetry.addData("TopX", positionRobot[1]);
                    telemetry.addData("Top Y", positionRobot[2]);
                    telemetry.addData("Bottom X", positionRobot[3]);
                    telemetry.addData("Bottom Y", positionRobot[4]);
                    break*/

            case ShortForward:
                if (motorR.getCurrentPosition() < motorRCurrentEnc + encE) {
                    motorLF.setPower(-.20);
                    motorL.setPower(-.20);
                    motorRF.setPower(.20);
                    motorR.setPower(.20);
                }
                else
                {
                    motorL.setPower(0.0);
                    motorLF.setPower(0.0);
                    motorR.setPower(0.0);
                    motorRF.setPower(0.0);
                    sleep(250);
                    state = States.ReadBeacon;
                }
                break;

            case ReadBeacon:
                beaconColors = readBeacon();
                beaconColorLeft = beaconColors[0];
                beaconColorRight = beaconColors[1];
                telemetry.addData("Left Color", beaconColorLeft);
                telemetry.addData("Right Color", beaconColorRight);
                if(beaconColorLeft.equals(beaconColorRight))
                {
                    /*sleep(500);
                    telemetry.addData("Same Color", "Why?");
                    break;*/
                    if(beaconColorRight.equals("BLUE") || (beaconColorRight.equals("GREEN")))
                    {
                        bBP.setPosition(0.88);
                        state = States.ForwardToBeacon;
                        motorLCurrentEnc = motorL.getCurrentPosition();
                        motorRCurrentEnc = motorR.getCurrentPosition();
                        sleep(500);
                        break;
                    }
                    if(beaconColorRight.equals("RED"))
                    {
                        bBP.setPosition(0.52);
                        state = States.ForwardToBeacon;
                        motorLCurrentEnc = motorL.getCurrentPosition();
                        motorRCurrentEnc = motorR.getCurrentPosition();
                        sleep(500);
                        break;
                    }
                }
                if(beaconColorLeft.equals("BLUE") || (beaconColorLeft.equals("GREEN")))
                {
                    bBP.setPosition(0.52);
                    motorLCurrentEnc = motorL.getCurrentPosition();
                    motorRCurrentEnc = motorR.getCurrentPosition();
                    state = States.ForwardToBeacon;
                    sleep(500);
                    break;
                }
                if(beaconColorLeft.equals("RED"))
                {
                    bBP.setPosition(0.88);
                    state = States.ForwardToBeacon;
                    motorLCurrentEnc = motorL.getCurrentPosition();
                    motorRCurrentEnc = motorR.getCurrentPosition();
                    sleep(500);
                    break;
                }
                break;

            case ForwardToBeacon:
                if(distance < .12 && (motorR.getCurrentPosition() < motorRCurrentEnc + encF))
                {
                    telemetry.addData("ODS_Raw_Light1", ODS.getRawLightDetected());
                    motorLF.setPower(-.15);
                    motorL.setPower(-.15);
                    motorRF.setPower(.15);
                    motorR.setPower(.15);
                    break;
                }
                else
                {
                    telemetry.addData("ODS_Raw_Light2", ODS.getRawLightDetected());
                    motorL.setPower(0.0);
                    motorLF.setPower(0.0);
                    motorR.setPower(0.0);
                    motorRF.setPower(0.0);
                    motorLCurrentEnc = motorL.getCurrentPosition();
                    motorRCurrentEnc = motorR.getCurrentPosition();
                    state = States.ShortBackward;
                    break;
                }

            case ShortBackward:
                if ((motorR.getCurrentPosition() > motorRCurrentEnc - encG))
                {
                    motorLF.setPower(.20);
                    motorL.setPower(.20);
                    motorRF.setPower(-.20);
                    motorR.setPower(-.20);
                }
                else
                {
                    motorL.setPower(0.0);
                    motorLF.setPower(0.0);
                    motorR.setPower(0.0);
                    motorRF.setPower(0.0);
                    state = States.SHOOT;
                }
                break;

            case SHOOT:
                if(test)
                {
                    //c = System.currentTimeMillis();
                    test = false;
                    launcherPower = -.47;
                    launcherWheel.setPower(launcherPower);
                    //avg = launcherWheel.getCurrentPosition();
                }
                else
                {
                    if(count < 2)
                    {
                        if (avg < -2200 && avg > -2300)
                        {
                            count++;
                            state = States.SHOOT2;
                        }
                        else
                        {
                            if(test3)
                            {
                                launchC2 = System.currentTimeMillis();
                                test3 = false;
                            }
                            launchA2 = System.currentTimeMillis();
                            if((launchA2 - launchC2) < t)
                            {
                                break;
                            }
                            else
                            {
                                if (avg < -2200) {
                                    launcherPower = launcherPower + .008;
                                }
                                if (avg > -2300) {
                                    launcherPower = launcherPower - .008;
                                }
                                launcherWheel.setPower(launcherPower);
                                test3 = true;
                            }

                            break;
                        }

                        test = true;
                    }
                    else
                    {
                        launcherWheel.setPower(0);
                        state = States.StopRobot;
                        break;
                    }
                    break;
            }
            break;

            case SHOOT2:
                if(test1)
                {
                    launchC = System.currentTimeMillis();
                    test1 = false;
                }
                popper.setPosition(popperUp);
                launchA = System.currentTimeMillis();
                if((launchA - launchC) < 1000)
                {
                    break;
                }
                else
                {
                    popper.setPosition(popperDown);
                    while(iterationCount < 1000)
                    {
                        iterationCount++;
                    }
                    state = States.SHOOT;
                    test = true;
                    test1 = true;
                }
                break;

            case ShootTurn:
                while(motorR.getCurrentPosition() < 100)
                {
                    motorLF.setPower(.65);
                    motorL.setPower(.65);
                    motorRF.setPower(.65);
                    motorR.setPower(.65);
                }
                motorL.setPower(0.0);
                motorLF.setPower(0.0);
                motorR.setPower(0.0);
                motorRF.setPower(0.0);
                motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                state = States.SHOOT;
                break;

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

