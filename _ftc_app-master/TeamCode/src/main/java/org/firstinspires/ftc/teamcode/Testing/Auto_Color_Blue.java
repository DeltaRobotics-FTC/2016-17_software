package org.firstinspires.ftc.teamcode.Testing;

import android.graphics.Bitmap;
import android.graphics.Color;

import for_camera_opmodes.OpModeCamera;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by RoboticsUser on 12/22/2016.
 */
//@Autonomous(name = "Auto_Color_Blue", group = "")
public class Auto_Color_Blue extends OpModeCamera{
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



    boolean ERROR = false;
    private int ds1 = 1;
    double leftBoundary = 75;
    double rightBoundary = 75;
    double center = 300;


    float lColor;
    float rColor;
    int downsampling = 1;

    boolean DriveToWhiteLine = false;
    boolean DrivePastWhiteLine = false;
    boolean TurnOntoWhiteLine = false;
    double[] robotTheta  = new double[16];
    String ReadBeacon = "ERROR";
    boolean DriveToBeaconButton = false;
    double topX = 0;
    double theta = 0;
    int rev = 0;


    double leftPosition = 0.5;
    double rightPosition = 0.5;
    boolean onLeft = false;

    double popperUp = 0.99;
    double popperDown = 0.8;

    enum States {DriveToWhiteLine, DrivePastWhiteLine, TurnOntoWhiteLine, CenterRobot, PositionRobot, ReadBeacon, PushBeaconButton, DriveToBeaconButton, Stop, PositionRobot2}
    States state;

    public void init()
    {
        setCameraDownsampling(2);
        super.init();

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
        popper.setPosition(popperDown);

        colorSensorR.setI2cAddress(I2cAddr.create7bit(0x1e));
        colorSensorL.setI2cAddress(I2cAddr.create7bit(0x26));
        colorSensorL.enableLed(true);
        colorSensorR.enableLed(true);

        motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        bBP.setPosition(0.0079);

        state = States.DriveToWhiteLine;
    }
    public void start()
    {
        colorSensorL.enableLed(false);
        sleep(10);
        colorSensorL.enableLed(true);
        colorSensorR.enableLed(false);
        sleep(10);
        colorSensorR.enableLed(true);
        ODS.enableLed(false);
        sleep(10);
        ODS.enableLed(true);
    }
    public void loop()
    {

        if (imageReady()) {
            Bitmap rgbImage;
            lColor = colorSensorL.argb() / 1000000;
            rColor = colorSensorR.argb() / 1000000;

            //Place order of code below here!
            switch (state) {
                case DriveToWhiteLine:
                    DriveToWhiteLine = goToColor(-0.1, 0.1, 50, 4000, motorL, colorSensorR);
                    if (!DriveToWhiteLine) {
                        break;
                    } else {
                        state = States.DrivePastWhiteLine;
                        resetEncoder(motorL);
                        resetEncoder(motorLF);
                        resetEncoder(motorR);
                        resetEncoder(motorRF);
                        break;
                    }

                case DrivePastWhiteLine:
                    DrivePastWhiteLine = goToEncoder(-0.10, 0.10, 50, motorL);
                    if (!DrivePastWhiteLine) {
                        break;
                    } else {
                        state = States.TurnOntoWhiteLine;
                        resetEncoder(motorL);
                        resetEncoder(motorLF);
                        resetEncoder(motorR);
                        resetEncoder(motorRF);
                        break;
                    }

                case TurnOntoWhiteLine:
                    TurnOntoWhiteLine = goToColor(.15, 0.10, 50, 4000, motorR, colorSensorL);
                    if (!TurnOntoWhiteLine) {
                        break;
                    } else {
                        bBP.setPosition(0.765);
                        state = States.PositionRobot;
                        resetEncoder(motorL);
                        resetEncoder(motorLF);
                        resetEncoder(motorR);
                        resetEncoder(motorRF);
                        break;
                    }

                case PositionRobot:
                    rev++;
                    rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds1);
                    robotTheta = positionRobot(rgbImage);
                    theta = robotTheta[0];
                    topX = robotTheta[1];
                    telemetry.addData("rev", rev);
                    if((topX - center) < -20)
                    {
                        telemetry.addData("Pivot", "Left!");
                        motorL.setPower(-0.05);
                        motorLF.setPower(-0.05);
                        motorR.setPower(-0.05);
                        motorRF.setPower(-0.05);
                        break;
                    }
                    else if(topX - center > 20)
                    {
                        telemetry.addData("Pivot", "Right!");
                        motorL.setPower(0.05);
                        motorLF.setPower(0.05);
                        motorR.setPower(0.05);
                        motorRF.setPower(0.05);
                        break;
                    }
                    else
                    {
                        telemetry.addData("Pivot", (topX - center));
                        motorL.setPower(0.0);
                        motorLF.setPower(0.0);
                        motorR.setPower(0.0);
                        motorRF.setPower(0.0);
                        sleep(2000);
                        state = States.ReadBeacon;
                        break;
                    }

                case PositionRobot2:
                    rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds1);
                    robotTheta = positionRobot(rgbImage);
                    theta = robotTheta[0];
                    topX = robotTheta[1];
                    if((topX - center) < -10)
                    {
                        telemetry.addData("Pivot", "Left!");
                        motorL.setPower(-0.02);
                        motorLF.setPower(-0.02);
                        motorR.setPower(-0.02);
                        motorRF.setPower(-0.02);
                        break;
                    }
                    else if(topX - center > 10)
                    {
                        telemetry.addData("Pivot", "Right!");
                        motorL.setPower(0.02);
                        motorLF.setPower(0.02);
                        motorR.setPower(0.02);
                        motorRF.setPower(0.02);
                        break;
                    }
                    else
                    {
                        telemetry.addData("Pivot", (topX - center));
                        motorL.setPower(0.0);
                        motorLF.setPower(0.0);
                        motorR.setPower(0.0);
                        motorRF.setPower(0.0);
                        sleep(3000);
                        state = States.ReadBeacon;
                        break;
                    }
                case ReadBeacon:
                    bBP.setPosition(0.0079);
                    ReadBeacon = readBeacon(robotTheta[1]);
                    if (ReadBeacon.equals("ERROR") || ReadBeacon.equals("test1") || ReadBeacon.equals("test2"))
                    {
                        telemetry.addData("Stuck Here?", "ReadBeacon");
                    }
                    else
                    {
                        bBP.setPosition(0.765);
                        state = States.PushBeaconButton;
                    }
                    break;

                case PushBeaconButton:
                    if (ReadBeacon.equals("red")) //left is red
                    {
                        bBP.setPosition(rightPosition);
                    } else
                    {
                        bBP.setPosition(leftPosition);
                    }
                    resetEncoder(motorL);
                    resetEncoder(motorLF);
                    resetEncoder(motorR);
                    resetEncoder(motorRF);
                    state = States.DriveToBeaconButton;
                    break;

                case DriveToBeaconButton:
                    if(ODS.getRawLightDetected() < .25)
                    {
                        motorL.setPower(-0.1);
                        motorLF.setPower(-0.1);
                        motorR.setPower(0.1);
                        motorRF.setPower(0.1);
                    }
                    else
                    {
                        state = States.Stop;
                        resetEncoder(motorL);
                        resetEncoder(motorLF);
                        resetEncoder(motorR);
                        resetEncoder(motorRF);
                    }
                    break;

                
                case Stop:
                    motorL.setPower(0.0);
                    motorLF.setPower(0.0);
                    motorR.setPower(0.0);
                    motorRF.setPower(0.0);
                    break;
                }
            //Telemetry Goes Here!!!
            telemetry.addData("Difference", (topX - center));
            telemetry.addData("MotorR", (motorR.getPower()));
            telemetry.addData("MotorL", (motorL.getPower()));
            telemetry.addData("MotorRF", (motorRF.getPower()));
            telemetry.addData("MotorLF", (motorLF.getPower()));
            telemetry.addData("ODS Raw Light Detected", ODS.getRawLightDetected());
            telemetry.addData("Current State", state);
            telemetry.addData("ERROR", ERROR);
            telemetry.addData("Read Beacon", ReadBeacon);
            //telemetry.addData("Color", ReadBeacon);
            }
        }
    private static void sleep(int time) // In milliseconds
    {
        double constant = System.currentTimeMillis();
        double current = System.currentTimeMillis();
        while ((current - constant) <= time) {
            current = System.currentTimeMillis();
        }
    }
    private boolean goToEncoder(double leftPower, double rightPower, int encPosition, DcMotor encMotor)
    {
        boolean endCondition;
        motorL.setPower(leftPower);
        motorLF.setPower(leftPower);
        motorR.setPower(rightPower);
        motorRF.setPower(rightPower);

        if(Math.abs(encMotor.getCurrentPosition()) < encPosition)
        {
            endCondition = false;
        }
        else
        {
            motorL.setPower(0.0);
            motorLF.setPower(0.0);
            motorR.setPower(0.0);
            motorRF.setPower(0.0);
            endCondition = true;
        }
        return endCondition;
    }
    private void resetEncoder(DcMotor motor)
    {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private boolean goToColor(double leftPower, double rightPower, float targetColor, int encTimeout, DcMotor encMotor, ColorSensor colorSensor)
    {
        boolean endCondition;
        motorLF.setPower(leftPower);
        motorL.setPower(leftPower);
        motorRF.setPower(rightPower);
        motorR.setPower(rightPower);
        telemetry.addData("LeftPower", leftPower);
        telemetry.addData("RightPower", rightPower);

        if((colorSensor.argb() / 1000000) < targetColor)
        {
            endCondition = false;
        }
        else
        {
            motorL.setPower(0.0);
            motorLF.setPower(0.0);
            motorR.setPower(0.0);
            motorRF.setPower(0.0);
            endCondition = true;
        }
        return endCondition;
    }
    private boolean offRight(double maxPower, double medPower, double minPower, float targetColor, double distance)
    {
        boolean endCondition;
        motorL.setPower(-minPower);
        motorR.setPower(maxPower);
        if((colorSensorR.argb() / 1000000) >= targetColor)
        {
            endCondition = false;
        }
        else
        {
            endCondition = true;
        }
        return endCondition;
    }
    private boolean offLeft(double maxPower, double medPower, double minPower, float targetColor, double distance)
    {
        boolean endCondition;
        motorL.setPower(-maxPower);
        motorR.setPower(minPower);
        if((colorSensorL.argb() / 1000000) >= targetColor)
        {
            endCondition = false;
        }
        else
        {
            endCondition = true;
        }
        return endCondition;
    }
    private double[] positionRobot(Bitmap rgbImage)
    {
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
            if(y > 635)
            {
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

            double opposite = center-topX;
            double adjacent = topY-bottomY;
            theta = Math.atan(opposite/adjacent);
            theta = Math.toDegrees(theta);
            returnArray[0] = theta;
            returnArray[1] = topX;

            return returnArray;
    }
    private String readBeacon(double topX)
    {
        String leftColor = "test1";
        String rightColor = "test2";
        if (imageReady()) {
            int redValueLeft = -76800;
            int blueValueLeft = -76800;
            int greenValueLeft = -76800;
            int redValueRight = -76800;
            int blueValueRight = -76800;
            int greenValueRight = -76800;

            Bitmap rgbImage;

            rgbImage = convertYuvImageToRgb(yuvImage, width, height, downsampling);

            //Evaluating left side of screen/beacon
            if(topX - 75 < 0)
            {
                leftBoundary = topX;
            }
            if(topX + 75 > 480)
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

            int colorLeft = highestColor(redValueLeft, greenValueLeft, blueValueLeft);
            int colorRight = highestColor(redValueRight, greenValueRight, blueValueRight);

            switch (colorLeft)
            {
                case 0:
                    leftColor = "red";
                    break;
                case 2:
                    leftColor = "blue";
                    break;
            }

            switch (colorRight)
            {
                case 0:
                    rightColor = "red";
                    break;
                case 2:
                    rightColor = "blue";
                    break;
            }


        }
        if(rightColor.equals(leftColor))
        {
            return "ERROR";
        }
        else
        {
            return leftColor;
        }
    }
}