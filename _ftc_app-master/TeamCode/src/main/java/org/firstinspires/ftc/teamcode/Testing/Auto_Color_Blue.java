package org.firstinspires.ftc.teamcode.Testing;

import android.graphics.Bitmap;

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
@Autonomous(name = "Auto_Color_Blue", group = "")
public class Auto_Color_Blue extends OpModeCamera{
    DcMotor motorL;
    DcMotor motorR;
    Servo bBP;
    ColorSensor colorSensorL;
    ColorSensor colorSensorR;
    OpticalDistanceSensor ODS;


    boolean ERROR = false;


    float lColor;
    float rColor;
    int downsampling = 2;

    boolean DriveToWhiteLine = false;
    boolean DrivePastWhiteLine = false;
    boolean TurnOntoWhiteLine = false;
    boolean LineFollowingLeft = false;
    boolean LineFollowingRight = false;
    boolean OffRight = false;
    boolean OffLeft = false;
    String ReadBeacon;
    boolean DriveToBeaconButton = false;

    double leftPosition = 0.5;
    double rightPosition = 0.5;
    boolean onLeft = false;

    enum States {DriveToWhiteLine, DrivePastWhiteLine, TurnOntoWhiteLine, CenterRobot, LineFollowing, ReadBeacon, PushBeaconButton, DriveToBeaconButton, Stop}
    States state;

    public void init()
    {
        motorL = hardwareMap.dcMotor.get("motorL");
        motorR = hardwareMap.dcMotor.get("motorR");
        bBP = hardwareMap.servo.get("bBP");
        colorSensorL = hardwareMap.colorSensor.get("colorSensorL");
        colorSensorR = hardwareMap.colorSensor.get("colorSensorR");
        ODS = hardwareMap.opticalDistanceSensor.get("ODS");

        colorSensorR.setI2cAddress(I2cAddr.create7bit(0x1e));
        colorSensorL.setI2cAddress(I2cAddr.create7bit(0x26));
        colorSensorL.enableLed(true);
        colorSensorR.enableLed(true);

        motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bBP.setPosition(0.0079);

        setCameraDownsampling(2);
        super.init();

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
        lColor = colorSensorL.argb() / 1000000;
        rColor = colorSensorR.argb() / 1000000;

        //Place order of code below here!
        switch(state)
        {
            case DriveToWhiteLine:
                DriveToWhiteLine = goToColor(-.15, .15, 50, 4000, motorL, colorSensorR);
                if(!DriveToWhiteLine)
                {
                    break;
                }
                else
                {
                    state = States.DrivePastWhiteLine;
                    resetEncoder(motorL);
                    resetEncoder(motorR);
                    break;
                }

            case DrivePastWhiteLine:
                DrivePastWhiteLine = goToEncoder(-.20, .20, 0, motorL);
                if(!DrivePastWhiteLine)
                {
                    break;
                }
                else
                {
                    state = States.TurnOntoWhiteLine;
                    resetEncoder(motorL);
                    resetEncoder(motorR);
                    break;
                }

            case TurnOntoWhiteLine:
                TurnOntoWhiteLine = goToColor(.15, 0.0, 50, 4000, motorR, colorSensorL);
                if(!TurnOntoWhiteLine)
                {
                    break;
                }
                else
                {
                    bBP.setPosition(0.765);
                    state = States.CenterRobot;
                    resetEncoder(motorL);
                    resetEncoder(motorR);
                    break;
                }

            case CenterRobot:
                if((colorSensorL.argb() / 1000000) >= 50)
                {
                    motorL.setPower(-.10);
                    motorR.setPower(.10);
                    break;
                }
                else
                {
                    state = States.LineFollowing;
                    break;
                }

            case LineFollowing:
                if(ODS.getRawLightDetected() < 0.15)
                {
                    if(((colorSensorL.argb() / 1000000) <= 50) && ((colorSensorR.argb() / 1000000) <= 50))
                    {
                        //Both are black; drive straight
                        motorL.setPower(-.05);
                        motorR.setPower(.05);
                    }
                    else if(((colorSensorL.argb() / 1000000) >= 50) && ((colorSensorR.argb() / 1000000) <= 50))
                    {
                        //Left is white, right is black; turn to left
                        OffLeft = offLeft(0.05, 0.15, 0.01, 50, .15);
                        if(!OffLeft)
                        {
                            break;
                        }
                        else
                        {
                            OffLeft = false;
                        }
                    }
                    else if(((colorSensorL.argb() / 1000000) <= 50) && ((colorSensorR.argb() / 1000000) >= 50))
                    {
                        //Left is black, right is white; turn to right
                        OffRight = offRight(0.05, 0.15, 0.01, 50, .15);
                        if(!OffRight)
                        {
                            break;
                        }
                        else
                        {
                            OffRight = false;
                            break;
                        }
                    }
                    else
                    {
                        ERROR = true;
                        break;
                    }
                }
                else
                {
                    resetEncoder(motorL);
                    resetEncoder(motorR);
                    state = States.ReadBeacon;
                    break;
                }

            case ReadBeacon:
                bBP.setPosition(0.0079);
                ReadBeacon = readBeacon();
                if(ReadBeacon.equals("ERROR") || ReadBeacon.equals("test1") || ReadBeacon.equals("test2"))
                {
                    break;
                }
                else
                {
                    state = States.PushBeaconButton;
                    break;
                }

            case PushBeaconButton:
                if(ReadBeacon.equals("red"))
                {
                    bBP.setPosition(rightPosition);
                }
                else
                {
                    bBP.setPosition(leftPosition);
                }
                resetEncoder(motorL);
                resetEncoder(motorR);
                state = States.DriveToBeaconButton;
                break;

            case DriveToBeaconButton:
                DriveToBeaconButton = goToEncoder(-0.1, 0.1, 50, motorL);
                if(!DriveToBeaconButton)
                {
                    break;
                }
                else
                {
                    state = States.Stop;
                    resetEncoder(motorL);
                    resetEncoder(motorR);
                    break;
                }
    
            case Stop:
                motorL.setPower(0.0);
                motorR.setPower(0.0);
        }



        telemetry.addData("Left Motor Encoder", motorL.getCurrentPosition());
        telemetry.addData("Right Motor Encoder", motorR.getCurrentPosition());
        telemetry.addData("Left Color Sensor", lColor);
        telemetry.addData("Right Color Sensor", rColor);
        telemetry.addData("ODS Raw Light Detected", ODS.getRawLightDetected());
        telemetry.addData("Current State", state);
        telemetry.addData("ERROR", ERROR);
    }

    public static void sleep(int time) // In milliseconds
    {
        double constant = System.currentTimeMillis();
        double current = System.currentTimeMillis();
        while ((current - constant) <= time) {
            current = System.currentTimeMillis();
        }
    }
    public boolean goToEncoder(double leftPower, double rightPower, int encPosition, DcMotor encMotor)
    {
        boolean endCondition;
        motorL.setPower(leftPower);
        motorR.setPower(rightPower);

        if(Math.abs(encMotor.getCurrentPosition()) < encPosition)
        {
            endCondition = false;
        }
        else
        {
            motorL.setPower(0.0);
            motorR.setPower(0.0);
            endCondition = true;
        }
        return endCondition;
    }
    public void resetEncoder(DcMotor motor)
    {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public boolean goToColor(double leftPower, double rightPower, float targetColor, int encTimeout, DcMotor encMotor, ColorSensor colorSensor)
    {
        boolean endCondition;
        motorL.setPower(leftPower);
        motorR.setPower(rightPower);

        if((colorSensor.argb() / 1000000) < targetColor)
        {
            endCondition = false;
        }
        else
        {
            motorL.setPower(0.0);
            motorR.setPower(0.0);
            endCondition = true;
        }
        return endCondition;
    }
    public boolean offRight(double maxPower, double medPower, double minPower, float targetColor, double distance)
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
    public boolean offLeft(double maxPower, double medPower, double minPower, float targetColor, double distance)
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
    public boolean lineFollowing(double maxPower, double medPower, double minPower, float targetColor, double distance)
    {
        boolean endCondition;
        if(ODS.getRawLightDetected() < distance)
        {
            endCondition = false;
            /*if(((colorSensorL.argb() / 1000000) <= targetColor) && ((colorSensorR.argb() / 1000000) <= targetColor))
            {
                //Both are black
                motorL.setPower(-medPower);
                motorR.setPower(medPower);
            }
            else if(((colorSensorL.argb() / 1000000) >= targetColor) && ((colorSensorR.argb() / 1000000) <= targetColor))
            {
                //Left = white; Right = black
                motorL.setPower(-maxPower);
                motorR.setPower(minPower);
            }
            else if(((colorSensorL.argb() / 1000000) <= targetColor) && ((colorSensorR.argb() / 1000000) >= targetColor))
            {
                //Left = black; Right = white
                motorL.setPower(-minPower);
                motorR.setPower(maxPower);
            }
            else
            {
                state = States.Stop;
                motorL.setPower(0.0);
                motorR.setPower(0.0);
            }*/
            if((colorSensorL.argb() / 1000000) >= targetColor)
            {
                onLeft = true;
                goToColor(-.15, 0.0, 50, 2000, motorL, colorSensorR);
            }

        }
        else
        {
            motorL.setPower(0.0);
            motorR.setPower(0.0);
            endCondition = true;
        }
        return endCondition;
    }
    public String readBeacon()
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
            for (int x = 0; x < 240; x++)
            {
                for (int y = 0; y < 320; y++)
                {
                    rgbImage.setPixel(x, y, greatestColor(rgbImage.getPixel(x, y)));
                }
            }
            SaveImage(rgbImage);


            //Evaluating left side of screen/beacon
            for (int x = 0; x < 120; x++)
            {
                for (int y = 90; y < 230; y++)
                {
                    int pixelL = rgbImage.getPixel(x, y);
                    redValueLeft += red(pixelL);
                    blueValueLeft += blue(pixelL);
                    greenValueLeft += green(pixelL);
                }
            }

            for (int a = 121; a < 240; a++)
            {
                for (int b = 90; b < 230; b++)
                {
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
