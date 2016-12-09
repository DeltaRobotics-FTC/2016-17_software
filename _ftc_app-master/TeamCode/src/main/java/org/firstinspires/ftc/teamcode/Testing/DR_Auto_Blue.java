package org.firstinspires.ftc.teamcode.Testing;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by RoboticsUser on 10/13/2016.
 */

@Autonomous (name = "Auto_Blue", group = "")
public class DR_Auto_Blue extends Camera_Testing {


    I2cDeviceSynch I2C;
    DcMotor motorL;
    DcMotor motorR;
    Servo bBP;
    ColorSensor colorSensorL;
    ColorSensor colorSensorR;
    OpticalDistanceSensor ODS;

    enum States {TURN, DRIVE_FORWARD1, TURN_TO_LINE1, TURN_TO_LINE2, LINE_FOLOWING, CAMERA, PRESS_BUTTON, DRIVE_FORWARD2, STOP, DRIVE_FORWARD3}

    States state;

    float L;
    float R;
    double O;
    final int WhiteMaxValR = 2000;
    final int WhiteMinValR = 50;
    final int WhiteMaxValL = 2000;
    final int WhiteMinValL = 50;


    int ds2 = 2;
    int bBPP;
    double bbpHomeVal = 0.0079;
    int followCount = 100000000;

    public void init() {

        motorL = hardwareMap.dcMotor.get("motorL");
        motorR = hardwareMap.dcMotor.get("motorR");

        bBP = hardwareMap.servo.get("bBP");
        colorSensorR = hardwareMap.colorSensor.get("colorSensorR");
        colorSensorL = hardwareMap.colorSensor.get("colorSensorL");
        ODS = hardwareMap.opticalDistanceSensor.get("ODS");

        //colorSensorL = hardwareMap.colorSensor.get("colorSensorL");
        colorSensorL.setI2cAddress(I2cAddr.create7bit(0x1e)); //0x3c - new, Port 0
        //colorSensorR = hardwareMap.colorSensor.get("colorSensorR");
        colorSensorR.setI2cAddress(I2cAddr.create7bit(0x26)); //0x4c - old, Port 2

        motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        colorSensorL.enableLed(true);
        colorSensorR.enableLed(true);
        bBP.setPosition(bbpHomeVal);

        state = States.TURN;

        setCameraDownsampling(2);
        super.init();

    }
    public void start(){
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
        R = readAvgHue(colorSensorL);
        L = readAvgHue(colorSensorR);
        telemetry.addData("Current State", state);

        switch (state) {

            case TURN:
                if(motorL.getCurrentPosition() < 1175)
                {
                    motorR.setPower(0.25);
                    motorL.setPower(0.0);
                    telemetry.addData("Position L", motorL.getCurrentPosition());
                    telemetry.addData("Position R", motorR.getCurrentPosition());
                }
                else{
                    motorL.setPower(0.0);
                    motorR.setPower(0.0);
                    state = States.DRIVE_FORWARD2;
                    motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                break;

            case DRIVE_FORWARD2:
                colorSensorL.enableLed(true);
                colorSensorR.enableLed(true);
                if ((L < WhiteMaxValL && L > WhiteMinValL) || (R < WhiteMaxValR && R > WhiteMinValR) || motorR.getCurrentPosition() > 3500)
                {

                    motorR.setPower(0.0);
                    motorL.setPower(0.0);
                    state = States.DRIVE_FORWARD3;
                    motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                else
                {
                    motorR.setPower(0.25);
                    motorL.setPower(-0.25);
                    telemetry.addData("Position L", motorL.getCurrentPosition());
                    telemetry.addData("Position R", motorR.getCurrentPosition());
                    telemetry.addData("Left (R)", R);
                    telemetry.addData("Right (L)", L);
                }
                break;


            case DRIVE_FORWARD3:
                if (motorR.getCurrentPosition() > 50)
                {

                    motorR.setPower(0.0);
                    motorL.setPower(0.0);
                    state = States.TURN_TO_LINE2;
                    motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                else
                {
                    motorR.setPower(0.25);
                    motorL.setPower(-0.25);
                    telemetry.addData("Position L", motorL.getCurrentPosition());
                    telemetry.addData("Position R", motorR.getCurrentPosition());
                    telemetry.addData("Left (R)", R);
                    telemetry.addData("Right (L)", L);
                }
                break;

            case TURN_TO_LINE1:
                colorSensorL.enableLed(true);
                colorSensorR.enableLed(true);
                if (L < WhiteMaxValR && L > WhiteMinValR)
                {
                    telemetry.addData("Right (L)", L);
                    telemetry.addData("Alpha", colorSensorR.alpha());

                    motorR.setPower(0.0);
                    motorL.setPower(0.0);
                    sleep(1000);
                    state = States.LINE_FOLOWING;
                }
                else
                {
                    motorR.setPower(0.25);
                    motorL.setPower(0.25);
                    telemetry.addData("Right (L)", L);
                    telemetry.addData("Alpha", colorSensorR.alpha());
                }
            break;

            case TURN_TO_LINE2:
                telemetry.addData("Left (R)", R);
                telemetry.addData("Alpha", colorSensorR.alpha());

                if (L <= WhiteMaxValR && L >= WhiteMinValR)
                {
                    motorR.setPower(0.0);
                    motorL.setPower(0.0);
                    sleep(2000);
                    state = States.LINE_FOLOWING;
                    bBP.setPosition(.66);
                }
                else
                {
                    motorR.setPower(0.25);
                    motorL.setPower(0.20);
                }
                break;

            case LINE_FOLOWING:
                ODS.enableLed(true);


                        if (O < .25) {
                            followCount--;

                            L = readAvgHue(colorSensorL);
                            R = readAvgHue(colorSensorR);
                            telemetry.addData("L", L);
                            telemetry.addData("R", R);
                            O = ODS.getRawLightDetected();
                            //O = readAvgODSVal(ODS);
                            telemetry.addData("rawLightDetected", O);
                            if (L >= WhiteMinValL && L <= WhiteMaxValL) {
                                motorL.setPower(-0.2);
                                motorR.setPower(0.3);
                            } else if (R >= WhiteMinValR && R <= WhiteMaxValR) {
                                motorL.setPower(-0.3);
                                motorR.setPower(0.2);
                            } else {
                                motorL.setPower(-0.2);
                                motorR.setPower(0.2);
                            }
                            telemetry.addData("MotorL", motorL.getPower());
                            telemetry.addData("MotorR", motorR.getPower());


                            }
                        else
                        {
                            motorR.setPower(0.0);
                            motorL.setPower(0.0);
                            state = States.CAMERA;
                        }
                break;

            case CAMERA:
                colorSensorL.enableLed(false);
                colorSensorR.enableLed(false);
                if (imageReady()) {
                    int redValueLeft = -76800;
                    int blueValueLeft = -76800;
                    int greenValueLeft = -76800;
                    int redValueRight = -76800;
                    int blueValueRight = -76800;
                    int greenValueRight = -76800;

                    Bitmap rgbImage;

                    //Put results to phone with red/blue/green
                    rgbImage = convertYuvImageToRgb(yuvImage, width, height, ds2);
                    for (int x = 0; x < 240; x++) {
                        for (int y = 0; y < 320; y++) {
                            rgbImage.setPixel(x, y, greatestColor(rgbImage.getPixel(x, y)));
                        }
                    }
                    SaveImage(rgbImage);


                    //Evaluating left side of screen/beacon
                    for (int x = 0; x < 120; x++) {
                        for (int y = 90; y < 230; y++) {
                            int pixelL = rgbImage.getPixel(x, y);
                            redValueLeft += red(pixelL);
                            blueValueLeft += blue(pixelL);
                            greenValueLeft += green(pixelL);
                        }
                    }

                    //Evaluating right side of screen/beacon
                    for (int a = 121; a < 240; a++) {
                        for (int b = 90; b < 230; b++) {
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

                    if (colorStringLeft.equals("BLUE")) {
                        bBPP = 0;
                        //0 = Left
                        state = States.PRESS_BUTTON;
                        break;
                    }
                    else if (colorStringRight.equals("BLUE")) {
                        bBPP = 1;
                        //1 = Right
                        state = States.PRESS_BUTTON;
                        break;
                    }
                    else {
                        telemetry.addData("Not Reading","Color Value Blue");
                        sleep(1000);
                        break;
                    }
                }
            case PRESS_BUTTON:
                sleep(1000);
                if(bBPP == 1)
                {
                    bBP.setPosition(0.2);
                    //Position for Right Side of Beacon
                }
                else
                {
                    bBP.setPosition(0.8);

                    //Position for Left Side of Beacon
                }
                state = States.STOP;
                break;
            case STOP:
                motorR.setPower(0.0);
                motorL.setPower(0.0);
                break;
        }
        //telemetry.update();
}
    public static void sleep(int amt) // In milliseconds
    {
        double a = System.currentTimeMillis();
        double b = System.currentTimeMillis();
        while ((b - a) <= amt) {
            b = System.currentTimeMillis();
        }
    }
    public float readAvgHue(ColorSensor colorSensor)
    {
        float averagedARGB = 0;
        float argbValToAverage = 0;
        boolean flag = true;
        for (int i = 2; i!= 10; ++i )
        {

            if (flag) {
                float arbg = colorSensor.argb() / 1000000;
                argbValToAverage = arbg + arbg;
            }
            flag = false;

            float arbg = colorSensor.argb() / 1000000;
            argbValToAverage = argbValToAverage + arbg;
        }

        averagedARGB = argbValToAverage / 10;



        return  averagedARGB;
    }
    public double readAvgODSVal(OpticalDistanceSensor ODS)
    {
        double averagedRawLight = 0;
        for (int i = 0; i < 100; ++i )
        {
            averagedRawLight += ODS.getRawLightDetected();
        }

        averagedRawLight /= 100;



        return  averagedRawLight;
    }
}





