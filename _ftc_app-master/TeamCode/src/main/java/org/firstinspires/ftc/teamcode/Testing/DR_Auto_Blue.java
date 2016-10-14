package org.firstinspires.ftc.teamcode.Testing;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by RoboticsUser on 10/13/2016.
 */
public class DR_Auto_Blue extends Camera_Testing {

    DcMotor motorLF;
    DcMotor motorLB;
    DcMotor motorRF;
    DcMotor motorRB;
    Servo bBP;
    ColorSensor colorSensorL;
    ColorSensor colorSensorR;
    OpticalDistanceSensor ODS;

    enum States {INIT_MOTORS, DRIVE_FORWARD1, TURN_TO_LINE1, TURN_TO_LINE2, LINE_FOLOWING, CAMERA, PRESS_BUTTON}

    States state;
    float L;
    float R;
    double O;
    private int ds2 = 2;
    int bBPP;

    public void init() {

        motorLB = hardwareMap.dcMotor.get("motorLB");
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorRB = hardwareMap.dcMotor.get("motorRB");
        motorRF = hardwareMap.dcMotor.get("motorRF");
        bBP = hardwareMap.servo.get("bBP");
        colorSensorR = hardwareMap.colorSensor.get("colorSensorR");
        colorSensorL = hardwareMap.colorSensor.get("colorSensorL");
        ODS = hardwareMap.opticalDistanceSensor.get("ODS");

        setCameraDownsampling(2);
        super.init();

    }

    public void loop()
    {

        L = colorSensorL.argb() / 1000000;
        R = colorSensorR.argb() / 1000000;
        O = ODS.getRawLightDetected();

        switch (state) {

            case INIT_MOTORS:
                motorRF.setPower(1.0);
                motorRB.setPower(1.0);
                motorLF.setPower(1.0);
                motorLB.setPower(1.0);
                state = States.DRIVE_FORWARD1;
                break;

            case DRIVE_FORWARD1:
                if (((motorLF.getCurrentPosition() + motorRF.getCurrentPosition()) / 2) > 3000)
                {

                    motorRF.setPower(0.0);
                    motorRB.setPower(0.0);
                    motorLF.setPower(0.0);
                    motorLB.setPower(0.0);
                    state = States.TURN_TO_LINE1;
                    break;


                }
                else
                {

                    break;

                }

            case TURN_TO_LINE1:
                if (R >= 240 && R <= 80)
                {
                    motorRB.setPower(0.0);
                    motorRF.setPower(0.0);
                    motorLB.setPower(0.0);
                    motorLF.setPower(0.0);
                    state = States.TURN_TO_LINE2;
                    break;

                }
                else
                {
                    motorLB.setPower(0.5);
                    motorLF.setPower(0.5);
                    break;
                }

            case TURN_TO_LINE2:
                if (L >= 240 && L <= 80)
                {
                    motorRB.setPower(0.0);
                    motorRF.setPower(0.0);
                    motorLB.setPower(0.0);
                    motorLF.setPower(0.0);
                    state = States.LINE_FOLOWING;
                    break;
                }
                else
                {
                    motorLB.setPower(0.25);
                    motorLF.setPower(0.25);
                    break;
                }

            case LINE_FOLOWING:
                while (O <= 5) {
                    L = colorSensorL.argb() / 1000000;
                    R = colorSensorR.argb() / 1000000;
                    O = ODS.getRawLightDetected();
                    if (L >= 80 && L <= 260) {
                        motorLF.setPower(0.4);
                        motorLB.setPower(0.4);
                        motorRF.setPower(0.5);
                        motorRB.setPower(0.5);

                    } else if (R >= 80 && L <= 260) {
                        motorLF.setPower(0.5);
                        motorLB.setPower(0.5);
                        motorRF.setPower(0.4);
                        motorRB.setPower(0.4);

                    } else {
                        motorLF.setPower(0.4);
                        motorLB.setPower(0.4);
                        motorRF.setPower(0.4);
                        motorRB.setPower(0.4);
                    }
                }
                motorLF.setPower(0.0);
                motorLB.setPower(0.0);
                motorRF.setPower(0.0);
                motorRB.setPower(0.0);
                state = States.CAMERA;
                break;

            case CAMERA:

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

                    if (colorStringLeft == "BLUE") {
                        bBPP = 0;
                        //0 = Left
                        state = States.PRESS_BUTTON;
                        break;
                    }
                    else if (colorStringRight == "BLUE") {
                        bBPP = 1;
                        //1 = Right
                        state = States.PRESS_BUTTON;
                        break;
                    }
                    else {
                        sleep(1000);
                        break;
                    }
                }
            case PRESS_BUTTON:
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
        }

}
    public static void sleep(int amt) // In milliseconds
    {
        double a = System.currentTimeMillis();
        double b = System.currentTimeMillis();
        while ((b - a) <= amt) {
            b = System.currentTimeMillis();
        }
    }

}





