package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by RoboticsUser on 11/28/2016.
 */
//@TeleOp (name="LED_Test", group = "")
public class LED_Test extends OpMode
{

    DcMotor LEDPower;
    DcMotor redLED;
    DcMotor greenLED;
    DcMotor blueLED;

    boolean LEDState= false;
    double LEDRedValue= 0.0;
    double LEDGreenValue= 0.0;
    double LEDBlueValue= 0.0;
    int sleep= 200;
    //int control= 0;
    //control 0 : b : Red LED
    //control 1 : b : Green LED
    //control 2 : b : Blue LED

    enum states {RED, GREEN, BLUE}
    states control;


    public void init()
    {
        LEDPower = hardwareMap.dcMotor.get("powerLEDPort");
        redLED = hardwareMap.dcMotor.get("redLEDPort");
        greenLED = hardwareMap.dcMotor.get("greenLEDPort");
        blueLED = hardwareMap.dcMotor.get("blueLEDPort");

    }

    public void loop()
    {

       // LEDController2.setMotorPower(1, LEDRedValue);
       // LEDController1.setMotorPower(2, LEDGreenValue);
       // LEDController1.setMotorPower(2, LEDBlueValue);

        if(gamepad1.dpad_up)
        {

            switch (control) {

                case RED:

                    LEDRedValue= LEDRedValue + 0.01;
                    redLED.setPower(LEDRedValue);

                    break;

                case GREEN:



                    break;

                case BLUE:



                    break;

                default:
                    break;

            }

        }




        /*if (gamepad1.right_bumper)
        {
            LEDController1.setMotorPower(1, 1.0);
            LEDState= true;
        }

        if (gamepad1.left_bumper)
        {
            LEDController1.setMotorPower(1, 0.0);
            LEDState= false;
        }

        if (gamepad1.b)
        {
            control= 0;
        }

        if (gamepad1.a)
        {
            control= 1;
        }

        if (gamepad1.x)
        {
            control= 2;
        }*/

        LEDRedValue = Range.clip(LEDRedValue, 0.0, 1.0);
        LEDGreenValue = Range.clip(LEDGreenValue, 0.0, 1.0);
        LEDBlueValue = Range.clip(LEDBlueValue, 0.0, 1.0);

        telemetry.addData("Control", control);
        telemetry.addData("LEDs ON", LEDState);
        telemetry.addData("Red LED", LEDRedValue);
        telemetry.addData("Green LED", LEDGreenValue);
        telemetry.addData("Blue LED", LEDBlueValue);

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
