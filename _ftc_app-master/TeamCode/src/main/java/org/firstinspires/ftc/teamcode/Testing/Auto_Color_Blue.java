package org.firstinspires.ftc.teamcode.Testing;

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

    float lColor;
    float rColor;

    boolean DriveToWhiteLine = false;
    boolean DrivePastWhiteLine = false;
    boolean TurnOntoWhiteLine = false;

    enum States {DriveToWhiteLine, DrivePastWhiteLine, TurnOntoWhiteLine, LineFollowing}
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

        bBP.setPosition(0.0);

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
                    state = States.LineFollowing;
                    resetEncoder(motorL);
                    resetEncoder(motorR);
                    break;
                }
        }



        telemetry.addData("Left Motor Encoder", motorL.getCurrentPosition());
        telemetry.addData("Right Motor Encoder", motorR.getCurrentPosition());
        telemetry.addData("Left Color Sensor", lColor);
        telemetry.addData("Right Color Sensor", rColor);
        telemetry.addData("Current State", state);
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
}
