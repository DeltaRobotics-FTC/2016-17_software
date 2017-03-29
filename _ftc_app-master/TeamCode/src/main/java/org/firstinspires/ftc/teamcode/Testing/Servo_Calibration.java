package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by RoboticsUser on 3/9/2017.
 */
@TeleOp (name = "Servo_Calibration", group = "")
public class Servo_Calibration extends OpMode
{

    Servo popper;
    Servo boot;
    Servo bBP;

    int iter = 0;

    double bBPPos = 0.5;
    double popperPos = 0.7;
    double bootPos = 0.5;
    double servoPos = 0.5;

    boolean bButtonState = false;
    boolean xButtonState = false;

    boolean leftBumperState = false;
    boolean rightBumperState = false;
    // Following variables are used to tell the program when a user presses and releases a button

    Servo servos[] = new Servo[3];
    double servoPoses[] = {bBPPos, popperPos, bootPos};


    public void init()
    {
        bBP = hardwareMap.servo.get("bBP");
        popper = hardwareMap.servo.get("popper");
        boot = hardwareMap.servo.get("boot");
        popper.setPosition(popperPos);
        bBP.setPosition(bBPPos);
        boot.setPosition(bootPos);
    }

    public void loop()
    {


        if (gamepad1.left_bumper && leftBumperState == false)
        {
            leftBumperState = true;
            servoPos -= 0.005;
        }
        else if (!gamepad1.left_bumper)
        {
            leftBumperState = false;
        }

        if (gamepad1.right_bumper && rightBumperState == false)
        {
            rightBumperState = true;
            servoPos += 0.005;
        }
        else if (!gamepad1.right_bumper)
        {
            rightBumperState = false;
        }

        if (gamepad1.x && xButtonState == false)
        {
            xButtonState = true;
            servoPos -= 0.1;
        }
        else if (!gamepad1.x)
        {
            xButtonState = false;
        }

        if (gamepad1.b && bButtonState == false)
        {
            bButtonState = true;
            servoPos += 0.1;
        }
        else if (!gamepad1.b)
        {
            bButtonState = false;
        }

        if (gamepad1.a)
        {
            servoPos = 0.5;
        }

        servoPos = Range.clip(servoPos, 0.01, 0.99);
        bBP.setPosition(servoPos);

        //telemetry.addData("Servo Pos", servo.getPosition());


    }

}
