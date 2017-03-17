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

    Servo servo;
    double servoPos = 0.7;
    boolean bButtonState = false;
    boolean xButtonState = false;

    boolean dpadRightState = false;
    boolean dpadLeftState = false;

    // Following variables are used to tell the program when a user presses and releases a button

    public void init()
    {
        servo = hardwareMap.servo.get("popper");
        // Change the name in .get to calibrate another servo
    }

    public void loop()
    {
        if (gamepad1.dpad_left && dpadLeftState == false)
        {
            dpadLeftState = true;
            servoPos -= 0.005;
        }
        else if (!gamepad1.dpad_left)
        {
            dpadLeftState = false;
        }

        if (gamepad1.dpad_right && dpadRightState == false)
        {
            dpadRightState = true;
            servoPos += 0.005;
        }
        else if (!gamepad1.dpad_right)
        {
            dpadRightState = false;
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
        servo.setPosition(servoPos);

        telemetry.addData("Servo Pos", servo.getPosition());
        telemetry.addData("Servo Pos Var", servoPos);
    }

}
