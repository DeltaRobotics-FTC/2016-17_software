package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by RoboticsUser on 3/9/2017.
 */
public class Servo_Calibration extends OpMode
{

    Servo servo;
    double currentServoPos = 0;

    public void init()
    {
        servo = hardwareMap.servo.get("popper");
        // Change the name in .get to calibrate another servo
    }

    public void loop()
    {
        currentServoPos = servo.getPosition();
        telemetry.addData("Servo Pos", servo.getPosition());
        telemetry.addData("Servo Pos Var", currentServoPos);

        if (gamepad1.dpad_up)
        {
            currentServoPos += 0.005;
        }

        if (gamepad1.dpad_down)
        {
            currentServoPos -= 0.005;
        }
    }

}
