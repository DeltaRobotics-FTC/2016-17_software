package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by RoboticsUser on 11/28/2016.
 */
@TeleOp (name="LED_Test", group = "")
public class LED_Test extends OpMode
{
    DcMotorController LEDController1;
    DcMotorController LEDController2;

    public void init()
    {
        LEDController1 = hardwareMap.dcMotorController.get("LEDController1");
        LEDController2 = hardwareMap.dcMotorController.get("LEDController2");

    }

    public void loop()
    {
        if (gamepad1.dpad_up)
        {
            LEDController1.setMotorPower(1, 1.0);
        }

        if (gamepad1.dpad_down)
        {
            LEDController1.setMotorPower(1, 0.0);
        }

        if (gamepad1.a)
        {
            LEDController2.setMotorPower(1, 0.1);
        }

        if (gamepad1.b)
        {
            LEDController1.setMotorPower(2, 0.1);
        }

        if (gamepad1.x)
        {
            LEDController2.setMotorPower(2, 0.1);
        }
    }

}
