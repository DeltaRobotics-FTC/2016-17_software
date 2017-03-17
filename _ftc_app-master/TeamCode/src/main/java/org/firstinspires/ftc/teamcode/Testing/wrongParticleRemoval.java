package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by RoboticsUser on 3/4/2017.
 */
@TeleOp (name = "wrongParticleRemoval", group = "")
public class wrongParticleRemoval extends OpMode
{
    DcMotor collector;
    ColorSensor colorSensor;

    double collectorPower = -0.50;
    String side;

    public void init()
    {
        collector = hardwareMap.dcMotor.get("collector");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        colorSensor.enableLed(true);

    }

    public void init_loop()
    {
        if (gamepad1.x)
        {
            side = "blue";
        }

        if (gamepad1.b)
        {
            side = "red";
        }

        telemetry.addData("Side", side);
    }

    public void loop()
    {
        if (side == "blue")
        {
            telemetry.addData("Red Value", colorSensor.red());

            if (colorSensor.red() > 5)
            {
                telemetry.addData("", "Reject Red Particle");
                collector.setPower(collectorPower);
            }
        }

        if (side == "red")
        {
            telemetry.addData("Blue Value", colorSensor.blue());

            if (colorSensor.blue() > 5)
            {
                telemetry.addData("", "Reject Blue Particle");
                collector.setPower(collectorPower);
            }
        }

    }

}
