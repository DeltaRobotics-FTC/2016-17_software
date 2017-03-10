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
    //DcMotor collector;
    ColorSensor colorSensor;

    double collectorPower = 0.50;

    public void init()
    {
        //collector = hardwareMap.dcMotor.get("collector");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        colorSensor.enableLed(true);

    }

    public void loop()
    {
        telemetry.addData("Blue Value", colorSensor.argb());
        telemetry.addData("Red Value", colorSensor.argb());
    }

}
