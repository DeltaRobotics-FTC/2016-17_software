package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;

/**
 * Created by RoboticsUser on 11/19/2016.
 */
@Autonomous (name = "I2c_Address_Writing_Test", group = "")
public class I2c_Address_Writing_Test extends OpMode
{

    I2cDevice colorSensorRI2c;
    I2cDevice colorSensorLI2c;
    ColorSensor colorSensorL;
    ColorSensor colorSensorR;



    public void init()
    {
        colorSensorRI2c = hardwareMap.i2cDevice.get("colorSensorR");
        colorSensorLI2c = hardwareMap.i2cDevice.get("colorSensorL");
        colorSensorL = hardwareMap.colorSensor.get("colorSensorL");
        colorSensorR = hardwareMap.colorSensor.get("colorSensorR");



    }
    public void loop()
    {
        telemetry.addData("colorSensroR_IC2_Address",colorSensorR.getI2cAddress());
        telemetry.addData("colorSensroL_IC2_Address",colorSensorL.getI2cAddress());


        if (gamepad1.a) {

        }

        if (gamepad1.b) {
        }
        }


}
