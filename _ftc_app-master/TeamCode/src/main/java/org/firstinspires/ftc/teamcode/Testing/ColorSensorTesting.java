package org.firstinspires.ftc.teamcode.Testing;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import org.firstinspires.ftc.teamcode.R;

/**
 * Created by RoboticsUser on 10/18/2016.
 */

@Autonomous (name = "ColorSensorTesting", group = "")
public class ColorSensorTesting extends OpMode
{
    ColorSensor colorSensorL;
    ColorSensor colorSensorR;
    boolean flag_left;
    boolean flag_right;

    public void init() {

        colorSensorL = hardwareMap.colorSensor.get("colorSensorL");
        colorSensorL.setI2cAddress(I2cAddr.create7bit(0x1e));
        colorSensorR = hardwareMap.colorSensor.get("colorSensorR");
        colorSensorR.setI2cAddress(I2cAddr.create7bit(0x26));
        flag_left = false;
        flag_right = false;
    }
    public void loop () {

        colorSensorL.enableLed(true);
        colorSensorR.enableLed(true);
        flag_left = true;
        flag_right = true;

        if (flag_left) {
            float hsvLValues[] = {0F, 0F, 0F};
            final float valuesL[] = hsvLValues;
            //final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);

            Color.RGBToHSV(colorSensorL.red() * 8, colorSensorL.green() * 8, colorSensorL.blue() * 8, hsvLValues);

            telemetry.addData("Object Reference L", colorSensorL.toString());
            telemetry.addData("Clear - L", colorSensorL.alpha());
            telemetry.addData("Red - L", colorSensorL.red());
            telemetry.addData("Blue - L", colorSensorL.blue());
            telemetry.addData("Green - L", colorSensorL.green());
            telemetry.addData("Hue - L", hsvLValues[0]);
        }
        if (flag_right) {
            float hsvRValues[] = {0F, 0F, 0F};
            final float valuesR[] = hsvRValues;

            Color.RGBToHSV(colorSensorR.red() * 8, colorSensorR.green() * 8, colorSensorR.blue() * 8, hsvRValues);

            telemetry.addData("Object Reference R", colorSensorR.toString());
            telemetry.addData("Clear - R", colorSensorR.alpha());
            telemetry.addData("Red - R", colorSensorR.red());
            telemetry.addData("Blue - R", colorSensorR.blue());
            telemetry.addData("Green - R", colorSensorR.green());
            telemetry.addData("Hue - R", hsvRValues[0]);
        }
    }
    public void stop () {
        colorSensorL.enableLed(false);
        colorSensorR.enableLed(false);
        }
}
