package org.firstinspires.ftc.teamcode.Testing;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

import for_camera_opmodes.OpModeCamera;

/**
 * Created by RoboticsUser on 1/26/2017.
 */
@Autonomous (name = "Full_Auto_Test", group = "")
public class Full_Auto_Test extends OpModeCamera {

    DcMotor motorL;
    DcMotor motorR;
    DcMotor motorLF;
    DcMotor motorRF;

    int motorLEncoder = 112358;

    public void init()
    {
        motorL = hardwareMap.dcMotor.get("motorL");
        motorR = hardwareMap.dcMotor.get("motorR");
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorRF = hardwareMap.dcMotor.get("motorRF");
    }
    public void loop()
    {
        if(Math.abs(motorL.getCurrentPosition()) < 1000)
        {
            motorL.setPower(0.75);
            motorLEncoder = motorL.getCurrentPosition();
            TelemetryLog("Encoder Data", Integer.toString(motorLEncoder), true);
        }
    }
    public void TelemetryLog(String teleCaption, String teleData, boolean log){

        if(log) {

            File f = new File(Environment.getExternalStorageDirectory() + File.separator + "log");
            f.mkdirs();

            try {

                //until you can figure out how to circumvent the usage of MediaScannerConnection.scanFile() this is useless
                FileOutputStream FOS = new FileOutputStream(f);
                FOS.write(new String(teleCaption + ":" + teleData + " " + System.nanoTime()).getBytes());
                FOS.close();

            } catch (IOException e) {

                System.out.println("I don't know how you got here but good job.");

            }
        }
    }

}