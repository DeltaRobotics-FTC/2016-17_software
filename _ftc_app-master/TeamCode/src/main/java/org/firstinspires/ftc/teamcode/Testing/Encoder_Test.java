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
@Autonomous (name = "Encoder Test", group = "")
public class Encoder_Test extends OpModeCamera {

    DcMotor motorL;
    DcMotor motorR;
    DcMotor motorLF;
    DcMotor motorRF;

    long current;
    long elapsed;
    long constant;
    String text = "";
    int loop = 0;

    boolean setTime = true;

    int motorLEncoder = 112358;
    File encoderFile;

    public void init()
    {
        motorL = hardwareMap.dcMotor.get("motorL");
        motorR = hardwareMap.dcMotor.get("motorR");
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorRF = hardwareMap.dcMotor.get("motorRF");

        encoderFile = new File(Environment.getExternalStorageDirectory() + File.separator + "FTC Encoder Data" + File.separator);
        encoderFile.mkdirs();

        do
        {

            encoderFile = new File(Environment.getExternalStorageDirectory() + File.separator + "FTC Encoder Data" + File.separator + "log_" + loop + ".txt");
            loop++;


        }while(encoderFile.exists() == true);

    }
    public void loop()
    {

        if(setTime)
        {

            constant = System.currentTimeMillis();
            motorL.setPower(1);
            motorR.setPower(1);
            motorLF.setPower(1);
            motorRF.setPower(1);
            setTime = false;
        }

        motorLEncoder = Math.abs(motorL.getCurrentPosition());

        if (motorLEncoder < 2500)
        {
            TelemetryLog("Encoder Data: ", Integer.toString(motorLEncoder), true, encoderFile);
        }
        else
        {
            motorL.setPower(0);
            motorR.setPower(0);
            motorLF.setPower(0);
            motorRF.setPower(0);
        }
    }
    public void TelemetryLog(String teleCaption, String teleData, boolean log, File f){

        if(log) {
            current = System.currentTimeMillis();
            elapsed = current - constant;

            text = text + teleData + "," + elapsed + "\n";

            try {

                //until you can figure out how to circumvent the usage of MediaScannerConnection.scanFile() this is useless
                FileOutputStream FOS = new FileOutputStream(f);
                FOS.write(text.getBytes());
                FOS.close();

            } catch (IOException e) {

                System.out.println("I don't know how you got here but good job.");

            }
        }
    }

}