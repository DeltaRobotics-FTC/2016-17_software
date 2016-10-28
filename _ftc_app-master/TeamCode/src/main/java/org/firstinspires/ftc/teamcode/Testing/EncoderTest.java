package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.ftcrobotcontroller.R;


/**
 * Created by Delta on 10/20/2016.
 */
@Autonomous (name = "Encoder_Test", group = "")
public class EncoderTest extends OpMode
{

    DcMotor motorRF;
    DcMotor motorLF;
    DcMotor motorRB;
    DcMotor motorLB;


    public void init(){

        motorRF = hardwareMap.dcMotor.get("motorRF");
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorRB = hardwareMap.dcMotor.get("motorRB");
        motorLB = hardwareMap.dcMotor.get("motorLB");

        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("", "Stopped Properly");



    }
    public void loop(){

        motorRF.setPower(-.5);
        motorLF.setPower(.5);
        motorRB.setPower(-.5);
        motorLB.setPower(.5);

        telemetry.addData("motorRF Current Pos", motorRF.getCurrentPosition());
        telemetry.addData("motorLF Current Pos", motorLF.getCurrentPosition());
        telemetry.addData("motorRB Current Pos", motorRB.getCurrentPosition());
        telemetry.addData("motorLB Current Pos", motorLB.getCurrentPosition());



    }






    }

