package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by RoboticsUser on 9/20/2016.
 */
@TeleOp (name = "DemoBot", group = "")
public class DemoBot extends OpMode
{
    DcMotor motorLeftFront;
    DcMotor motorRightFront;
    DcMotor motorLeftBack;
    DcMotor motorRightBack;

    float throttleLeft = 0;
    float throttleRight = 0;


    @Override
    public void init()
    {
        motorLeftFront = hardwareMap.dcMotor.get("motorLeftFront");
        motorRightFront = hardwareMap.dcMotor.get("motorRightFront");
        motorLeftBack = hardwareMap.dcMotor.get("motorLeftBack");
        motorRightBack = hardwareMap.dcMotor.get("motorRightBack");


    }


    @Override
    public void loop()
    {

        throttleLeft = Range.clip(throttleLeft, -1, 1);
        throttleRight = Range.clip(throttleRight, -1, 1);
        telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
        telemetry.addData("Right Stick Y", gamepad1.right_stick_y);
        telemetry.addData("throttleRight", throttleRight);
        telemetry.addData("throttleLeft", throttleLeft);

        //throttleLeft = gamepad1.right_stick_y;
        //throttleRight = gamepad1.left_stick_y;

        if (gamepad1.right_stick_y > 0){
             throttleRight = gamepad1.right_stick_y * gamepad1.right_stick_y;
        } else if (gamepad1.right_stick_y < 0){
            throttleRight = gamepad1.right_stick_y * gamepad1.right_stick_y * -1;
        }
        else
            {
                throttleRight = 0;
            }
        if (gamepad1.left_stick_y > 0){
            throttleLeft = gamepad1.left_stick_y * gamepad1.left_stick_y;
        } else if (gamepad1.left_stick_y < 0){
            throttleLeft = gamepad1.left_stick_y * gamepad1.left_stick_y * -1;
        }
        else
        {
            throttleLeft = 0;
        }




        motorLeftFront.setPower(throttleRight);
        motorRightFront.setPower(-throttleLeft);
        motorLeftBack.setPower(throttleRight);
        motorRightBack.setPower(-throttleLeft);






    }

}
