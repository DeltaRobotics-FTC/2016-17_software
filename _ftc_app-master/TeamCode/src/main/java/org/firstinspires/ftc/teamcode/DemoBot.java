package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
// Imports files for all of the necessary components


import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by RoboticsUser on 9/20/2016.
 */
@TeleOp (name = "DemoBot", group = "")
// Delcares that this OpMode is a TeleOp OpMode
public class DemoBot extends OpMode
{
    DcMotor motorLeftFront;
    DcMotor motorRightFront;
    DcMotor motorLeftBack;
    DcMotor motorRightBack;
    // Declares variables for each hardware component

    float throttleLeft = 0;
    float throttleRight = 0;
    double throttleScalingLeft = 0.70;
    double throttleScalingRight = 0.70;
    // Declares 2 float variables for the throttle


    @Override
    public void init()
    {
        // Code inside the init method is the code that is run when you press the
        //"Init" button on the driver's station
        motorLeftFront = hardwareMap.dcMotor.get("motorLeftFront");
        motorRightFront = hardwareMap.dcMotor.get("motorRightFront");
        motorLeftBack = hardwareMap.dcMotor.get("motorLeftBack");
        motorRightBack = hardwareMap.dcMotor.get("motorRightBack");

        // Adds the components you previously initialized to the config
    }


    @Override
    public void loop()
    {
        // Code inside the loop method is run over and over again when you press the start
        // button. When the opmode ends, this loop stops

        throttleLeft = Range.clip(throttleLeft, -1, 1);
        throttleRight = Range.clip(throttleRight, -1, 1);
        // Makes it so the variables can't go below -1 or above 1
        telemetry.addData("Left Stick Y", gamepad1.left_stick_y);
        telemetry.addData("Right Stick Y", gamepad1.right_stick_y);
        telemetry.addData("throttleRight", throttleRight);
        telemetry.addData("throttleLeft", throttleLeft);
        // Sends telemetry (a message) to the driver's station that displays the
        // left and right stick Y values and the 2 variables throttleLeft and throttleRight

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
        // Scales the  variable throttleRight exponentially
        if (gamepad1.left_stick_y > 0){
            throttleLeft = gamepad1.left_stick_y * gamepad1.left_stick_y;
        } else if (gamepad1.left_stick_y < 0){
            throttleLeft = gamepad1.left_stick_y * gamepad1.left_stick_y * -1;
        }
        else
        {
            throttleLeft = 0;
        }

        // Scales the  variable throttleLeft exponentially

        throttleLeft = throttleLeft * (float)throttleScalingLeft;
        throttleRight = throttleRight * (float)throttleScalingRight;


        motorLeftFront.setPower(throttleRight);
        motorRightFront.setPower(-throttleLeft);
        motorLeftBack.setPower(throttleRight);
        motorRightBack.setPower(-throttleLeft);
        // Sets the appropriate motors to the appropriate variables






    }

}
