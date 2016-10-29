package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by RoboticsUser on 10/15/2016.
 */
@TeleOp (name = "DRTeleOp", group = "")
public class DRTeleOp extends OpMode

    {
        DcMotor motorRF;
        DcMotor motorRB;
        DcMotor motorLF;
        DcMotor motorLB;
        Servo bBP;
        float throttleLeft = 0;
        float throttleRight = 0;
        double throttleScalingLeft = 0.75;
        double throttleScalingRight = 0.75;
        double bBPvalue = 0.5;
        boolean guide = true;
        boolean drive = true;
        // Declares 2 float variables for the throttle


        @Override
        public void init()
        {
            // Code inside the init method is the code that is run when you press the
            //"Init" button on the driver's station
            motorLF = hardwareMap.dcMotor.get("motorLF");
            motorRF = hardwareMap.dcMotor.get("motorRF");
            motorLB = hardwareMap.dcMotor.get("motorLB");
            motorRB = hardwareMap.dcMotor.get("motorRB");
            bBP = hardwareMap.servo.get("bBP");

            // Adds the components you previously initialized to the config
        }


        @Override
        public void loop()
        {
            if(gamepad1.guide) {
                guide = !guide;
            }
            if(gamepad1.dpad_down){
                drive = false;
            }
            if(gamepad1.dpad_up){
                drive = true;
            }
            // Code inside the loop method is run over and over again when you press the start
            // button. When the opmode ends, this loop stops

            throttleLeft = Range.clip(throttleLeft, -1, 1);
            throttleRight = Range.clip(throttleRight, -1, 1);
            // Makes it so the variables can't go below -1 or above 1
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

            if(gamepad2.b){
                bBPvalue =+ .001;
            }
            else if(gamepad2.x){
                bBPvalue =- .001;
            }

            // Scales the  variable throttleLeft exponentially


            if (guide) {
                throttleLeft = throttleLeft * (float).33;
                throttleRight = throttleRight * (float).33;
                throttleLeft = throttleLeft * (float)throttleScalingLeft;
                throttleRight = throttleRight * (float)throttleScalingRight;
            }
            if (!guide){
                throttleLeft = throttleLeft * (float)throttleScalingLeft;
                throttleRight = throttleRight * (float)throttleScalingRight;
            }

            if(drive){
                motorLF.setPower(-throttleLeft);
                motorRF.setPower(throttleRight);
                motorLB.setPower(-throttleLeft);
                motorRB.setPower(throttleRight);
            }
            else{
                motorLF.setPower(throttleLeft);
                motorRF.setPower(-throttleRight);
                motorLB.setPower(throttleLeft);
                motorRB.setPower(-throttleRight);
            }

            bBP.setPosition(bBPvalue);

            telemetry.addData("bBP", bBP);
            telemetry.addData("Guide", guide);
            telemetry.addData("Drive", drive);
            // Sets the appropriate motors to the appropriate variables






        }

    }
