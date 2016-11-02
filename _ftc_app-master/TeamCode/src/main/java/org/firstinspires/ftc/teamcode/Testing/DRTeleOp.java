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
        DcMotor launcherWheel;
        Servo popper;
        Servo bBP;
        float throttleLeft = 0;
        float throttleRight = 0;
        double throttleScalingLeft = 1.0;
        double throttleScalingRight = 1.0;
        double bBPvalue = 0.01;
        boolean guide = false;
        boolean drive = true;
        double popperUp = 0.24;
        double popperDown = 0.07;
        boolean bumper = false;

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
            launcherWheel = hardwareMap.dcMotor.get("launcherWheel");
            popper = hardwareMap.servo.get("popper");
            popper.setPosition(popperDown);
            bBP.setPosition(bBPvalue);

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

            if (gamepad2.right_bumper)
            {
                bumper = true;
            }

            if (gamepad2.left_bumper)
            {
               bumper = false;
            }

            if (bumper)
            {
                launcherWheel.setPower(1.0);
            }

            if (!bumper)
            {
                launcherWheel.setPower(0.0);
            }

            /*if(gamepad2.a){
                popperUp += .005;
            }
            if(gamepad2.y){
                popperUp -= .005;
            }
            */
            if (gamepad2.right_trigger > 0.2)
            {
                popper.setPosition(popperUp);

            }
            if (gamepad2.right_trigger < 0.2)
            {
                popper.setPosition(popperDown);
            }

            // Code inside the loop method is run over and over again when you press the start
            // button. When the opmode ends, this loop stops

            throttleLeft = Range.clip(throttleLeft, -1, 1);
            throttleRight = Range.clip(throttleRight, -1, 1);
            bBPvalue = Range.clip(bBPvalue, .01, .99);
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
                bBPvalue += .005;
            }
            if(gamepad2.x){
                bBPvalue -= .005;
            }

            // Scales the  variable throttleLeft exponentially


            if (guide) {
                throttleLeft = throttleLeft * (float).5;
                throttleRight = throttleRight * (float).5;
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
                motorLF.setPower(throttleRight);
                motorRF.setPower(-throttleLeft);
                motorLB.setPower(throttleRight);
                motorRB.setPower(-throttleLeft);
            }

            bBP.setPosition(bBPvalue);
            //popper.setPosition(popperUp);

            telemetry.addData("bBP", bBPvalue);
            telemetry.addData("Guide", guide);
            telemetry.addData("Drive", drive);
            telemetry.addData("Popper Pos", popper.getPosition());
            // Sets the appropriate motors to the appropriate variables






        }

    }
