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
        DcMotor motorR;
        DcMotor motorL;
        DcMotor launcherWheel;
        DcMotor collector;
        //Servo popper;
        Servo bBP;
        float throttleLeft = 0;
        float throttleRight = 0;
        double throttleScalingLeft = 1.0;
        double throttleScalingRight = 1.0;
        double bBPvalue = 0.079;
        boolean bBpStop = false;
        boolean drive = true;
        double popperUp = 0.17;
        double popperDown = 0.0;
        boolean bumper = false;

        long constant = 0;
        long current = 0;
        long time = 0;
        int launcherE = 0;
        double rpm = 0;
        boolean collectorVar = false;
        double launcherPower = 0.36;
        boolean dPadLeftState = false;
        boolean dPadRightState = false;


        boolean test = true;


        // Declares 2 float variables for the throttle


        @Override
        public void init()
        {
            // Code inside the init method is the code that is run when you press the
            //"Init" button on the driver's station
            motorL = hardwareMap.dcMotor.get("motorL");
            motorR = hardwareMap.dcMotor.get("motorR");
            bBP = hardwareMap.servo.get("bBP");
            collector = hardwareMap.dcMotor.get("collector");
            launcherWheel = hardwareMap.dcMotor.get("launcherWheel");
            /*
                popper = hardwareMap.servo.get("popper");
                popper.setPosition(popperDown);
            */
            bBP.setPosition(bBPvalue);
            // Adds the components you previously initialized to the config
        }

        @Override
        public void loop() {

                if (gamepad1.dpad_down) {
                    drive = false;
                }
                if (gamepad1.dpad_up) {
                    drive = true;
                }

                if (gamepad2.right_bumper) {
                    bumper = true;
                }

                if (gamepad2.left_bumper) {
                    bumper = false;
                }
            if (gamepad2.dpad_up)
            {
                collectorVar = true;
            }
            if (gamepad2.dpad_down)
            {
                collectorVar = false;
            }
            if (collectorVar)
            {
                collector.setPower(-0.4);
            }
            if (!collectorVar)
            {

                collector.setPower(0);
            }

            if (gamepad2.dpad_left && dPadLeftState == false)
            {
                dPadLeftState = true;
                launcherPower -= 0.05;
            }
            else if (!gamepad2.dpad_left)
            {
                dPadLeftState = false;
            }

            if (gamepad2.dpad_right && dPadRightState == false)
            {
                dPadRightState = true;
                launcherPower += 0.05;
            }
            else if (!gamepad2.dpad_right)
            {
                dPadRightState = false;
            }

                if (bumper) {
                    if (test) {
                        constant = System.currentTimeMillis();
                        test = false;
                    }
                    launcherWheel.setPower(launcherPower);
                }

                if (!bumper) {
                    launcherWheel.setPower(0.0);
                }


               /*
                    if (gamepad2.right_trigger > 0.2) {
                    popper.setPosition(popperUp);
                    }
                    if (gamepad2.right_trigger < 0.2) {
                    popper.setPosition(popperDown);
                    }


                    if (gamepad2.dpad_down) {
                        popperUp -= .05;
                    }
                    if (gamepad2.dpad_up) {
                        popperUp += .05;
                    }

                    Code inside the loop method is run over and over again when you press the start
                    button. When the OpMode ends, this loop stops
                */

                throttleLeft = Range.clip(throttleLeft, -1, 1);
                throttleRight = Range.clip(throttleRight, -1, 1);
                bBPvalue = Range.clip(bBPvalue, .01, .99);
                // Makes it so the variables can't go below -1 or above 1
                // Sends telemetry (a message) to the driver's station that displays the
                // left and right stick Y values and the 2 variables throttleLeft and throttleRight

                //throttleLeft = gamepad1.right_stick_y;
                //throttleRight = gamepad1.left_stick_y;

                if (gamepad1.right_stick_y > 0) {
                    throttleRight = gamepad1.right_stick_y * gamepad1.right_stick_y;
                } else if (gamepad1.right_stick_y < 0) {
                    throttleRight = gamepad1.right_stick_y * gamepad1.right_stick_y * -1;
                } else {
                    throttleRight = 0;
                }
                // Scales the  variable throttleRight exponentially
                if (gamepad1.left_stick_y > 0) {
                    throttleLeft = gamepad1.left_stick_y * gamepad1.left_stick_y;
                } else if (gamepad1.left_stick_y < 0) {
                    throttleLeft = gamepad1.left_stick_y * gamepad1.left_stick_y * -1;
                } else {
                    throttleLeft = 0;
                }

                if (gamepad2.b && bBpStop == false) {
                    bBPvalue += .005;
                }
                if (gamepad2.x && bBpStop == false) {
                    bBPvalue -= .005;
                }
                if (gamepad2.a) {
                    bBpStop = true;
                    bBPvalue = 0.0079;
                    bBP.setPosition(bBPvalue);
                    sleep(500);
                }
                if (!gamepad2.a) {
                    bBpStop = false;
                }
            if (gamepad2.y) {
                bBpStop = true;
                bBPvalue = 0.765;
                bBP.setPosition(bBPvalue);
                sleep(500);
            }
            if (!gamepad2.a) {
                bBpStop = false;
            }


                // Scales the  variable throttleLeft exponentially


                if (gamepad1.b) {
                    throttleScalingLeft = 0.5;
                    throttleScalingRight = 0.5;
                }
                if (gamepad1.a) {
                    throttleScalingLeft = 1;
                    throttleScalingRight = 1;
                }
                throttleLeft = throttleLeft * (float) throttleScalingLeft;
                throttleRight = throttleRight * (float) throttleScalingRight;

                if (drive) {
                    motorL.setPower(-throttleLeft);
                    motorR.setPower(throttleRight);
                } else {
                    motorL.setPower(throttleRight);
                    motorR.setPower(-throttleLeft);
                    }

                bBP.setPosition(bBPvalue);


                telemetry.addData("bBP", bBPvalue);
                telemetry.addData("Drive", drive);
                //telemetry.addData("Popper Pos", popper.getPosition());
                telemetry.addData("Constant", constant);
                telemetry.addData("launcherrPower", launcherPower);
                telemetry.addData("collectorStatus", collectorVar);
                // Sets the appropriate motors to the appropriate variables

            if(System.currentTimeMillis() - constant > 1000) {
                current = System.currentTimeMillis();
                time = current - constant;
                launcherE = motorL.getCurrentPosition();
                telemetry.addData("Launcher Encoder", motorL.getCurrentPosition());
                telemetry.addData("Right Front", motorR);
                time /= 1000;
                //time is in seconds
                launcherE /= 140;
                telemetry.addData("Launcher Rotations", launcherE);
                //launcherE is in rotations
                rpm = 60 * launcherE / time;
                telemetry.addData("RPM", rpm);
            }
        }

        public static void sleep(int amt) // In milliseconds
        {
            double a = System.currentTimeMillis();
            double b = System.currentTimeMillis();
            while ((b - a) <= amt) {
                b = System.currentTimeMillis();
            }
        }


    }
