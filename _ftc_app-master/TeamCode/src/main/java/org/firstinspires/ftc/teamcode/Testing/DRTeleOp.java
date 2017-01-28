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
        DcMotor motorRF;
        DcMotor motorL;
        DcMotor motorLF;
        DcMotor launcherWheel;
        DcMotor motorLift;
        DcMotor collector;
        Servo popper;
        Servo bBP;

        float throttleLeft = 0;
        float throttleRight = 0;
        double throttleScalingLeft = 1.0;
        double throttleScalingRight = 1.0;
        double bBPvalue = 0.079;
        double popperUp = 0.99;
        double popperDown = 0.8;
        double popperPosition = 0.8;

        boolean bBpStop = false;
        boolean drive = true;
        boolean spinning = false;
        boolean collecting = false;
        double constant = 0;
        boolean collectback = false;
        int lastE = 0;
        int encoderCount;
        int cps = 0;
        int rev = 0;
        long timeConstant;
        double launcherPower = -0.36;
        boolean dPadLeftState = false;
        boolean dPadRightState = false;
        boolean test = true;
        boolean spinStart = false;
        boolean popperTime;
        boolean autoAdjust = true;
        long popperStart;

        int a = 0;
        int p = 10;

        // Declares 2 float variables for the throttle

        @Override
        public void init()
        {
            // Code inside the init method is the code that is run when you press the
            //"Init" button on the driver's station
            motorL = hardwareMap.dcMotor.get("motorL");
            motorR = hardwareMap.dcMotor.get("motorR");
            motorRF = hardwareMap.dcMotor.get("motorRF");
            motorLF = hardwareMap.dcMotor.get("motorLF");
            bBP = hardwareMap.servo.get("bBP");
            motorLift = hardwareMap.dcMotor.get("motorLift");
            launcherWheel = hardwareMap.dcMotor.get("launcherWheel");
            collector = hardwareMap.dcMotor.get("collector");
            popper = hardwareMap.servo.get("popper");
            popper.setPosition(popperDown);

            bBP.setPosition(bBPvalue);
            // Adds the components you previously initialized to the config
        }

        @Override
        public void loop() {

            if(spinStart)
            {
                a = launcherWheel.getCurrentPosition();
                spinStart = false;
                timeConstant = System.currentTimeMillis();
            }
            telemetry.addData("cps", cps);
            telemetry.addData("a", a);
            telemetry.addData("AutoAdjust", autoAdjust);
            if(System.currentTimeMillis() - timeConstant > 50)
            {
                if (spinning) {
                    timeConstant = System.currentTimeMillis();
                    encoderCount = launcherWheel.getCurrentPosition() - lastE; //Change in encoder counts
                    lastE = launcherWheel.getCurrentPosition(); //Resetting the encoder position for calculating difference
                    //****For Not Averaging****//
                    cps = encoderCount * 20;
                    //****For Averaging****//
                    a = ((a * (p - 1) + cps) / p);
                    if (a < -1800 && autoAdjust)
                    {
                        if (a > -2000) {
                            launcherPower -= .002;
                        }
                        if (a < -2075) {
                            launcherPower += .002;
                        }
                    }
                }
            }

            if(gamepad1.dpad_right)
            {
                collectback = true;
            }

            if(gamepad1.dpad_left)
            {
                collectback = false;
            }

            if (gamepad1.dpad_down)
            {
                drive = false;
            }

            if (gamepad1.dpad_up)
            {
                drive = true;
            }

            if (gamepad2.right_bumper)
            {
                spinning = true;
            }

            if (gamepad2.left_bumper)
            {
                spinning = false;
            }

            if (gamepad1.right_trigger > 0.75)
            {
              collecting = true;
            }

            if (gamepad1.left_trigger > 0.75)
            {
                collecting = false;
            }


            //Adjusting the Launcher Power - incrementing by 5%
            if (gamepad2.dpad_left && !dPadLeftState)
            {
                dPadLeftState = true;
                launcherPower -= 0.005;
            }
            else if (!gamepad2.dpad_left)
            {
                dPadLeftState = false;
            }

            if (gamepad2.dpad_right && !dPadRightState)
            {
                dPadRightState = true;
                launcherPower += 0.005;
            }
            else if (!gamepad2.dpad_right)
            {
                dPadRightState = false;
            }


            //Beacon Button Presser - incrementing by .005
            if (gamepad2.b && !bBpStop)
            {
                bBPvalue += .005;
            }
            if (gamepad2.x && !bBpStop)
            {
                bBPvalue -= .005;
            }
            if (gamepad2.a)
            {
                bBpStop = true;
                bBPvalue = 0.0079;
                bBP.setPosition(bBPvalue);
                sleep(500);
            }
            if (!gamepad2.a)
            {
                bBpStop = false;
            }
            if (gamepad2.y)
            {
                bBpStop = true;
                bBPvalue = 0.765;
                bBP.setPosition(bBPvalue);
                sleep(500);
            }
            if (!gamepad2.a) {
                bBpStop = false;
            }

            //Setting the Launcher Power
            if (spinning)
            {
                launcherWheel.setPower(launcherPower);
            }

            if (!spinning)
            {
                launcherWheel.setPower(0);
            }

            //Setting the Popper Position
            if (gamepad2.right_trigger > 0.8 && a < -2000 && a > -2075 && cps > -2100)
            {

                popperTime = true;
                popperPosition = popperUp;
                popperStart = System.currentTimeMillis();
            }
            if (gamepad2.left_trigger > 0.8 )
            {
                popperPosition = popperDown;
            }

            if(popperTime)
            {
                if(System.currentTimeMillis() - popperStart > 200)
                {
                    popperPosition = popperDown;
                    popperTime = false;
                }
            }


            if(gamepad2.dpad_up)
            {
                autoAdjust = true;
            }

            if(gamepad2.dpad_down)
            {
                autoAdjust = false;
            }
            popper.setPosition(popperPosition);


            //Setting the Collector and the Lift Motor
            if(collecting)
            {
                collector.setPower(-0.8);
                motorLift.setPower(-1.0);
            }
            else if(collectback)
            {
                collector.setPower(.8);
                motorLift.setPower(1.0);
            }
            else
            {
                collector.setPower(0.0);
                motorLift.setPower(0.0);
            }

            /*if (gamepad2.back && backState == false)
            {
                backState = true;
                collectorPower -= 0.05;
            }
            else if (!gamepad2.back)
            {
                backState = false;
            }

            /*if (gamepad2.start && startState == false)
            {
                startState = true;
                collectorPower += 0.05;
            }
            else if (!gamepad2.start)
            {
                startState = false;
            }*/


            throttleLeft = Range.clip(throttleLeft, -1, 1);
            throttleRight = Range.clip(throttleRight, -1, 1);
            bBPvalue = Range.clip(bBPvalue, .01, .99);
            // Makes it so the variables can't go below -1 or above 1

            //Scales the motors exponentially
            if (gamepad1.right_stick_y > 0)
            {
                throttleRight = gamepad1.right_stick_y  *  gamepad1.right_stick_y;
            }
            else if (gamepad1.right_stick_y < 0)
            {
                throttleRight = gamepad1.right_stick_y;// * gamepad1.right_stick_y * -1;
            }
            else
            {
                throttleRight = 0;
            }

            if (gamepad1.left_stick_y > 0)
            {
                throttleLeft = gamepad1.left_stick_y;// * gamepad1.left_stick_y;
            }
            else if (gamepad1.left_stick_y < 0)
            {
                throttleLeft = gamepad1.left_stick_y;// * gamepad1.left_stick_y * -1;
            }
            else
            {
                throttleLeft = 0;
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
                    motorLF.setPower(-throttleLeft);
                    motorR.setPower(throttleRight);
                    motorRF.setPower(throttleRight);
                } else {
                    motorL.setPower(throttleRight);
                    motorLF.setPower(throttleRight);
                    motorR.setPower(-throttleLeft);
                    motorRF.setPower(-throttleLeft);
                    }

            bBP.setPosition(bBPvalue);

            //telemetry.addData("bBP Position", bBPvalue);
            //telemetry.addData("Popper Position", popperPosition);
            //telemetry.addData("Collector Power", collector.getPower());
            telemetry.addData("Launcher Wheel (the motor)", launcherWheel.getPower());
            telemetry.addData("Launcher Power (the variable)", launcherPower);
            telemetry.addData("Drive", drive);
            //telemetry.addData("Lift Power", motorLift.getPower());

                // Sets the appropriate motors to the appropriate variables

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
