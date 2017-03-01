package org.firstinspires.ftc.teamcode.Testing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by RoboticsUser on 11/1/2016.
 */
@Autonomous (name = "Basic_Auto_DontMove", group = "Basic Autonomous")
public class Basic_Auto_NoMove extends OpMode
{
    DcMotor motorL;
    DcMotor motorLF;
    DcMotor motorLaunch;
    DcMotor motorR;
    DcMotor motorRF;

    DcMotor launcherWheel;
    Servo popper;
    DcMotor collector;

    double constant = 0;
    int lastE = 0;
    int encoderCount;
    int cps = 0;
    long a2 = 0;
    long c2 = 0;
    long c = 0;
    long a = 0;
    double popperUp = 0.88;
    double popperDown = 0.7;

    int x = 0;
    int count = 0;
    int rev = 0;
    boolean test = true;
    boolean test1 = true;
    boolean testloop = true;
    boolean test3 = true;


    int avg = 0;
    int p = 5;
    //p-Previous was 20
    int t = 100;
    //t-Previous was 10

    double launcherPower = 0;

    enum states {DRIVE1, STOP, TURN, DRIVE2, SHOOT1, SHOOT2, SHOOT}
    states state;

    public void init() {
        state = states.DRIVE1;
        motorL = hardwareMap.dcMotor.get("motorL");
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorR = hardwareMap.dcMotor.get("motorR");
        motorRF = hardwareMap.dcMotor.get("motorRF");
        launcherWheel = hardwareMap.dcMotor.get("launcherWheel");
        popper = hardwareMap.servo.get("popper");
        collector = hardwareMap.dcMotor.get("collector");
        popper.setPosition(popperDown);

    }
    public void loop()
    {
        if (testloop) {
            constant = System.currentTimeMillis();
            testloop = false;
        }

        if(System.currentTimeMillis() - constant > t)
        {
            rev++; //Making sure that the loop keeps running
            constant = System.currentTimeMillis(); //Resetting current time for calculating time difference
            encoderCount = launcherWheel.getCurrentPosition() - lastE; //Change in encoder counts
            lastE = launcherWheel.getCurrentPosition(); //Resetting the encoder position for calculating difference
            cps = encoderCount * (1000/t);
            //****For Averaging****//
            avg = ((avg * (p - 1) + cps) / p);

        }
        telemetry.addData("Rev", rev);
        telemetry.addData("Encoder Position", launcherWheel.getCurrentPosition());
        telemetry.addData("CPS", cps);
        telemetry.addData("Avg", avg);
        telemetry.addData("Launcher Power", launcherWheel.getPower());
        collector.setPower(.8);

        switch (state)
        {
            case DRIVE1:
                //Changed from 700 for testing
                if (motorL.getCurrentPosition() > 650)
                {
                    // Previous value was -1500
                    motorR.setPower(0.0);
                    motorRF.setPower(0.0);
                    motorL.setPower(0.0);
                    motorLF.setPower(0.0);
                    state = states.SHOOT;
                    motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                else
                {
                    telemetry.addData("Position L", motorL.getCurrentPosition());
                    telemetry.addData("Position R", motorR.getCurrentPosition());
                    motorL.setDirection(DcMotorSimple.Direction.FORWARD);
                    motorR.setDirection(DcMotorSimple.Direction.REVERSE);
                    motorLF.setDirection(DcMotorSimple.Direction.FORWARD);
                    motorRF.setDirection(DcMotorSimple.Direction.REVERSE);
                    motorR.setPower(0.4);
                    motorL.setPower(0.4);
                    motorRF.setPower(0.4);
                    motorLF.setPower(0.4);
                    break;
                }
                break;

            case SHOOT:
            {
                if(test)
                {
                    //c = System.currentTimeMillis();
                    test = false;
                    launcherPower = -.47;
                    launcherWheel.setPower(launcherPower);
                    //avg = launcherWheel.getCurrentPosition();
                }

                /*
                a = System.currentTimeMillis();
                if((a - c) < 2000)
                {
                    break;
                }

                else
                {
                */
                if(count < 2)
                {
                    if (avg < -2150 && avg > -2250)
                    {
                        count++;
                        state = states.SHOOT2;
                    }
                    else
                    {
                        if(test3)
                        {
                            c2 = System.currentTimeMillis();
                            test3 = false;
                        }
                        a2 = System.currentTimeMillis();
                        if((a2 - c2) < t)
                        {
                            break;
                        }
                        else
                        {
                            if (avg < -2250) {
                                launcherPower = launcherPower + .002;
                            }
                            if (avg > -2150) {
                                launcherPower = launcherPower - .002;
                            }
                            launcherWheel.setPower(launcherPower);
                            test3 = true;
                        }

                        break;
                    }

                    test = true;
                }
                else
                {
                    launcherWheel.setPower(0);
                    state = states.STOP;
                    break;
                }
                break;
                //}
            }

            case SHOOT2:
                if(test1)
                {
                    c = System.currentTimeMillis();
                    test1 = false;
                }
                popper.setPosition(popperUp);
                a = System.currentTimeMillis();
                if((a - c) < 1000)
                {
                    break;
                }
                else
                {
                    popper.setPosition(popperDown);
                    //launcherWheel.setPower(0.00);
                    state = states.SHOOT;
                    test = true;
                    test1 = true;
                }
                break;
            case TURN:
                if(motorL.getCurrentPosition() < -50)
                {
                    state = states.DRIVE2;
                    motorR.setPower(0.0);
                    motorL.setPower(0.0);
                    motorRF.setPower(0.0);
                    motorLF.setPower(0.0);
                    motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    break;
                }
                else
                {
                    telemetry.addData("Position L", motorL.getCurrentPosition());
                    telemetry.addData("Position R", motorR.getCurrentPosition());
                    motorL.setDirection(DcMotorSimple.Direction.FORWARD);
                    motorR.setDirection(DcMotorSimple.Direction.REVERSE);
                    motorLF.setDirection(DcMotorSimple.Direction.FORWARD);
                    motorRF.setDirection(DcMotorSimple.Direction.REVERSE);
                    motorR.setPower(0.4);
                    motorL.setPower(-.4);
                    motorRF.setPower(0.4);
                    motorLF.setPower(-.4);
                    break;
                }

            case DRIVE2:
                if (motorL.getCurrentPosition() > 1500)
                {
                    // Previous value was -1500
                    motorR.setPower(0.0);
                    motorL.setPower(0.0);
                    motorRF.setPower(0.0);
                    motorLF.setPower(0.0);
                    state = states.STOP;
                    motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                else
                {
                    telemetry.addData("Position L", motorL.getCurrentPosition());
                    telemetry.addData("Position R", motorR.getCurrentPosition());
                    motorL.setDirection(DcMotorSimple.Direction.FORWARD);
                    motorR.setDirection(DcMotorSimple.Direction.REVERSE);
                    motorLF.setDirection(DcMotorSimple.Direction.FORWARD);
                    motorRF.setDirection(DcMotorSimple.Direction.REVERSE);
                    motorR.setPower(0.4);
                    motorL.setPower(0.4);
                    motorRF.setPower(0.4);
                    motorLF.setPower(0.4);
                    break;
                }
                break;
            case STOP:
                motorR.setPower(0.0);
                motorL.setPower(0.0);
                motorRF.setPower(0.0);
                motorLF.setPower(0.0);
                break;
        }

    }
}
