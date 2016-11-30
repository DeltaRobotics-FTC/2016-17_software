package org.firstinspires.ftc.teamcode.Testing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by RoboticsUser on 11/1/2016.
 */
@Autonomous (name = "DR_Auto_New_Launcher", group = "")
public class DR_Auto_New_Launcher extends OpMode
{
    DcMotor motorLF;
    DcMotor motorLB;
    DcMotor motorRF;
    DcMotor motorRB;
    DcMotor collector;
    DcMotor launcherWheel;
    Servo popper;

    double constant = 0;
    int lastE = 0;
    int encoderCount;
    int cps = 0;
    long a2 = 0;
    long c2 = 0;
    long c = 0;
    long a = 0;
    double popperUp = 0.14 ;
    double popperDown = 0.0;

    int x = 0;
    int count = 1;
    int rev = 0;
    boolean test = true;
    boolean test1 = true;
    boolean testloop = true;
    boolean test3 = true;
    boolean test4 = true;

    double launcherPower = 0;
    double collectorPower = -0.4;

    enum states {STOP, DRIVE, DRIVE2, SHOOT2, SHOOT}
    states state;

    public void init() {
        state = states.DRIVE;
        motorLB = hardwareMap.dcMotor.get("motorLB");
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorRB = hardwareMap.dcMotor.get("motorRB");
        motorRF = hardwareMap.dcMotor.get("motorRF");
        collector = hardwareMap.dcMotor.get("collector");
        launcherWheel = hardwareMap.dcMotor.get("launcherWheel");
        popper = hardwareMap.servo.get("popper");
        popper.setPosition(popperDown);

    }
    public void loop()
    {
        if (testloop) {
            constant = System.currentTimeMillis();
            testloop = false;
        }
        if(System.currentTimeMillis() - constant > 100)
        {
            rev++;
            constant = System.currentTimeMillis();
            encoderCount = motorLF.getCurrentPosition() - lastE;
            lastE = motorLF.getCurrentPosition();
            cps = encoderCount * 10;

        }
        telemetry.addData("Rev", rev);
        telemetry.addData("Encoder Position", motorLF.getCurrentPosition());
        telemetry.addData("CPS", cps);
        telemetry.addData("Launcher Power", launcherWheel.getPower());


        switch (state)
        {
            case DRIVE:
                if (motorLB.getCurrentPosition() > 1000)
                {
                    // Previous value was -1500
                    motorRF.setPower(0.0);
                    motorRB.setPower(0.0);
                    motorLF.setPower(0.0);
                    motorLB.setPower(0.0);
                    state = states.SHOOT;
                    motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    resetEncoder(collector);
                }
                else
                {
                    telemetry.addData("Position LF", motorLF.getCurrentPosition());
                    telemetry.addData("Position LB", motorLB.getCurrentPosition());
                    telemetry.addData("Position RF", motorRF.getCurrentPosition());
                    telemetry.addData("Position RB", motorRB.getCurrentPosition());
                    motorLB.setDirection(DcMotorSimple.Direction.FORWARD);
                    motorRB.setDirection(DcMotorSimple.Direction.REVERSE);
                    motorRF.setPower(-0.4);
                    motorRB.setPower(0.4);
                    motorLF.setPower(0.4);
                    motorLB.setPower(0.4);
                    break;
                }
                break;



            case SHOOT:
            {
                if(test)
                {
                    c = System.currentTimeMillis();
                    test = false;
                    launcherPower = .4;
                    launcherWheel.setPower(launcherPower);
                }
                a = System.currentTimeMillis();
                if((a - c) < 2000)
                {
                    break;
                }
                else
                {
                    if(count < 2)
                    {
                        if (cps > 2000 && cps < 2100) {
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
                            if((a2 - c2) < 500)
                            {
                                break;
                            }
                            else
                            {
                                if (cps > 2100) {
                                    launcherPower = launcherPower - .01;
                                }
                                if (cps < 2000) {
                                    launcherPower = launcherPower + .01;
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
                        collector.setPower(0.0);
                        state = states.DRIVE2;
                        break;
                    }
                    break;
                }
            }

            case SHOOT2:
                if(test1)
                {
                    c = System.currentTimeMillis();
                    test1 = false;
                }
                    if (test4)
                    {
                        collector.setPower(collectorPower);
                        a = System.currentTimeMillis();
                        if((a - c) < 500)
                        {
                            break;
                        }
                        collector.setPower(0);
                        resetEncoder(collector);
                        test4 = false;
                        a = System.currentTimeMillis();
                        if((a - c) < 1000)
                        {
                            break;
                        }
                    }

                if (test4 == false)
                {
                    collector.setPower(collectorPower);
                    a = System.currentTimeMillis();
                    if((a - c) < 2000)
                    {
                        break;
                    }
                    collector.setPower(0);
                    resetEncoder(collector);
                }

                a = System.currentTimeMillis();
                if((a - c) < 3000)
                {
                    break;
                }
                else
                {
                    a = System.currentTimeMillis();
                    if((a - c) < 1000)
                    {
                        break;
                    }
                    state = states.SHOOT;
                    test = true;
                    test1 = true;
                }
                break;

            case DRIVE2:
                sleep(1500);
                if (motorLB.getCurrentPosition() > 1000)
                {
                    // Previous value was -1500
                    motorRF.setPower(0.0);
                    motorRB.setPower(0.0);
                    motorLF.setPower(0.0);
                    motorLB.setPower(0.0);
                    state = states.STOP;
                    motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                else
                {
                    telemetry.addData("Position LF", motorLF.getCurrentPosition());
                    telemetry.addData("Position LB", motorLB.getCurrentPosition());
                    telemetry.addData("Position RF", motorRF.getCurrentPosition());
                    telemetry.addData("Position RB", motorRB.getCurrentPosition());
                    motorLB.setDirection(DcMotorSimple.Direction.FORWARD);
                    motorRB.setDirection(DcMotorSimple.Direction.REVERSE);
                    motorRF.setPower(-0.4);
                    motorRB.setPower(0.4);
                    motorLF.setPower(0.4);
                    motorLB.setPower(0.4);
                    break;
                }
                break;



            case STOP:
                motorRF.setPower(0.0);
                motorRB.setPower(0.0);
                motorLF.setPower(0.0);
                motorLB.setPower(0.0);
                collector.setPower(0.0);
                break;
        }

        telemetry.addData("currentState", state);
        telemetry.addData("collectorCurrentPos", collector.getCurrentPosition());


    }
    public static void resetEncoder(DcMotor TheMotor)
    {

        TheMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TheMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
