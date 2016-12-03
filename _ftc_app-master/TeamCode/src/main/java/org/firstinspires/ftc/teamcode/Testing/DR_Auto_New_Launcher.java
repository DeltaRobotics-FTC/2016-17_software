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

    int lastE = 0;
    int encoderCount;
    int cps = 0;
    long updatingCurrentT2 = 0;
    // Second version of updatingCurrentT
    long currentT2 = 0;
    // Second version of currentT
    long currentT = 0;
    // current time in millis read
    long updatingCurrentT = 0;
    // Current time in millis read that continues to read
    /*
        double popperUp = 0.14 ;
        double popperDown = 0.0;
    */

    int count = 0;
    int rev = 0;
    boolean runOnce1 = true;
    boolean runOnce2 = true;
    boolean runOnce3 = true;
    boolean runOnce4 = true;
    boolean runOnce5 = true;
    boolean runOnce6 = true;
    boolean runOnce7 = true;
    boolean runOnce8 = true;
    boolean runOnce9 = true;


    double launcherPower = 0.4;
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
        resetEncoder(motorLB);
        resetEncoder(motorRB);
        resetEncoder(collector);

    }
    public void loop() {
        if (runOnce1) {
            currentT = System.currentTimeMillis();
            runOnce1 = false;
        }
        if (System.currentTimeMillis() - currentT > 100) {
            rev++;
            currentT = System.currentTimeMillis();
            encoderCount = motorLF.getCurrentPosition() - lastE;
            lastE = motorLF.getCurrentPosition();
            cps = encoderCount * 10;

        }
        telemetry.addData("Rev", rev);
        telemetry.addData("Encoder Position", motorLF.getCurrentPosition());
        telemetry.addData("CPS", cps);
        telemetry.addData("Launcher Power", launcherWheel.getPower());


        switch (state) {
            case DRIVE:
                if (motorLB.getCurrentPosition() > 1000) {
                    // Previous value was -1500
                    stopMotors(motorLB, motorLF, motorRB, motorRF);
                    state = states.SHOOT;
                    resetEncoder(motorLB);
                    resetEncoder(motorRB);
                    resetEncoder(collector);
                } else {
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


            case SHOOT: {
                if (runOnce2) {
                    currentT = System.currentTimeMillis();
                    runOnce2 = false;
                    launcherWheel.setPower(launcherPower);
                }
                updatingCurrentT = System.currentTimeMillis();
                if ((updatingCurrentT - currentT) < 2000) {
                    break;
                } else {
                    if (count < 2) {
                        if (cps > 2000 && cps < 2100) {
                            count++;
                            state = states.SHOOT2;
                        } else {
                            if (runOnce3) {
                                currentT2 = System.currentTimeMillis();
                                runOnce3 = false;
                            }
                            updatingCurrentT2 = System.currentTimeMillis();
                            if ((updatingCurrentT2 - currentT2) < 500) {
                                break;
                            } else {
                                if (cps > 2100) {
                                    launcherPower = launcherPower - .01;
                                }
                                if (cps < 2000) {
                                    launcherPower = launcherPower + .01;
                                }
                                launcherWheel.setPower(launcherPower);
                                runOnce3 = true;
                            }

                            break;
                        }

                        runOnce1 = true;
                    } else {
                        launcherWheel.setPower(0);
                        collector.setPower(0.0);
                        state = states.DRIVE2;
                        break;
                    }
                    break;
                }
            }

            case SHOOT2:
                if (runOnce4) {
                    currentT = System.currentTimeMillis();
                    runOnce4 = false;
                }
                if (runOnce5) {
                    collector.setPower(collectorPower);
                    updatingCurrentT = System.currentTimeMillis();
                    if ((updatingCurrentT - currentT) < 500) {
                        break;
                    }
                    collector.setPower(0);
                    resetEncoder(collector);
                    runOnce5 = false;
                    if (runOnce6) {
                        currentT = System.currentTimeMillis();
                        runOnce6 = false;
                    }
                    updatingCurrentT = System.currentTimeMillis();
                    if ((updatingCurrentT - currentT) < 1000) {
                        break;

                    }
                }

                if (runOnce5 == false) {
                    collector.setPower(collectorPower);
                    updatingCurrentT = System.currentTimeMillis();
                    if (runOnce7) {
                        currentT = System.currentTimeMillis();
                        runOnce7 = false;
                    }
                    if ((updatingCurrentT - currentT) < 2000) {
                        break;
                    }
                    collector.setPower(0);
                    resetEncoder(collector);
                }
                if (runOnce8) {
                    currentT = System.currentTimeMillis();
                    runOnce8 = false;
                }

                updatingCurrentT = System.currentTimeMillis();
                if ((updatingCurrentT - currentT) < 2000) {
                    break;
                } else {
                    state = states.SHOOT;
                    runOnce2 = true;
                    runOnce4 = true;
                }
                break;

            case DRIVE2:
                if (runOnce9) {
                    currentT = System.currentTimeMillis();
                    runOnce9 = false;
                }

                updatingCurrentT = System.currentTimeMillis();
                if ((updatingCurrentT - currentT) < 1500) {
                    break;
                }
                if (motorLB.getCurrentPosition() > 1000) {
                    // Previous value was -1500
                    stopMotors(motorLB, motorLF, motorRB, motorRF);
                    state = states.STOP;
                    resetEncoder(motorLB);
                    resetEncoder(motorRB);
                } else {
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

                stopMotors(motorLB, motorLF, motorRB, motorRF);
                collector.setPower(0);
                break;

        }
            telemetry.addData("currentState", state);
            telemetry.addData("collectorCurrentPos", collector.getCurrentPosition());


    }




    public static void stopMotors(DcMotor motor1, DcMotor motor2, DcMotor motor3, DcMotor motor4)
    {
        motor1.setPower(0);
        motor2.setPower(0);
        motor3.setPower(0);
        motor4.setPower(0);
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

