package org.firstinspires.ftc.teamcode.Testing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by RoboticsUser on 11/1/2016.
 */
@Autonomous (name = "DR_Auto_10_Blue", group = "")
public class DR_Auto_10_Blue extends OpMode
{
    DcMotor motorLF;
    DcMotor motorLB;
    DcMotor motorRF;
    DcMotor motorRB;

    DcMotor launcherWheel;
    Servo popper;

    long b = 0;
    long c = 0;
    long a = 0;
    double popperUp = 0.14;
    double popperDown = 0.0;

    boolean test = true;
    boolean test1 = true;

    enum states {DRIVE1, STOP, TURN, DRIVE2, SHOOT1, SHOOT2}
    states state;

    public void init() {
        state = states.SHOOT1;
        motorLB = hardwareMap.dcMotor.get("motorLB");
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorRB = hardwareMap.dcMotor.get("motorRB");
        motorRF = hardwareMap.dcMotor.get("motorRF");

        launcherWheel = hardwareMap.dcMotor.get("launcherWheel");
        popper = hardwareMap.servo.get("popper");
        popper.setPosition(popperDown);

    }
    public void loop()
    {

        switch (state)
        {
            case SHOOT1:
                if(test)
                {
                    c = System.currentTimeMillis();
                    test = false;
                }
                launcherWheel.setPower(.40);
                a = System.currentTimeMillis();
                if((a - c) < 5000)
                {
                    break;
                }
                else {
                    if(test1)
                    {
                        c = System.currentTimeMillis();
                        test1 = false;
                    }
                    popper.setPosition(popperUp);
                    b = System.currentTimeMillis();
                    if ((b - c) > 2000) {
                        launcherWheel.setPower(0.0);
                        popper.setPosition(popperDown);
                        test = true;
                        test1 = true;
                        state = states.SHOOT2;
                    }
                    break;
                }

            case SHOOT2:
                if(test)
                {
                    c = System.currentTimeMillis();
                    test = false;
                }
                launcherWheel.setPower(.40);
                a = System.currentTimeMillis();
                if((a - c) < 3000)
                {
                    break;
                }
                else {
                    if(test1)
                    {
                        c = System.currentTimeMillis();
                        test1 = false;
                    }
                    popper.setPosition(popperUp);
                    b = System.currentTimeMillis();
                    if ((b - c) > 2000) {
                        launcherWheel.setPower(0.0);
                        state = states.DRIVE2;
                    }
                    break;
                }

            case DRIVE1:
                if (motorLB.getCurrentPosition() < -300)
                {
                    motorRF.setPower(0.0);
                    motorRB.setPower(0.0);
                    motorLF.setPower(0.0);
                    motorLB.setPower(0.0);
                    state = states.TURN;
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
                    motorRF.setPower(0.6);
                    motorRB.setPower(-0.6);
                    motorLF.setPower(-0.6);
                    motorLB.setPower(-0.6);
                    break;
                }
                break;

            case TURN:
                if(motorRB.getCurrentPosition() < 0)
                {
                    motorRF.setPower(0.0);
                    motorRB.setPower(0.0);
                    motorLF.setPower(0.0);
                    motorLB.setPower(0.0);
                    telemetry.addData("Position LF", motorLF.getCurrentPosition());
                    telemetry.addData("Position LB", motorLB.getCurrentPosition());
                    telemetry.addData("Position RF", motorRF.getCurrentPosition());
                    telemetry.addData("Position RB", motorRB.getCurrentPosition());
                }
                else{
                    motorRF.setPower(0.0);
                    motorRB.setPower(0.0);
                    motorLF.setPower(0.0);
                    motorLB.setPower(0.0);
                    state = states.DRIVE2;
                    motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                break;
            case DRIVE2:
                if (motorLB.getCurrentPosition() > 3000)
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
                break;
        }

    }
}
