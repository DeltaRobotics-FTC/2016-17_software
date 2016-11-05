package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


/**
 * Created by RoboticsUser on 11/1/2016.
 */
@Autonomous (name = "DR_Auto_10_Red", group = "")
public class DR_Auto_10_Red extends OpMode
{
    DcMotor motorLF;
    DcMotor motorLB;
    DcMotor motorRF;
    DcMotor motorRB;

    enum states {DRIVE1, STOP, TURN, DRIVE2}
    states state;

    public void init() {
        state = states.TURN;
        motorLB = hardwareMap.dcMotor.get("motorLB");
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorRB = hardwareMap.dcMotor.get("motorRB");
        motorRF = hardwareMap.dcMotor.get("motorRF");
    }
    public void loop()
    {

        switch (state)
        {
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
                if(motorLB.getCurrentPosition() > -1000)
                {
                    motorRF.setPower(0.0);
                    motorRB.setPower(0.0);
                    motorLF.setPower(-0.9);
                    motorLB.setPower(-0.9);
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
                if (motorLB.getCurrentPosition() < -1500)
                {
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
                    motorRF.setPower(0.6);
                    motorRB.setPower(-0.6);
                    motorLF.setPower(-0.6);
                    motorLB.setPower(-0.6);
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
