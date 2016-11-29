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
    boolean finished = false;

    enum states {DRIVE1, STOP, TURN, DRIVE2}
    states state;

    public void init() {
        state = states.TURN;
//        motorLB = hardwareMap.dcMotor.get("motorLB");
//        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorRB = hardwareMap.dcMotor.get("motorRB");
//        motorRF = hardwareMap.dcMotor.get("motorRF");
    }

    public void loop() {
        if (!finished) {
            telemetry.addData("Status", "Finished initialization");
            move(0.25, 0.25, 0.25, 0.25, 1000, motorRB);
            move(0.25, 0.25, -0.25, -0.25, 500, motorRB);
            move(-0.25, -0.25, -0.25, -0.25, 250, motorRB);
            finished = true;
        }
    }

    private void move(double rbSpeed, double rfSpeed, double lbSpeed, double lfSpeed) {
        motorRB.setPower(rbSpeed);
//        motorRF.setPower(rfSpeed);
//        motorLB.setPower(-lbSpeed);
//        motorLF.setPower(-lfSpeed);
        sendMotorTelemetry();
    }

    private void move(double rbSpeed, double rfSpeed, double lbSpeed, double lfSpeed, int encoderTarget, DcMotor encoderMotor) {
        encoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (Math.abs(encoderMotor.getCurrentPosition()) < Math.abs(encoderTarget)) {
            move(rbSpeed, rfSpeed, lbSpeed, lfSpeed);
        }
        stopMotors();
    }

    private void move(double rbSpeed, double rfSpeed, double lbSpeed, double lfSpeed, int millis) {
        // TODO
    }

    private void stopMotors() {
        move(0, 0, 0, 0);
    }

    private void sendMotorTelemetry() {
//        telemetry.addData("MotorLF", motorLF.getCurrentPosition());
//        telemetry.addData("MotorLB", motorLB.getCurrentPosition());
//        telemetry.addData("MotorRF", motorRF.getCurrentPosition());
        telemetry.addData("MotorRB", motorRB.getCurrentPosition());
        telemetry.update();
    }
}