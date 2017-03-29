package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by RoboticsUser on 12/17/2016.
 */
@TeleOp(name = "CapBallTakeDown", group = "")
public class CapBallTakeDown extends OpMode {

    DcMotor motorR;
    DcMotor motorRF;
    DcMotor motorL;
    DcMotor motorLF;
    DcMotor launcherWheel;
    DcMotor motorLift;
    DcMotor collector;
    DcMotor capLift;
    Servo popper;
    Servo bBP;
    Servo boot;

    double capLiftPower = 0;

    public void init()
    {
        bBP = hardwareMap.servo.get("bBP");
        boot = hardwareMap.servo.get("boot");
        launcherWheel = hardwareMap.dcMotor.get("launcherWheel");
        popper = hardwareMap.servo.get("popper");
        collector = hardwareMap.dcMotor.get("collector");
        capLift = hardwareMap.dcMotor.get("capLift");
        popper.setPosition(.94);
        bBP.setPosition(.99);
        boot.setPosition(.1);
    }
    public void loop()
    {
        popper.setPosition(.94);
        bBP.setPosition(.99);
        boot.setPosition(.1);

        if (gamepad2.right_stick_y > 0)
        {
            capLiftPower = 1.0;
        }
        else if (gamepad2.right_stick_y < 0)
        {
            capLiftPower = -1.0;
        }
        else
        {
            capLiftPower = 0.0;
        }
        capLift.setPower(capLiftPower);

    }
}