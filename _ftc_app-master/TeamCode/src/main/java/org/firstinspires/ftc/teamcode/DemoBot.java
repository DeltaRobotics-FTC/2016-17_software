package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by RoboticsUser on 9/20/2016.
 */
@TeleOp (name = "DemoBot", group = "")
public class DemoBot extends OpMode
    {
        DcMotor motorLeftFront;
        DcMotor motorRightFront;
        DcMotor motorLeftBack;
        DcMotor motorRightBack;

        float throttleLeft;
        float throttleRight;


        public void init()
            {
               hardwareMap.dcMotor.get("motorLeftFront");
               hardwareMap.dcMotor.get("motorRightFront");
                hardwareMap.dcMotor.get("motorLeftBack");
                hardwareMap.dcMotor.get("motorRightBack");


            }


        public void loop()
            {

                gamepad1.left_stick_y = throttleLeft;
                gamepad1.right_stick_y = throttleRight;
                Range.clip(throttleLeft, -1.0, 1.0);
                Range.clip(throttleRight, -1.0, 1.0);
                motorLeftFront.setPower(throttleLeft);
                motorRightFront.setPower(throttleRight);
                motorLeftBack.setPower(throttleLeft);
                motorRightBack.setPower(throttleRight);






            }

    }
