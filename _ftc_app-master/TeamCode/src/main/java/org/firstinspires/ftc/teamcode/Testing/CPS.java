package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by RoboticsUser on 12/17/2016.
 */
public class CPS extends OpMode
{
    public DcMotor motorLaunch;
    public long start;
    public long end = 0;
    public boolean launch;


    public void init()
    {
        motorLaunch = hardwareMap.dcMotor.get("motorLaunch");
    }

    public void loop()
    {

    }
    public void Shoot(int lower, int upper, DcMotor motor)
    {
        start = System.currentTimeMillis();
        if(!launch)
        {

        }


    }
}
