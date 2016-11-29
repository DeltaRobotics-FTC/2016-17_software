package org.firstinspires.ftc.teamcode.Testing;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**

 * Created by RoboticsUser on 10/29/2016.
 */
//@Autonomous (name = "ODS_Test", group = "")
public class ODS_Test extends OpMode
    {
        OpticalDistanceSensor ODS;


        public void init()
        {
            ODS = hardwareMap.opticalDistanceSensor.get("ODS");


        }


        public void loop()
        {

            telemetry.addData("ODSRawLightDetected", readAvgODSVal(ODS));

        }

        public double readAvgODSVal(OpticalDistanceSensor ODS)
        {
            double averagedRawLight = 0;
            for (int i = 0; i < 100; ++i )
            {
                averagedRawLight += ODS.getRawLightDetected();;
            }

            averagedRawLight /= 100;



            return  averagedRawLight;
        }


    }
