package org.firstinspires.ftc.teamcode.Testing;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;


/**
 * Created by User on 10/27/2016.
 */

public class senseLine extends OpMode
{
   ColorSensor colorSensorR;
    ColorSensor colorSensorL;
    public void init()
    {

        colorSensorL = hardwareMap.colorSensor.get("colorSensorL");
        colorSensorR = hardwareMap.colorSensor.get("colorSensorR");
    }
   public senseLine(ColorSensor colorSensorR,ColorSensor colorSensorL)
    {
      this.colorSensorL = colorSensorL;
        this.colorSensorR = colorSensorR;


    }
    final int WhiteMaxVal = 110;
    final int WhiteMinVal = 95;



    public boolean senseLeftEdgeOfLine()
    {
        boolean sensedLine =  false;
        if (readAvgHueL() >= WhiteMinVal && readAvgHueL() <= WhiteMaxVal)
        {
            sensedLine = true;
        }
        else
        {
            sensedLine = false;
        }
        return sensedLine;
    }



    public boolean senseRightEdgeOfLine()
    {
        boolean sensedLine = false;
        if (readAvgHueR() >= WhiteMinVal && readAvgHueR() <= WhiteMaxVal)
        {
            sensedLine = true;
        }
        else
        {
            sensedLine = false;
        }
        return sensedLine;
    }



    public float readAvgHueL()
    {
        final int numOfSamples = 10;
        float averagedARGB = 0;
        float argbValToAverage = 0;
        for (int i = 0; i < numOfSamples; ++i )
        {
            argbValToAverage += (this.colorSensorL.argb() / 1000000);
        }

        averagedARGB = argbValToAverage / numOfSamples;



        return  averagedARGB;
    }

    public float readAvgHueR()
    {
        final int numOfSamples = 10;
        float averagedARGB = 0;
        float argbValToAverage = 0;
        for (int i = 0; i < numOfSamples; ++i )
        {
            argbValToAverage += (this.colorSensorR.argb() / 1000000);
        }

        averagedARGB = argbValToAverage / numOfSamples;



        return  averagedARGB;
    }
    public void loop()
    {


    }

}



