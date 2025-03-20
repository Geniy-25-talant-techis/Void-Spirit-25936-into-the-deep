package org.firstinspires.ftc.teamcode.Module;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Touch_sensor {
    private DigitalChannel Low_Drive,Max_Drive,Max_Static;
    private LinearOpMode l;

    public void init_touch_sensor(HardwareMap hardwareMap, LinearOpMode l){ //init
        Low_Drive = hardwareMap.get(DigitalChannel.class,"Static");
        Max_Drive = hardwareMap.get(DigitalChannel.class,"Max");
        Max_Static = hardwareMap.get(DigitalChannel.class,"Max_Low");

        Max_Static.setMode(DigitalChannel.Mode.INPUT);
        Low_Drive.setMode(DigitalChannel.Mode.INPUT);
        Max_Drive.setMode(DigitalChannel.Mode.INPUT);

        this.l = l;
        telemetry.addData("Touch Sensor Init",null);

    }
    public boolean max(){ //check max pos
        if(Max_Drive.getState()&&Max_Static.getState()){
            return false;
        }else{
            return true;
        }
    }

    public boolean min(){ //check min pos
        if(Low_Drive.getState()){
            return true;
        }else{
            return false;
        }
    }
}
