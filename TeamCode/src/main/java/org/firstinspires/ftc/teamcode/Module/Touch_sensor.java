package org.firstinspires.ftc.teamcode.Module;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class Touch_sensor {
    private DigitalChannel Low_Drive,Max_Drive,Max_Static,Min_Strela;
    private LinearOpMode l;

    public void init_touch_sensor(HardwareMap hardwareMap, LinearOpMode l){ //init
        Low_Drive = hardwareMap.get(DigitalChannel.class,"Static");
        Max_Drive = hardwareMap.get(DigitalChannel.class,"Max");
        Max_Static = hardwareMap.get(DigitalChannel.class,"Max_Low");
        //Min_Strela = hardwareMap.get(DigitalChannel.class,"Min");

       // Min_Strela.setMode(DigitalChannel.Mode.INPUT);
        Max_Static.setMode(DigitalChannel.Mode.INPUT);
        Low_Drive.setMode(DigitalChannel.Mode.INPUT);
        Max_Drive.setMode(DigitalChannel.Mode.INPUT);

        this.l = l;
        l.telemetry.addData("Touch Sensor Init",null);

    }
    public void return_ts(){
        l.telemetry.addData("maxx",Max_Drive.getState());
        l.telemetry.addData("midle",Max_Static.getState());
        l.telemetry.addData("low",Low_Drive.getState());
        l.telemetry.update();

    }
    public boolean max(){//check max pos
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
    public boolean min_pvrt(){ //check min pos strela
    if (Min_Strela.getState()){
        return true;
    }else{
        return false;
    }
    }
}
