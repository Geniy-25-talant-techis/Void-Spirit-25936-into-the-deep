package org.firstinspires.ftc.teamcode.Module;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class Grab {
    private Servo Pvrt,Clash;
    private LinearOpMode l;

    public void init_grab(HardwareMap hardwareMap,LinearOpMode l){
        Pvrt = hardwareMap.get(Servo.class,"SLC");
        Clash = hardwareMap.get(Servo.class,"SLP");

        boolean true_pvrt;
        boolean true_clash;

        Clash.setDirection(Servo.Direction.REVERSE);
        Pvrt.setDirection(Servo.Direction.FORWARD);

        this.l = l;

        l.telemetry.addData("Grab init",null);
    }
    public static double clash_close0 = 0.035;
    public static double clash_open0 = 0.3;
    public static double povorot_up0 = 0.5;
    public static double povorot_down0 = 0;
    public void clash_open (boolean status_clash_open) throws InterruptedException{
        if(status_clash_open) {
            Clash.setPosition(clash_open0);
        }
    }
    public void clash_close(boolean status_clash_close) throws InterruptedException{
        if(status_clash_close){
            Clash.setPosition(clash_close0);
        }
    }

    public void povorot_up (boolean status_povorot_up) throws InterruptedException{
        if(status_povorot_up){
            Pvrt.setPosition(povorot_up0);
        }

    }
    public void povorot_down(boolean status_povorot_down) throws InterruptedException{
        if(status_povorot_down){
            Pvrt.setPosition(povorot_down0);
        }
    }

}
