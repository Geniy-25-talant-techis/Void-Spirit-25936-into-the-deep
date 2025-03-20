package org.firstinspires.ftc.teamcode.Module;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Grab {
    private Servo Pvrt,Clash;
    private LinearOpMode l;

    public void init_grab(HardwareMap hardwareMap,LinearOpMode l){
        Pvrt = hardwareMap.get(Servo.class,"SLC");
        Clash = hardwareMap.get(Servo.class,"SLP");

        Clash.setDirection(Servo.Direction.REVERSE);
        Pvrt.setDirection(Servo.Direction.FORWARD);

        this.l = l;

        telemetry.addData("Grab init",null);
    }
    double clash_close = 0;
    double clash_open = 0.5;
    public void clash (boolean close_true,boolean open_true) throws InterruptedException{
        if(close_true){
            Clash.setPosition(clash_close);
        }
        if(open_true){
            Clash.setPosition(clash_open);
        }
    }

}
