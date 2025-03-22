package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Initilization;
@Autonomous(name = "Auto_park")
public class auto_park extends LinearOpMode {
    Initilization i = new Initilization();
    @Override
    public void runOpMode() throws InterruptedException {
        i.init(this);
        i.g.clash_open(true);
        i.g.povorot_up(true);
        waitForStart();
        i.auto_parking();
    }
}
