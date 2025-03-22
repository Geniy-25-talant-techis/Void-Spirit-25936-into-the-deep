package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Initilization;
@Autonomous(name = "auto")
public class auto extends LinearOpMode {
    Initilization i = new Initilization();
    @Override
    public void runOpMode() throws InterruptedException {
        i.init(this);
        i.g.clash_close(true);
        i.g.povorot_up(true);
        waitForStart();
        i.auto();


    }
}
