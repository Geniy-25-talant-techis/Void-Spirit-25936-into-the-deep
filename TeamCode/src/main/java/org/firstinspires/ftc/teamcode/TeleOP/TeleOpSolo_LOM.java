package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Initilization;
@TeleOp(name = "teleOpsolo_LOM")
public class TeleOpSolo_LOM extends LinearOpMode {
    Initilization i = new Initilization();
    @Override
    public void runOpMode() throws InterruptedException {
        i.init(this);
        telemetry.addData("Robot Ready",null);
        telemetry.update();
        waitForStart();
        i.teleOop_solo();

        //
    }
}
