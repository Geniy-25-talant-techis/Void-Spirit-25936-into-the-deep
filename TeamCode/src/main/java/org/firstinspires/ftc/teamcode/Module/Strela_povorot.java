package org.firstinspires.ftc.teamcode.Module;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Strela_povorot {
    private DcMotorEx Lift_Moment = null;
    private LinearOpMode l;
    public void init(HardwareMap hardwareMap, LinearOpMode l){
        Lift_Moment = hardwareMap.get(DcMotorEx.class,"MLM");

    }
}
