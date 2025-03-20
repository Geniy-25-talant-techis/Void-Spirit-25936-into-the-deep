package org.firstinspires.ftc.teamcode.Module;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Strela { //class for strela
    private DcMotor strela_motor;
    private LinearOpMode l;//motor for lift

    public void init_strela(HardwareMap hardwareMap, LinearOpMode l ){ //init motor for lift
        strela_motor = hardwareMap.get(DcMotor.class,"MLTS");
        strela_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        strela_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.l = l;
        telemetry.addData("Strela init",null);
    }

    public void power_strela (double pwr_strela,boolean max,boolean min){ //power for motor strela+check max/min pos
        if(max && !l.isStopRequested()){
            if(pwr_strela>=0){
                pwr_strela = 0;
            }else{
                pwr_strela = pwr_strela;
            }
        }
        if(min && !l.isStopRequested()){
            if(pwr_strela<=0){
                pwr_strela = 0;
            }else{
                pwr_strela = pwr_strela;
            }
        }
        strela_motor.setPower(pwr_strela);
    }
}
