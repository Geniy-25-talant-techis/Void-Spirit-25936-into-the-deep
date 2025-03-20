package org.firstinspires.ftc.teamcode.TeleOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Tele")
public class tele extends OpMode {
    public DcMotorEx Lift_Moment = null;
    public DcMotorEx Lift_TS = null;
    //
    public DcMotor LeftFrotnDrive= null;
    public DcMotor RightFrontDrive= null;
    public DcMotor leftBackDrive= null;
    public DcMotor RightBackDrive = null;
    public Servo Pvrt,Clash;
    public DigitalChannel Low_Drive,Max_Drive,Max_Static;
    @Override
    public void init() {

        LeftFrotnDrive = hardwareMap.get(DcMotor.class,"MFL");
        RightFrontDrive = hardwareMap.get(DcMotor.class,"MFR");
        leftBackDrive = hardwareMap.get(DcMotor.class,"MBL");
        RightBackDrive = hardwareMap.get(DcMotor.class,"MBR");
        Lift_Moment = hardwareMap.get(DcMotorEx.class,"MLM");
        Lift_TS = hardwareMap.get(DcMotorEx.class,"MLTS");
        Pvrt = hardwareMap.get(Servo.class,"SLC");
        Clash = hardwareMap.get(Servo.class,"SLP");
        Low_Drive = hardwareMap.get(DigitalChannel.class,"Static");
        Max_Drive = hardwareMap.get(DigitalChannel.class,"Max");
        Max_Static = hardwareMap.get(DigitalChannel.class,"Max_Low");
        Max_Static.setMode(DigitalChannel.Mode.INPUT);
        Low_Drive.setMode(DigitalChannel.Mode.INPUT);
        Max_Drive.setMode(DigitalChannel.Mode.INPUT);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftFrotnDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LeftFrotnDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        RightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        RightFrontDrive.setDirection(DcMotor.Direction.REVERSE); //прапвое заднее

        LeftFrotnDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


       // Lift_TS.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Lift_TS.setDirection(DcMotorSimple.Direction.REVERSE);
//        Lift_TS.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//
//        Lift_TS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
Lift_Moment.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Lift_TS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Lift_Moment.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

double power_up;


        Clash.setDirection(Servo.Direction.REVERSE);

        double ask = Clash.getPosition();
        telemetry.addData("PositionClash",ask);
        telemetry.addData("Position Povorot",Pvrt.getPosition());
        telemetry.addData("Robot","Ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean revers =false;
        boolean Max = false;
       double pwr = gamepad2.right_stick_y;
        if(Max_Static.getState()==false&&Max_Drive.getState()==false){
            Max = true;
        }
        Lift_TS.setPower(pwr);
        double max;
        double axial   = -gamepad1.left_stick_y;
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;


        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }
        LeftFrotnDrive.setPower(leftFrontPower);
        RightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        RightBackDrive.setPower(rightBackPower*0.7);

        if(gamepad1.b){
            Pvrt.setPosition(0.5);
        }
        if(gamepad1.x){
            Pvrt.setPosition(0);
        }

        if(gamepad1.a){
            Clash.setPosition(0);
        }
        if(gamepad1.y){
            Clash.setPosition(0.5);
        }
//        if(Low_Drive.getState()==true){
//            Lift_TS.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//            Lift_TS.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//            Lift_TS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            revers = false;
//        }
//        if(Max_Static.getState()==true&&Max_Drive.getState()==true){
//            Lift_TS.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//            Lift_TS.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//            Lift_TS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            revers = true;
//        }if(gamepad2.dpad_left&&Low_Drive.getState()==false){
//            Lift_TS.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//            Lift_TS.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//            Lift_TS.setVelocity(4500);
//            Lift_TS.setTargetPosition(Lift_TS.getCurrentPosition()-100);
//            Lift_TS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//
//
//        }
//        if(gamepad2.dpad_right&&Max == false ){
//            Lift_TS.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//            Lift_TS.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//            Lift_TS.setVelocity(4500);
//            Lift_TS.setTargetPosition((Lift_TS.getCurrentPosition()+100));
//            Lift_TS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
        Lift_Moment.setPower(-gamepad2.left_trigger+gamepad2.right_trigger   );

//        if(gamepad2.dpad_up){
//            Lift_Moment.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//            Lift_Moment.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            Lift_Moment.setVelocity(500);
//            Lift_Moment.setTargetPosition(Lift_Moment.getCurrentPosition()+25);
//            Lift_Moment.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//        if(gamepad2.dpad_down){
//
//            Lift_Moment.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//            Lift_Moment.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            Lift_Moment.setVelocity(500);
//            Lift_Moment.setTargetPosition(Lift_Moment.getCurrentPosition()-25);
//            Lift_Moment.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }

//        if(gamepad2.a){
//            Low_Pvrt_Lift();
//        }
//        if(gamepad2.y){
//            Max_Pvrt_Lift();
//        }
//
        telemetry.addData("PositionPovorot",Pvrt.getPosition());
        telemetry.addData("PositionClash",Clash.getPosition());
        telemetry.addData("PositionLift_M",Lift_Moment.getTargetPosition());
        telemetry.addData("PositionLift_TS",Lift_TS.getTargetPosition());
        telemetry.addData("Max",Max);


        telemetry.update();
    }
//    public void Max_Pvrt_Lift(){
//        Lift_Moment.setVelocity(700);
//
//        Lift_Moment.setTargetPosition(300);
//    }
//    public void Low_Pvrt_Lift(){
//        Lift_Moment.setVelocity(700);
//        Lift_Moment.setTargetPosition(0);
//    }
}
