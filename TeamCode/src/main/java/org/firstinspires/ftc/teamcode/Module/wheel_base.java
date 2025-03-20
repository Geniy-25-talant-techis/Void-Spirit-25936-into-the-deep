package org.firstinspires.ftc.teamcode.Module;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.ExecutionException;

public class wheel_base { //class for wheel_base
    private DcMotor front_right_motor_drive = null; //init motors for wheel_base
    private DcMotor front_left_motor_drive = null;
    private DcMotor back_right_motor_drive = null;
    private DcMotor back_left_motor_drive = null;
    private BNO055IMU imu;
    private LinearOpMode l;

    public void init_wheel_base(HardwareMap hardwareMap, BNO055IMU imu, LinearOpMode l) { //init wheel_base

        front_right_motor_drive = hardwareMap.get(DcMotor.class, "MFR");
        back_right_motor_drive = hardwareMap.get(DcMotor.class, "MBR");
        front_left_motor_drive = hardwareMap.get(DcMotor.class, "MFL");
        back_left_motor_drive = hardwareMap.get(DcMotor.class, "MBL");

        front_left_motor_drive.setDirection(DcMotor.Direction.FORWARD);
        back_left_motor_drive.setDirection(DcMotor.Direction.FORWARD);
        back_right_motor_drive.setDirection(DcMotor.Direction.FORWARD);
        front_right_motor_drive.setDirection(DcMotor.Direction.REVERSE);

        front_right_motor_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        back_left_motor_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right_motor_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_left_motor_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right_motor_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        front_left_motor_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left_motor_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_motor_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right_motor_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.l = l;
        this.imu = imu;

        telemetry.addData("Wheel base Init",null);
    }

    public void drive(double x, double y, double xy, boolean fast_xy) throws InterruptedException {  //drive wheel_base
        double fast_XY = 0.5;

        if (fast_xy) {
            fast_XY += 0.5;
        }

        double max;
        double Y = -y;
        double X = -x;
        double XY = xy * fast_XY;


        double leftFrontPower = Y + X + XY;
        double rightFrontPower = -(Y + X + XY);
        double leftBackPower = Y - X + XY;
        double rightBackPower = Y + X - XY;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        front_left_motor_drive.setPower(leftFrontPower);
        front_right_motor_drive.setPower(rightFrontPower);
        back_left_motor_drive.setPower(leftBackPower);
        back_right_motor_drive.setPower(rightBackPower * 0.7);

    }


    public static double R = 7.5 / 2;
    public double pwr_YAW = 0.5;
    public double pwr_move = 1; //power for drive
    public static double ticks_motor = 1425; //ticks motor AndyMark Neverest 40
    public double one_tick = (2 * Math.PI * R) / ticks_motor; //one tick = S

    public void move_x(double x0) throws InterruptedException {
        front_right_motor_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_right_motor_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double ticks = x0 / one_tick;
        double now_pos_wheel_base = front_right_motor_drive.getCurrentPosition();

        while ((Math.abs(ticks) != 0) && (!l.isStopRequested())) {

            now_pos_wheel_base = front_right_motor_drive.getCurrentPosition();
            ticks = ticks - now_pos_wheel_base;
            if (ticks > 0) {
                drive(pwr_move, 0, 0, false);
            } else {
                drive(-pwr_move, 0, 0, false);
            }
        }
    }

    public void move_y(double y0) throws InterruptedException {
        front_right_motor_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_right_motor_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double ticks = y0 / one_tick;
        double now_pos_wheel_base = front_right_motor_drive.getCurrentPosition();

        while ((Math.abs(ticks) != 0) && (!l.isStopRequested())) {

            now_pos_wheel_base = front_right_motor_drive.getCurrentPosition();
            ticks = ticks - now_pos_wheel_base;
            if (ticks > 0) {
                drive(0, pwr_move, 0, false);
            } else {
                drive(0, 0, -pwr_move, false);
            }
        }
    }

        public void move_yaw (double xy0) throws InterruptedException {
            double alfha = imu.getAngularOrientation().thirdAngle;
            double YAW = xy0 - alfha;
            while (Math.abs(YAW) >= 5 && (!l.isStopRequested())) {
                if (YAW > 0) {
                    drive(0, 0, pwr_YAW, false);
                } else {
                    drive(0, 0, -pwr_YAW, false);
                }
            }
        }



    }

