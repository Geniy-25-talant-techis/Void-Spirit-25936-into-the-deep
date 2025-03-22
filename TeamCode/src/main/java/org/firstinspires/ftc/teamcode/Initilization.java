package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.icu.text.MessagePattern;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Module.Grab;
import org.firstinspires.ftc.teamcode.Module.Strela;
import org.firstinspires.ftc.teamcode.Module.Touch_sensor;
import org.firstinspires.ftc.teamcode.Module.wheel_base;
@Config
public class Initilization {
    wheel_base wb = new wheel_base();
    public Grab g = new Grab();
    Touch_sensor ts = new Touch_sensor();
    Strela strl = new Strela();
    Gamepad gamepad1;
    Gamepad gamepad2;
    FtcDashboard dash;

    HardwareMap hwm;
    //BNO055IMU imu;
    LinearOpMode l;
    public void init(LinearOpMode l){
        hwm = l.hardwareMap;
        this.l = l;
        this.gamepad1 = l.gamepad1;
        this.gamepad2 = l.gamepad2;
        dash =  FtcDashboard.getInstance();

//       BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//        imu = hwm.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
//        while (!imu.isGyroCalibrated());
        wb.init_wheel_base(hwm,l);
        strl.init_strela(hwm,l);
        g.init_grab(hwm,l);
        ts.init_touch_sensor(hwm,l);
    }

    public void teleOp() throws InterruptedException {
        while (!l.isStopRequested()) {
            wb.drive(l.gamepad1.left_stick_x, l.gamepad1.left_stick_y, l.gamepad1.right_stick_x, l.gamepad1.left_bumper);
            strl.power_strela(l.gamepad2.left_stick_y, (l.gamepad2.right_stick_y ),ts.max(),ts.min());
            g.povorot_up(l.gamepad2.right_bumper);
            g.povorot_down(l.gamepad2.left_bumper);
            g.clash_close(l.gamepad1.b);
            g.clash_open(l.gamepad1.x);
            l.telemetry.addData("Max", ts.max());
            l.telemetry.addData("Min", ts.min());
            ts.return_ts();

            l.telemetry.update();
            //
        }
    }
    public void teleOop_solo()throws InterruptedException{
        while(!l.isStopRequested()){
            wb.drive(l.gamepad1.left_stick_x, l.gamepad1.left_stick_y,(-l.gamepad1.right_trigger+l.gamepad1.left_trigger),l.gamepad2.left_bumper);
            strl.power_strela(l.gamepad1.right_stick_x, (l.gamepad1.right_stick_y ),ts.max(),ts.min());
            g.povorot_up(l.gamepad1.right_bumper);
            g.povorot_down(l.gamepad1.left_bumper);
            g.clash_close(l.gamepad1.b);
            g.clash_open(l.gamepad1.x);
            l.telemetry.addData("Max", ts.max());
            l.telemetry.addData("Min", ts.min());
            ts.return_ts();

            l.telemetry.update();
        }
    }
    public static double zero = 0;
    public static double power__full = 1;
    public static double power__yaw__lift = 0.7;
    public void auto() throws InterruptedException{
       wb.drive_auto(power__full,0,0,false);
        strl.power_strela(power__yaw__lift,0.3,false,false);
        l.sleep(1000);
        wb.drive_auto((power__yaw__lift-0.5),0,0,false);
        strl.power_strela((power__full-0.1),power__yaw__lift,false,false);
        g.povorot_up(false);
        g.povorot_down(true);
        l.sleep(1000);
        wb.drive_auto(zero,0,0,false);
        strl.power_strela(zero,power__yaw__lift,false,false);
        g.clash_close(false);
        g.clash_open(true);
        l.sleep(1000);
        g.povorot_down(false);
        g.povorot_up(true);
         wb.drive_auto(-power__full,0,0,false);
        strl.power_strela(zero,power__yaw__lift,false,false);
        l.sleep(500);
        wb.drive_auto(-power__full,0,0,false);
        strl.power_strela(-power__yaw__lift,power__yaw__lift,false,false);
        l.sleep(2000);
        wb.drive_auto(-power__full,0,0,false);
        l.sleep(4000);
        wb.drive_auto(zero,0,0,false);
    }
    public void auto_parking() throws InterruptedException {
        wb.drive_auto(-0.7,0,0,false);
        l.sleep(1000);
        wb.drive_auto(0,0,0,false);
    }

}
