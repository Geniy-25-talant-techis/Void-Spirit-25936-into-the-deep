package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.icu.text.MessagePattern;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Module.Grab;
import org.firstinspires.ftc.teamcode.Module.Strela;
import org.firstinspires.ftc.teamcode.Module.Touch_sensor;
import org.firstinspires.ftc.teamcode.Module.wheel_base;

public class Initilization {
    wheel_base wb = new wheel_base();
    Grab g = new Grab();
    Touch_sensor ts = new Touch_sensor();
    Strela strl = new Strela();
    HardwareMap hwm;
    BNO055IMU imu;
    LinearOpMode l;
    public void init(LinearOpMode l){
        hwm = l.hardwareMap;
        this.l = l;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwm.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        while (!imu.isGyroCalibrated());
        wb.init_wheel_base(hwm,imu,l);
        strl.init_strela(hwm,l);
        g.init_grab(hwm,l);
        ts.init_touch_sensor(hwm,l);
    }
    public void teleOp() throws InterruptedException{
        while(!l.isStopRequested())
            wb.drive(gamepad1.left_stick_x, gamepad1.left_stick_y,gamepad1.right_stick_x,gamepad1.left_bumper);
            strl.power_strela(gamepad2.left_stick_y,gamepad2.right_stick_y,ts.max(),ts.min());
            g.povorot_up(gamepad1.y);
            g.povorot_down(gamepad1.a);
            g.clash_close(gamepad1.b);
            g.clash_open(gamepad1.x);
            telemetry.addData("Max",ts.max());
            telemetry.addData("Min",ts.min());
            telemetry.update();
    }

}
