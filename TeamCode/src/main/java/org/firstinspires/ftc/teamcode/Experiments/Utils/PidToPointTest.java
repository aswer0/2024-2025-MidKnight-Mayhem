package org.firstinspires.ftc.teamcode.Experiments.Utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.DriveCorrection;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.Experiments.Utils.utils;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDController;
import org.opencv.core.Point;

@TeleOp
@Config
public class PidToPointTest extends OpMode {
    Odometry odometry;
    WheelControl drive;
    VectorField vf;

    double powerLevel = 1;

    public static double target_x = 18;
    public static double target_y = 125;
    public static double target_angle = 125;
    public static double start_x = 7.25;
    public static double start_y = 96+7.7;
    public static double start_angle = 90;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void init(){
        odometry = new Odometry(hardwareMap, start_angle, start_x, start_y, "OTOS");
        drive = new WheelControl(hardwareMap, odometry);
        vf = new VectorField(drive, odometry);
    }

    @Override
    public void loop() {
        if (gamepad1.y) {
            odometry.opt.calibrate_tracking();
        }

        if (gamepad1.x) {
            odometry.opt.calibrate_imu();
        }

        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) powerLevel -= 0.1;
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) powerLevel += 0.1;

        odometry.opt.update();

        vf.pid_to_point(new Point(target_x, target_y), target_angle, 0.7);

        telemetry.addData("power level", powerLevel);
        telemetry.addData("Heading | OTOS", odometry.opt.get_heading());
        telemetry.addData("X pos | OTOS", odometry.opt.get_x());
        telemetry.addData("Y pos | OTOS", odometry.opt.get_y());

        telemetry.update();
    }
}
