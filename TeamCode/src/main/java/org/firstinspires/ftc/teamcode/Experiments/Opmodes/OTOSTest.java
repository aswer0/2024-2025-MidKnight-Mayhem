package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.DriveCorrection;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDController;

@TeleOp
public class OTOSTest extends OpMode {
    public static double hp = 0.011, hi = 0, hd = 0.0001;
    PIDController heading;

    Odometry odometry;
    WheelControl drive;
    DriveCorrection driveCorrection;

    double powerLevel = 1;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void init(){
        odometry = new Odometry(hardwareMap, 0, 8, 7, "OTOS");
        drive = new WheelControl(hardwareMap, odometry);
        heading = new PIDController(hp, hi, hd);

        driveCorrection = new DriveCorrection(drive, odometry);

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

        double target_angle = gamepad1.right_stick_x*Math.abs(gamepad1.right_stick_x);
        drive.drive(-gamepad1.left_stick_y, 1.1*gamepad1.left_stick_x, driveCorrection.stable_correction(target_angle), 0, powerLevel);

        telemetry.addData("power level", powerLevel);
        telemetry.addData("Linear Scalar", odometry.opt.get_linear_scalar());
        telemetry.addData("Angular Scalar", odometry.opt.get_angular_scalar());
        telemetry.addData("X position", odometry.opt.get_x());
        telemetry.addData("Y position", odometry.opt.get_y());
        telemetry.addData("Heading", odometry.opt.get_heading());
        telemetry.addData("Target angle: ", target_angle);

        telemetry.update();
    }
}

