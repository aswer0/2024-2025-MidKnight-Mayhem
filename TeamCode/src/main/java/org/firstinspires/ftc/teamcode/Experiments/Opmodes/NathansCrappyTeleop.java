package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;

@TeleOp
public class NathansCrappyTeleop extends OpMode {
    Odometry odometry;
    WheelControl drive;

    double powerLevel = 1;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void init(){
        odometry = new Odometry(hardwareMap, 0, 7.875, 6.625, "OTOS");
        drive = new WheelControl(hardwareMap, odometry);

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

        //drive.correction_drive(currentGamepad1, powerLevel, telemetry);

        telemetry.addData("power level", powerLevel);
        telemetry.addData("Heading", odometry.opt.get_heading());
        telemetry.addData("X pos", odometry.opt.get_x());
        telemetry.addData("Y pos", odometry.opt.get_y());

        odometry.opt.update();
        telemetry.update();
    }
}
