package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.DriveCorrection;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;

import java.util.List;

@TeleOp
public class FieldOrientedDrive extends OpMode {
    Odometry odometry;
    DriveCorrection driveCorrection;
    WheelControl drive;

    double powerLevel=1;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, 0, 0, 0, "OTOS");
        drive = new WheelControl(hardwareMap, odometry);
        driveCorrection = new DriveCorrection(odometry);
    }

    @Override
    public void init_loop() {
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        odometry.opt.update();
        drive.correction_drive(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, Math.toRadians(odometry.opt.get_heading()), powerLevel);

        telemetry.addData("power level", powerLevel);

        telemetry.addData("xPos", odometry.getxPos());
        telemetry.addData("yPos", odometry.getyPos());
        telemetry.addData("heading", odometry.getHeading());
    }
}
