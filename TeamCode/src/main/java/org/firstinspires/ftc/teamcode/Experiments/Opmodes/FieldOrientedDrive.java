package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.DriveCorrection;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFNew.VectorField;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.opencv.core.Point;

import java.util.List;

@TeleOp
public class FieldOrientedDrive extends OpMode {
    Odometry odometry;
    DriveCorrection driveCorrection;
    WheelControl drive;
    VectorField vf;

    double powerLevel=1;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    enum PID_state {
        pid1, pid2, pid3, idle
    }

    PID_state pid_state = PID_state.idle;

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, 0, 0, 72, "OTOS");
        drive = new WheelControl(hardwareMap, odometry);
        driveCorrection = new DriveCorrection(odometry);
        vf = new VectorField(new WheelControl(hardwareMap, odometry), odometry);
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

        if (!previousGamepad1.circle && currentGamepad1.circle){
            pid_state = PID_state.pid1;
        }
        if (!previousGamepad1.square && currentGamepad1.square){
            pid_state = PID_state.pid2;
        }
        if (!previousGamepad1.triangle && currentGamepad1.triangle){
            pid_state = PID_state.pid3;
        }
        if (!previousGamepad1.x && currentGamepad1.x){
            pid_state = PID_state.idle;
        }

        switch (pid_state) {
            case pid1:
                vf.pid_to_point(new Point(28, 72), -90, 1);
                break;

            case pid2:
                vf.pid_to_point(new Point(20, 15), 90, 1);
                break;

            case pid3:
                vf.pid_to_point(new Point(10, 30), 0, 1);
                break;

            case idle:
                drive.correction_drive(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, Math.toRadians(odometry.opt.get_heading()), powerLevel);
                break;

        }

        telemetry.addData("xPos", odometry.opt.get_x());
        telemetry.addData("yPos", odometry.opt.get_y());
        telemetry.addData("heading", odometry.opt.get_heading());
    }
}
