package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFSimplfied.Path;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.opencv.core.Point;

@Config
@TeleOp
public class GVFSimplifiedTest extends OpMode{
    Path path;
    WheelControl wheelControl;
    Odometry odometry;
    double a;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, 90, 0, 0, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);

        Point[] cp = {
                new Point(0, 0),
                new Point(76.3, 75.4),
                new Point(132, 12.6)
        };

        path = new Path(cp, wheelControl, odometry, telemetry, 0.001);
    }

    @Override
    public void loop() {
        odometry.opt.update();

        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        if (0.0 <= path.get_d() && path.get_d() <= 1.0){
            a = path.follow_path(0.4);

        }
        else{
            path.stop();
        }

        telemetry.addData("X position", odometry.opt.get_x());
        telemetry.addData("Y position", odometry.opt.get_y());
        telemetry.addData("Heading", odometry.opt.get_heading());
        telemetry.addData("target angle", a);
        telemetry.update();
    }
}
