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

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, 90, 0, 0, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);

        Point[] cp = {
                new Point(7, 80.3),
                new Point(9, 86.1),
                new Point(22.8, 109.5),
                new Point(45.6, 88),
                new Point(45.6, 112.3)
        };


        path = new Path(cp, wheelControl, odometry, telemetry, 0.005, 15, 90, 0.5);
    }

    @Override
    public void loop() {
        odometry.opt.update();

        path.update();

        telemetry.addData("X position", odometry.opt.get_x());
        telemetry.addData("Y position", odometry.opt.get_y());
        telemetry.addData("Heading", odometry.opt.get_heading());
        telemetry.addData("Distance threshold", a);
        telemetry.addData("D value", path.get_d());
        telemetry.update();
    }
}
