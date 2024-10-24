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
        odometry = new Odometry(hardwareMap, -90, 7.875, 72, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);

        Point[] cp = {
                new Point(7.875, 72),
                new Point(19, 87),
                new Point(9.8, 106.8),
                new Point(38, 105.6),
        };


        path = new Path(cp, wheelControl, odometry, telemetry, 0.005, 5, -90, 0.5);
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
