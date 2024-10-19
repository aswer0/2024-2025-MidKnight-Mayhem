package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.BezierCurve;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.Path;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Arrays;

@Config
@TeleOp
public class GVFTest extends OpMode {
    Odometry odometry;
    Path path;
    VectorField vf;
    WheelControl wheelControl;

    @Override
    public void init() {
        Point[][] cp = {
                {
                        new Point(6.4, 7.4),
                        new Point(142.3, 10.5),
                        new Point(145, 45),
                        new Point(120, 140),
                        new Point(12.4, 110.4)
                },
        };

        odometry = new Odometry(hardwareMap, 0, 8, 7, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);

        path = new Path(cp);
        vf = new VectorField(wheelControl, odometry, path, 0.7,0.5, 20, 20, 0.1);
    }

    @Override
    public void loop() {
        odometry.opt.update();
        vf.move();
        telemetry.addData("xPos", odometry.opt.get_x());
        telemetry.addData("yPos", odometry.opt.get_y());
        telemetry.addData("heading", Math.toDegrees(odometry.opt.get_heading()));
        telemetry.addData("angle_to_path", vf.angle_to_path());
        telemetry.addData("vf_xPos", vf.odometry.opt.get_x());
        telemetry.addData("turn_speed", vf.turn_speed);
        telemetry.addData("speed", vf.speed);
        telemetry.addData("D", vf.D);
    }
}

