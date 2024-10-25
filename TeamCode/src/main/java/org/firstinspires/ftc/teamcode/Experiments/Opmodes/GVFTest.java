package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.ExperimentalDrive;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.Path;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.opencv.core.Point;

@Config
@TeleOp
public class GVFTest extends OpMode {
    public static double target_x = 40;
    public static  double target_y = 105;

    Odometry odometry;
    Path path;
    VectorField vf;
    ExperimentalDrive wheelControl;

    @Override
    public void init() {
        // Bezier control points
        Point[][] cp = {
                {
                        new Point(0, 72),
                        new Point(20, 90),
                        new Point(16.4, 113.6),
                        new Point(35, 100),
                }
        };

        odometry = new Odometry(hardwareMap, 0, 0, 72, "OTOS");
        wheelControl = new ExperimentalDrive(hardwareMap, odometry);

        path = new Path(cp);
        vf = new VectorField(wheelControl, odometry, path, -90);
    }

    @Override
    public void loop() {
        odometry.opt.update();
        vf.move();
        //vf.pid_to_point(new Point(target_x, target_y), -90, 0.4);
        telemetry.addData("xPos", odometry.opt.get_x());
        telemetry.addData("yPos", odometry.opt.get_y());
        telemetry.addData("heading", Math.toDegrees(odometry.opt.get_heading()));
        telemetry.addData("vf_xPos", vf.odometry.opt.get_x());
        telemetry.addData("turn_speed", vf.turn_speed);
        telemetry.addData("speed", vf.speed);
        telemetry.addData("D", vf.D);
        telemetry.addData("PID", vf.PID);
    }
}