package org.firstinspires.ftc.teamcode.FinalPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFSimplfied.Path;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.opencv.core.Point;

@Autonomous
public class RedRight extends OpMode {
    Point specimen_target;
    ElapsedTime timer;
    boolean start_hang_path = false;

    Odometry odometry;
    WheelControl wheelControl;
    Path path;

    @Override
    public void init() {
        Point[] hang_path = {
                new Point(22, 72),
                new Point(5, -18),
                new Point(55, 45)
        };
        specimen_target = new Point(26, 72);
        timer = new ElapsedTime();

        odometry = new Odometry(hardwareMap, 0, 0, 72, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);
        path = new Path(hang_path, wheelControl, odometry, telemetry, 0.01, 12, 90, 0.7);
    }

    @Override
    public void loop() {
        odometry.opt.update();

        if (!start_hang_path){
            path.pid_to_point(specimen_target, 0);
        }

        if (timer.milliseconds() >= 2500){
            start_hang_path = true;
            path.update(true);
        }

        telemetry.addData("X position: ", odometry.opt.get_x());
        telemetry.addData("Y position: ", odometry.opt.get_y());
        telemetry.addData("Heading: ", odometry.opt.get_heading());
        telemetry.addData("D value: ", path.get_d());
        telemetry.update();
    }
}
