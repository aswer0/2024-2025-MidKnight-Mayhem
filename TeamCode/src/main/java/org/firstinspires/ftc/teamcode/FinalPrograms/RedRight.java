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
                new Point(3.5, 158),
                new Point(64.3, 93)
        };
        specimen_target = new Point(16, 72);
        timer = new ElapsedTime();

        odometry = new Odometry(hardwareMap, 0, 0, 72, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);
        path = new Path(hang_path, wheelControl, odometry, telemetry, 0.01, 12, -90, 0.55);
    }

    @Override
    public void loop() {
        odometry.opt.update();

        if (!path.at_point(specimen_target, 2.5) && !start_hang_path){
            path.pid_to_point(specimen_target, 0);
            start_hang_path = true;
        }
        else{
            if (timer.milliseconds() >= 5000){
                path.update(true);
            }
        }

        telemetry.addData("X position: ", odometry.opt.get_x());
        telemetry.addData("Y position: ", odometry.opt.get_y());
        telemetry.addData("Heading: ", odometry.opt.get_heading());
        telemetry.addData("D value: ", path.get_d());
        telemetry.update();
    }
}
