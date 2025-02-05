package org.firstinspires.ftc.teamcode.FinalPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFSimplfied.Path;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Manipulator;
import org.opencv.core.Point;

@Disabled
@Autonomous
public class Park extends OpMode {
    Odometry odometry;
    WheelControl wheelControl;
    Path path;

    Lift lift;
    Manipulator manipulator;
    Point park_point;

    @Override
    public void init() {
        Point[] hang_path = {
                new Point(22, 72),
                new Point(5, -18),
                new Point(55, 45)
        };

        odometry = new Odometry(hardwareMap, 0, 0, 0, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);
        path = new Path(hang_path, wheelControl, odometry, telemetry, 0.01, 12, 90, 0.7);

        park_point = new Point(34, 5);
    }

    @Override
    public void loop() {
        //odometry.opt.update();

        wheelControl.drive(0, 1, 0, 0, 0.5);

        telemetry.addData("X position: ", odometry.opt.get_x());
        telemetry.addData("Y position: ", odometry.opt.get_y());
        telemetry.addData("Heading: ", odometry.opt.get_heading());
        telemetry.addData("D value: ", path.get_d());

        telemetry.update();
    }

}
