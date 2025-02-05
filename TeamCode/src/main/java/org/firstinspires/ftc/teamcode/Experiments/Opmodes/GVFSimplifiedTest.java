package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFSimplfied.Path;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.opencv.core.Point;

@Disabled
@Config
@TeleOp
public class GVFSimplifiedTest extends OpMode{
    public static double target_angle = 0;

    Path path;
    WheelControl wheelControl;
    Odometry odometry;

    Point specimen_target;

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, 0, 7.875, 6.625, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);

        Point[] cp = {
                new Point(4.3, 8.4),
                new Point(72, 51),
                new Point(107, 6.4)
        };

        path = new Path(cp, wheelControl, odometry, telemetry, 0.01, 12, 180, 0.7);
        specimen_target = new Point(24, 72);
    }

    @Override
    public void loop() {
        odometry.opt.update();

        path.update(false);

        telemetry.addData("X position", odometry.opt.get_x());
        telemetry.addData("Y position", odometry.opt.get_y());
        telemetry.addData("Heading", odometry.opt.get_heading());
        telemetry.addData("D value", path.get_d());

        telemetry.update();
    }
}
