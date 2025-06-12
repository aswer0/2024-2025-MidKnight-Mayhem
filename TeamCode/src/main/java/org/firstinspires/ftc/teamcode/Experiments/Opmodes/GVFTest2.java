package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFSimplfied.Path;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.opencv.core.Point;

@Config
@TeleOp
@Disabled
public class GVFTest2 extends OpMode {
    Odometry odometry;
    WheelControl wheelControl;
    Path path;

    @Override
    public void init() {
        // Bezier control points
        Point[] cp = {
                new Point(11,28),
                new Point(15.3, 65.5),
                new Point(21.2, 68.2),
                new Point(36.5, 75)
        };

        odometry = new Odometry(hardwareMap, 0, 0, 72, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);

        path = new Path(cp, wheelControl, odometry, telemetry, 1, 0.1, -90, 1);
    }

    @Override
    public void loop() {
        odometry.opt.update();
        //wheelControl.drive(-1, 0, 0, 0, 1);
        TelemetryPacket telemetry = new TelemetryPacket();
        //vf.set_velocity();
        path.follow_pid_to_point(new Point(28, 72), -90);
        telemetry.put("vf_xPos", odometry.opt.get_x());
        telemetry.put("vf_yPos", odometry.opt.get_y());
        telemetry.put("vf_heading", odometry.opt.get_heading());
        //telemetry.put("Closest", vf.get_closest());
        //telemetry.put("Velocity", vf.velocity);
        (FtcDashboard.getInstance()).sendTelemetryPacket(telemetry);
    }
}