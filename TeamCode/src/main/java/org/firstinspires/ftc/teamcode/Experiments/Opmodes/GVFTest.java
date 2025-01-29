package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.BCPath;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.VectorFieldOld;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.opencv.core.Point;

@Config
@TeleOp
public class GVFTest extends OpMode {
    Odometry odometry;
    BCPath path;
    VectorFieldOld vf;
    WheelControl wheelControl;

    @Override
    public void init() {
        // Bezier control points
        Point[][] cp = {
                {
                        new Point(0, 72),
                        new Point(10, 3),
                        new Point(64, 30)
                }
        };

        odometry = new Odometry(hardwareMap, 0, 0, 72, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);

        path = new BCPath(cp);
        vf = new VectorFieldOld(wheelControl, odometry, path, -90);
    }

    @Override
    public void loop() {
        odometry.opt.update();
        vf.move();
        //wheelControl.drive(0.2, -0.2, 0, 0, 0.5);
        TelemetryPacket telemetry = new TelemetryPacket();
        //vf.move_to_point(new Point(28, 72), 0, 0.4);
        telemetry.put("targetAngle", vf.target_angle);
        telemetry.put("heading", vf.get_heading());
        telemetry.put("vf_xPos", vf.get_x());
        telemetry.put("vf_yPos", vf.get_y());
        telemetry.put("turn_speed", vf.turn_speed);
        telemetry.put("speed", vf.speed);
        telemetry.put("D", vf.D);
        telemetry.put("PID", vf.PID);
        telemetry.put("X Error", vf.x_error);
        telemetry.put("Y Error", vf.y_error);
        telemetry.put("Closest", vf.get_closest());
        telemetry.put("Velocity", vf.velocity);
        (FtcDashboard.getInstance()).sendTelemetryPacket(telemetry);
    }
}