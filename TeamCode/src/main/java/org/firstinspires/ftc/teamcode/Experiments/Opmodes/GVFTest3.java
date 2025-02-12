package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFNew.BezierPath;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFNew.CompositePath;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFNew.VectorField;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.opencv.core.Point;

@Config
@TeleOp
public class GVFTest3 extends OpMode {
    Odometry odometry;
    CompositePath path;
    VectorField vf;
    WheelControl wheelControl;

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, 0, 0, 72, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);

        /*path = new CompositePath(
                10, -90,
                new BezierPath(
                       0.3, 0.5, 0.5, 5,
                        new Point(0, 72),
                        new Point(10, 10),
                        new Point(64, 30)
                ).linear_heading(0, -45)
        );*/

        vf = new VectorField(wheelControl, odometry);
    }

    @Override
    public void loop() {
        odometry.opt.update();
        //wheelControl.drive_relative(0, 0, 0.3, 0.5);
        //wheelControl.drive_angle(45, 0, 0.2, vf.get_heading())
        //wheelControl.drive_limit_power(0.2, 0, 0, 0.7, Math.toRadians(vf.get_heading()));
        vf.pid_to_point(new Point(72, 72), 0, 0.8);
        //vf.follow(path);
        //wheelControl.drive(-1, 0, 0, 0, 1);
        TelemetryPacket telemetry = new TelemetryPacket();
        telemetry.put("turn_power", vf.turn_power);
        telemetry.put("x_error", vf.x_error);
        telemetry.put("y_error", vf.y_error);
        telemetry.put("weird", vf.h_PID.error);
        //vf.set_velocity();
        //telemetry.put("v/p", Utils.len_v(vf.velocity)/vf.speed);
        //vf.move_to_point(new Point(28, 72), 0, 0.7);
        //telemetry.put("derivative", Math.toDegrees(Utils.angle_v(vf.path.derivative(vf.D))));
        //telemetry.put("closest", vf.get_closest());
        //telemetry.put("target_heading", vf.cur_bz);
        //telemetry.put("strafe_angle", vf.strafe_angle);
        //telemetry.put("x_error", vf.x_error);
        //telemetry.put("y_error", vf.y_error);
        //telemetry.put("heading", vf.get_heading());
        //telemetry.put("turn_power", vf.turn_power);
        //telemetry.put("speed", vf.speed);
        //telemetry.put("T", vf.T);
        //telemetry.put("PID", vf.PID);
        //telemetry.put("vf_xPos", vf.get_x());
        //telemetry.put("vf_yPos", vf.get_y());
        //telemetry.put("Closest", vf.get_closest());
        //telemetry.put("Velocity", vf.velocity);
        (FtcDashboard.getInstance()).sendTelemetryPacket(telemetry);
    }
}