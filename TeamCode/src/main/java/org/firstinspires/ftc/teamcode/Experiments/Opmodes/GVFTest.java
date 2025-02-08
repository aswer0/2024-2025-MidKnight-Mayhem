package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.Utils;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.BCPath;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.opencv.core.Point;

@Config
@TeleOp
public class GVFTest extends OpMode {
    Odometry odometry;
    BCPath path;
    VectorField vf;
    WheelControl wheelControl;

    @Override
    public void init() {
        // Bezier control points
        Point[][] cp = {
                {
                        new Point(0, 72),
                        //new Point(28, 72),
                        new Point(10, 3),
                        new Point(64, 30)
                }
        };

        odometry = new Odometry(hardwareMap, 0, 0, 72, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);

        path = new BCPath(cp);
        vf = new VectorField(wheelControl, odometry, path, -90, true);
    }

    @Override
    public void start() {
        vf.timer.reset();
    }

    @Override
    public void loop() {
        odometry.opt.update();
        vf.move();
        //wheelControl.drive(-1, 0, 0, 0, 1);
        TelemetryPacket telemetry = new TelemetryPacket();
        //vf.set_velocity();
        //telemetry.put("v/p", Utils.len_v(vf.velocity)/vf.speed);
        //vf.move_to_point(new Point(28, 72), 0, 0.7);
//        telemetry.put("temp", vf.temp_turn);
//        telemetry.put("derivative", Math.toDegrees(Utils.angle_v(vf.path.derivative(vf.D))));
        telemetry.put("heading", vf.get_heading());
//        telemetry.put("turn_power", vf.turn_power);
        telemetry.put("speed", vf.speed);
//        telemetry.put("D", vf.D);
//        telemetry.put("PID", vf.PID);
        telemetry.put("vf_xPos", vf.get_x());
        telemetry.put("vf_yPos", vf.get_y());
        //telemetry.put("Closest", vf.get_closest());
        telemetry.put("powers", vf.powers);
        telemetry.put("turn speed", vf.turn_speed);
        (FtcDashboard.getInstance()).sendTelemetryPacket(telemetry);
    }
}