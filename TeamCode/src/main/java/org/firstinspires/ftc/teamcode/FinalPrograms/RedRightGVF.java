package org.firstinspires.ftc.teamcode.FinalPrograms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.Path;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.opencv.core.Point;

@Autonomous
public class RedRightGVF extends OpMode {
    Point specimen_target;
    ElapsedTime timer;
    boolean start_hang_path = false;

    Odometry odometry;
    WheelControl wheelControl;
    VectorField hang_gvf;

    @Override
    public void init() {
        Point[][] hang_path_cp = {
                {
                        new Point(22, 72),
                        new Point(5, -18),
                        new Point(55, 45)
                }
        };
        Path hang_path = new Path(hang_path_cp);
        specimen_target = new Point(26, 72);

        odometry = new Odometry(hardwareMap, 0, 0, 72, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);
        hang_gvf = new VectorField(wheelControl, odometry, hang_path, 90);
    }

    @Override
    public void start() {
        timer = new ElapsedTime();
    }

    @Override
    public void loop() {
        odometry.opt.update();
        TelemetryPacket telemetry = new TelemetryPacket();
        //hang_gvf.move_to_point(specimen_target, 0, 0.4);
        if (!start_hang_path){
            hang_gvf.move_to_point(specimen_target, 0, 0.4);
        }

        if (timer.milliseconds() >= 2500) {
            start_hang_path = true;
            hang_gvf.move();
        }

        telemetry.put("X position: ", odometry.opt.get_x());
        telemetry.put("Y position: ", odometry.opt.get_y());
        telemetry.put("Heading: ", odometry.opt.get_heading());
        telemetry.put("D: ", hang_gvf.D);
        telemetry.put("Speed: ", hang_gvf.speed);
        telemetry.put("Velocity: ", hang_gvf.velocity);
        telemetry.put("Error: ", hang_gvf.error);
        (FtcDashboard.getInstance()).sendTelemetryPacket(telemetry);
    }
}
