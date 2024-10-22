package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.Path;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.GVFPID;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.opencv.core.Point;

@Config
@TeleOp
public class GVFPIDTest extends OpMode {
    Odometry odometry;
    Path path;
    GVFPID vf;
    WheelControl wheelControl;

    @Override
    public void init() {
        // Bezier control points
        /*Point[][] cp = {
                {
                        new Point(5, 5),
                        new Point(70, 40),
                        new Point(116, 7)
                },
        };*/

        Point[][] cp = {
                {
                        new Point(0, 72),
                        new Point(20, 90),
                        new Point(16.4, 113.6),
                        new Point(35, 100),
                }
        };

        odometry = new Odometry(hardwareMap, 0, 8, 7, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);

        path = new Path(cp);
        vf = new GVFPID(wheelControl, odometry, path, 0.7,0.5, -45);
    }

    @Override
    public void loop() {
        odometry.opt.update();
        vf.move();
        telemetry.addData("xPos", odometry.opt.get_x());
        telemetry.addData("yPos", odometry.opt.get_y());
        telemetry.addData("heading", Math.toDegrees(odometry.opt.get_heading()));
        //telemetry.addData("vf_xPos", vf.odometry.opt.get_x());
        //telemetry.addData("speed", vf.speed);
        telemetry.addData("D", vf.D);
    }
}
