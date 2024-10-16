package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.BezierCurve;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.Path;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Arrays;

@Config
@TeleOp
public class GVFTest extends OpMode {
    Odometry odometry;
    Path path;
    VectorField vf;
    WheelControl wheelControl;

    @Override
    public void init() {
        Point[][] cp = {
                {
                    new Point(10, 40),
                    new Point(114, -15),
                    new Point(130, 70)
                },
        };

        odometry = new Odometry(hardwareMap, 0, 8, 36, "BL", "FR", "FL");
        wheelControl = new WheelControl(hardwareMap, odometry);

        path = new Path(cp);
        vf = new VectorField(wheelControl, odometry, path,0.1,0.03, 0.5, 50, 0.1);
    }

    @Override
    public void loop() {
        odometry.update();
        vf.move();

        telemetry.addData("xPos", odometry.getxPos());
        telemetry.addData("yPos", odometry.getyPos());
        telemetry.addData("heading", Math.toDegrees(odometry.getHeading()));
    }
}
