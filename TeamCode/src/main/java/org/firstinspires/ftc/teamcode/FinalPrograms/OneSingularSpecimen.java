package org.firstinspires.ftc.teamcode.FinalPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.BezierCurve;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.Path;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.opencv.core.Point;

public class OneSingularSpecimen extends OpMode {
    Odometry odometry;
    VectorField vf;
    WheelControl wheelControl;
    Point[][] cp = {{}};
    Path path = new Path(cp);

    Point specimen_target;

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, 0, 0, 72, "OTOS");
        specimen_target = new Point(26, 72);
        wheelControl = new WheelControl(hardwareMap, odometry);
        vf = new VectorField(wheelControl, odometry, path, 0);
    }

    @Override
    public void loop() {
        odometry.update();
        vf.move_to_point(specimen_target, 0, 0.4);
    }
}
