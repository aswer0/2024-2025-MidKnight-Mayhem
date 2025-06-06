package org.firstinspires.ftc.teamcode.OldOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.BCPath;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Outtake.Lift;
import org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Outtake.Manipulator;
import org.opencv.core.Point;

@Autonomous
@Disabled
public class OneSingularSpecimen extends OpMode {
    Odometry odometry;
    VectorField vf;
    WheelControl wheelControl;

    ElapsedTime timer;

    Point[][] cp = {{}};
    BCPath path = new BCPath(cp);

    Point specimen_target;
    Point park_target;

    Lift lift;
    Manipulator manipulator;

    boolean clawOpen = false;
    double deposit_by = Double.POSITIVE_INFINITY;

    // Hangs a specimen if already at position
    public void hang_specimen() {
        if (!clawOpen) {
            deposit_by = timer.milliseconds() + 500;
            lift.setPosition(lift.getPosition() - 250);
        }
        if(deposit_by - timer.time() < 0 && !clawOpen) {
            deposit_by = Double.POSITIVE_INFINITY;
            manipulator.openClaw();
            clawOpen = true;
        }
    }

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, 0, 0, 72, "OTOS");
        specimen_target = new Point(26, 72);
        park_target = new Point(2, 5);
        wheelControl = new WheelControl(hardwareMap, odometry);
        //vf = new VectorField(wheelControl, odometry, path, 0);
    }

    @Override
    public void start() {
        timer = new ElapsedTime();
    }

    @Override
    public void loop() {
        odometry.update();
        lift.update();

        if (timer.milliseconds() < 5000) {
            vf.pid_to_point(specimen_target, 0, 0.4);
            lift.toHighChamber();
        }
    }
}
