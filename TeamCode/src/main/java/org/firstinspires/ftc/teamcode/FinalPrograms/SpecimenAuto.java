package org.firstinspires.ftc.teamcode.FinalPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFSimplfied.Path;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Manipulator;
import org.opencv.core.Point;

@TeleOp
@Config
public class SpecimenAuto extends OpMode {
    public static double pos = 600;

    Point start_target;
    ElapsedTime timer;

    Odometry odometry;
    WheelControl wheelControl;
    Path path;

    SpecimenAuto.State state = SpecimenAuto.State.startPID;
    Lift lift;
    Manipulator manipulator;

    Intake intake;

    enum State {
        startPID,
        followPath,
        deposit
    }

    @Override
    public void init() {
        Point[] follow_path = {
                new Point(28.9, 54.9),
                new Point(2.4, 48.5),
                new Point(23.8, 46.5),
                new Point(1.6, 28.2),
                new Point(24.2, 29.2),
                new Point(-1.7, 13.5),
        };
        start_target = new Point(35.5, 66);
        timer = new ElapsedTime();

        odometry = new Odometry(hardwareMap, 0, 7.875, 66, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);
        path = new Path(follow_path, wheelControl, odometry, telemetry, 0.01, 12, 180, 0.7);

        lift = new Lift(hardwareMap);
        manipulator = new Manipulator(hardwareMap);

        intake = new Intake(hardwareMap);

        manipulator.closeClaw();
    }

    @Override
    public void loop() {
        odometry.opt.update();

        switch (state){
            case startPID:
                manipulator.closeClaw();
                lift.toHighChamber();
                path.pid_to_point(start_target, 0);

                if (path.at_point(start_target, 0.35)){
                    timer.reset();
                    state = State.deposit;
                }

                break;

            case deposit:
                lift.setPosition(pos);
                if (timer.milliseconds() >= 250){
                    manipulator.openClaw();
                }

                break;

        }

        telemetry.addData("X position: ", odometry.opt.get_x());
        telemetry.addData("Y position: ", odometry.opt.get_y());
        telemetry.addData("Heading: ", odometry.opt.get_heading());
        telemetry.addData("D value: ", path.get_d());
        telemetry.addData("Motor position: ", lift.getPosition());
        telemetry.addData("State: ", state);

        lift.update();
        intake.up();
        telemetry.update();
    }
}
