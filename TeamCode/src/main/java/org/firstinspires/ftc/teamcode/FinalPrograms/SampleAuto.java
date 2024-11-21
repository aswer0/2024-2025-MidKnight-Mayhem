package org.firstinspires.ftc.teamcode.FinalPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFSimplfied.Path;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Manipulator;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Sensors;
import org.opencv.core.Point;

@Autonomous
@Config
public class SampleAuto extends OpMode {
    Point start_target;
    Point get_specimen_target;

    ElapsedTime timer;
    Sensors sensors;

    Odometry odometry;
    WheelControl wheelControl;
    Path path;

    SampleAuto.State state = SampleAuto.State.startPID;
    Lift lift;
    Manipulator manipulator;
    double deposit_state = 0;

    Intake intake;

    enum State {
        startPID,
        goToSpecimen,
        pickupSpecimen,
        followPathBack,
        deposit
    }

    @Override
    public void init() {
        start_target = new Point(36.2, 66);
        get_specimen_target = new Point(36.2, 66);

        Point[] follow_path = {
                new Point(7.875, 21.3),
                new Point(48, 21.3),
                new Point(3.7, 28.6),
                new Point(43.6, 47.3),
                new Point(16.5, 72.4),
                new Point(23, 65),
                get_specimen_target
        };

        timer = new ElapsedTime();
        sensors = new Sensors(hardwareMap, telemetry);

        odometry = new Odometry(hardwareMap, 0, 7.875, 66, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);
        path = new Path(follow_path, wheelControl, odometry, telemetry, 0.01, 12, 0, 0.7);

        wheelControl.change_mode(DcMotor.ZeroPowerBehavior.BRAKE);

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

                path.follow_pid_to_point(start_target, 0);

                if (sensors.atChamber()){
                    timer.reset();
                    state = State.deposit;
                }

                break;

            case deposit:
                lift.setPosition(550);

                if (timer.milliseconds() >= 300){
                    manipulator.openClaw();
                    if (deposit_state == 0 || deposit_state == 1){
                        state = State.goToSpecimen;
                    }
                }

                break;

            case goToSpecimen:
                path.update( 185);
                lift.toLowChamber();

                if (sensors.get_front_dist() >= 2.5){
                    timer.reset();
                    state = State.pickupSpecimen;
                }

                break;

            case pickupSpecimen:
                path.stop();

                if (timer.milliseconds() >= 250){
                    manipulator.closeClaw();

                    timer.reset();

                    get_specimen_target.y += 1.2;
                    state = State.followPathBack;
                }

                break;

            case followPathBack:
                path.retrace(true);
                lift.toHighChamber();

                if (sensors.atChamber()){
                    state = State.deposit;
                }

                break;

        }

        telemetry.addData("X position: ", odometry.opt.get_x());
        telemetry.addData("Y position: ", odometry.opt.get_y());
        telemetry.addData("Heading: ", odometry.opt.get_heading());
        telemetry.addData("D value: ", path.get_d());
        telemetry.addData("Motor position: ", lift.getPosition());
        telemetry.addData("State: ", state);
        telemetry.addData("Front Distance", sensors.get_front_dist());

        lift.update();
        intake.up();
        telemetry.update();
    }
}
