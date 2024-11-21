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
public class SpecimenAuto extends OpMode {
    Point start_target;
    Point get_specimen_target;

    ElapsedTime timer;
    Sensors sensors;

    Odometry odometry;
    WheelControl wheelControl;
    Path path;

    SpecimenAuto.State state = SpecimenAuto.State.startPID;
    Lift lift;
    Manipulator manipulator;
    double deposit_state = 0;

    Intake intake;

    enum State {
        startPID,
        goToSpecimen,
        pickupSpecimen,
        followPath,
        deposit
    }

    @Override
    public void init() {
        Point[] follow_path = {
                new Point(7.875, 21.3),
                new Point(48, 21.3),
                new Point(3.7, 28.6),
                new Point(43.6, 47.3),
                new Point(16.5, 72.4),
                new Point(23, 65),
                new Point(36.2, 66)
        };

        start_target = new Point(36.2, 72);
        get_specimen_target = new Point(12.875, 28);

        timer = new ElapsedTime();
        sensors = new Sensors(hardwareMap, telemetry);

        odometry = new Odometry(hardwareMap, 0, 7.875, 66, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);
        path = new Path(follow_path, wheelControl, odometry, telemetry, 0.01, 12, 180, 1);

        wheelControl.change_mode(DcMotor.ZeroPowerBehavior.BRAKE);

        lift = new Lift(hardwareMap);
        manipulator = new Manipulator(hardwareMap);

        intake = new Intake(hardwareMap);

        manipulator.closeClaw();
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        odometry.opt.update();

        switch (state){
            case startPID:
                manipulator.closeClaw();

                if (timer.milliseconds() >= 200){
                    lift.toHighChamber();
                }

                path.follow_pid_to_point(start_target, 0);

                if (sensors.atChamber() && odometry.opt.get_heading()>-50 && odometry.opt.get_heading()<50){
                    timer.reset();
                    state = State.deposit;
                }

                if (timer.milliseconds()>5000) {
                    timer.reset();
                    state = State.deposit;
                }

                break;

            case deposit:
                lift.setPosition(550);
                if (timer.milliseconds() >= 300){
                    manipulator.openClaw();
                    if (deposit_state == 0){
                        state = State.goToSpecimen;
                    }
                }

                break;

            case goToSpecimen:
                path.follow_pid_to_point(get_specimen_target, 180);
                lift.toLowChamber();

                if (path.at_point(get_specimen_target, 4)){
                    state = State.pickupSpecimen;
                }

                break;

            case pickupSpecimen:
                if (sensors.get_front_dist() >= 2.5){
                    wheelControl.drive(-0.3, 0, 0, 0, 0.7);
                }
                else{
                    wheelControl.drive(0, 0, 0, 0, 0.7);
                    manipulator.closeClaw();

                    timer.reset();
                    start_target.y += 1;
                    start_target.x -= 1.75;
                    state = State.startPID;
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
