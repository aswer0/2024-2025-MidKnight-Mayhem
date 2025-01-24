package org.firstinspires.ftc.teamcode.FinalPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.BCPath;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFSimplfied.Path;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Arm;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Manipulator;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Sensors;
import org.opencv.core.Point;

@Autonomous
@Config
public class SpecimenAuto extends OpMode {
    public static double sample_x = 24.5;
    public static double sample_y = 37;

    public static double intake_sample_x = 24;
    public static double intake_sample_y = 35;

    public static double pos = 650;
    public static double dist_thresh = 1;
    public static double intake_dist_thresh = 3.5;

    public static double horizontal_pos = -450;
    public static double target_angle_intake = 140;
    public static double target_angle_spit = 22;

    public static double power = 0.7;

    public static double target_x = 50; //36.2
    public static double target_y = 75;
    double deposit_state = 0;

    Point target;
    Point get_specimen_target;
    Point get_sample_target;

    ElapsedTime timer;
    Sensors sensors;

    Odometry odometry;
    WheelControl wheelControl;
    Path path;

    State state = State.pid;
    Lift lift;
    Intake intake;
    HorizontalSlides horizontalSlides;
    Arm arm;

    // pid -> deposit -> goToSpecimen / intakeSample (pick up from lines)
    // goToSpecimen -> pickupSpecimen -> pid
    // intakeSample -> goToSpecimen (if done w/ samples) / setupSpitSample
    // setupSpitSample -> spitSample

    enum State {
        pid,
        goToSpecimen,
        pickupSpecimen,
        deposit,
        intakeSample,
        spitSample,
        setupSpitSample
    }

    @Override
    public void init() {
        Point[] follow_path = {
                new Point(target_x, target_y),
                new Point(33.7, 50.6),
                new Point(35, 25),
                new Point(11, 28),
        };

        target = new Point(target_x, target_y);
        get_specimen_target = new Point(12.875, 28);
        get_sample_target = new Point(sample_x, sample_y);

        timer = new ElapsedTime();
        sensors = new Sensors(hardwareMap, telemetry);

        odometry = new Odometry(hardwareMap, 0, 7.875, 66, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);
        path = new Path(follow_path, wheelControl, odometry, telemetry, 0.01, 12, 180, power);

        wheelControl.change_mode(DcMotor.ZeroPowerBehavior.BRAKE);

        lift = new Lift(hardwareMap);
        intake = new Intake(hardwareMap, sensors);
        horizontalSlides = new HorizontalSlides(hardwareMap);
        arm = new Arm(hardwareMap);
        arm.toAutoStartPosition();
        arm.closeClaw();

        
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        odometry.opt.update();

        switch (state) {
            case pid:
                intake.up();
                intake.stop();
                arm.outtakeSpecimen1();
                arm.closeClaw();

                lift.toHighChamber();

                path.follow_pid_to_point(target, 0);

                if (sensors.get_front_dist() <= dist_thresh && odometry.opt.get_heading() > -50 && odometry.opt.get_heading() < 50) {
                    deposit_state++;
                    timer.reset();
                    state = State.deposit;
                }

                if (timer.milliseconds() > 5000) {
                    timer.reset();
                    deposit_state++;
                    state = State.deposit;
                }

                break;

            case deposit:
                intake.stop();
                intake.up();

                arm.openClaw();
                arm.outtakeSpecimen1();

                if (timer.milliseconds() >= 400){
                    // im not sure if it will intake 3 times then move to go to specimen state, check this
                    if (deposit_state >= 2 && deposit_state < 6){
                        timer.reset();
                        state = State.intakeSample;
                    }
                    else {
                        timer.reset();
                        state = State.goToSpecimen;
                    }
                }

                break;

            case goToSpecimen:
                intake.up();
                intake.stop();
                path.follow_pid_to_point(get_specimen_target, 0);

                if (timer.milliseconds() > 500){
                    lift.intakeSpecimen();
                    arm.intakeSpecimen();
                }

                if (path.at_point(get_specimen_target, 5)) { //4
                    wheelControl.drive(0, 0, 0, 0, 0.7);
                    state = State.pickupSpecimen;
                }

                break;

            case pickupSpecimen:
                intake.up();
                intake.stop();
                arm.intakeSpecimen();

                if (sensors.get_back_dist() >= intake_dist_thresh || timer.milliseconds() < 3000) {
                    wheelControl.drive(-0.5, 0, 0, 0, 0.7);
                } else {
                    wheelControl.drive(0, 0, 0, 0, 0);
                    arm.closeClaw();

                    timer.reset();
                    target.y -= 1.5; // this has to be tuned better for more space on the right
                    state = State.pid;
                }

                break;

            case intakeSample:
                // im not sure if it will intake 3 times then move to go to specimen state, check this
                if (deposit_state > 4){
                    state = State.goToSpecimen;
                }

                if (odometry.opt.get_heading()<(target_angle_intake-5)) {
                    path.follow_pid_to_point(new Point(intake_sample_x,intake_sample_y), target_angle_intake);
                } else {
                    wheelControl.drive(0,0,0,0,0);
                }

                intake.down();
                intake.intake();
                if (timer.milliseconds()>1000) {
                    horizontalSlides.setPosition(horizontal_pos);
                }

                if (timer.milliseconds()>1500){ //original 2500
                    timer.reset();
                    state = State.setupSpitSample;
                }

                break;

            case setupSpitSample:
                intake.intake();
                intake.reverseDown();
                path.follow_pid_to_point(new Point(30, 30), target_angle_spit);
                horizontalSlides.setPosition(-300);

                if (odometry.opt.get_heading()<60 || timer.milliseconds()> 1000){ //original 1000
                    wheelControl.drive(0,0,0,0, 0);
                    timer.reset();
                    state = State.spitSample;
                }

                break;

            case spitSample:
                if (timer.milliseconds()>0) horizontalSlides.setPosition(-470);
                if (timer.milliseconds() >= 150){
                    intake.reverseDown();
                    intake.reverse();
                }

                if (timer.milliseconds() >= 250){ //original 600
                    intake_sample_y -= 8;
                    intake_sample_x += 0.5;
                    target_angle_intake += 2.5;
                    deposit_state++;

                    horizontalSlides.setPosition(-100);
                    timer.reset();
                    state = State.intakeSample;
                }

                break;
        }

        telemetry.addData("X position: ", odometry.opt.get_x());
        telemetry.addData("Y position: ", odometry.opt.get_y());
        telemetry.addData("Heading: ", odometry.opt.get_heading());
        telemetry.addData("D value: ", path.get_d());
        telemetry.addData("Deposit State", deposit_state);
        telemetry.addData("Motor position: ", lift.getPosition());
        telemetry.addData("Horizontal Motor position: ", horizontalSlides.horizontalSlidesMotor.getCurrentPosition());
        telemetry.addData("State: ", state);
        telemetry.addData("Front Distance", sensors.get_front_dist());

        lift.update();
        telemetry.update();
        horizontalSlides.update();
    }
}