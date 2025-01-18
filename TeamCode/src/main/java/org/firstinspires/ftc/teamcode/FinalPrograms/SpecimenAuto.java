package org.firstinspires.ftc.teamcode.FinalPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    public static double pos = 550;

    public static double horizontal_pos = -470;
    public static double target_angle_intake = 140;
    public static double target_angle_spit = 45;

    public static double power = 0.7;
    public static double arm_pow = 0.7;
    public static double arm_for = 0.3;

    public static double target_x = 36.5; //36.2
    public static double target_y = 72;
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
    Manipulator manipulator;
    Intake intake;
    HorizontalSlides horizontalSlides;
    Arm arm;

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
                new Point(40.2, 68.7),
                new Point(18.8, 14.7),
                new Point(65.3, 55.2),
                new Point(60.9, 23.3),
        };

        target = new Point(target_x, target_y);
        //retune specemine target
        get_specimen_target = new Point(15, 28);
        //maybe retune this
        get_sample_target = new Point(sample_x, sample_y);

        timer = new ElapsedTime();
        sensors = new Sensors(hardwareMap, telemetry);

        odometry = new Odometry(hardwareMap, 0, 7.875, 66, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);
        path = new Path(follow_path, wheelControl, odometry, telemetry, 0.01, 12, 180, power);

        wheelControl.change_mode(DcMotor.ZeroPowerBehavior.BRAKE);

        lift = new Lift(hardwareMap);
        manipulator = new Manipulator(hardwareMap);
        intake = new Intake(hardwareMap, sensors);
        horizontalSlides = new HorizontalSlides(hardwareMap);
        arm = new Arm(hardwareMap);

        arm.toAutoStartPosition();
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
                arm.closeClaw();
                arm.outtakeSpecimen1();

                path.follow_pid_to_point(target, 0);

                if (sensors.atArmDistChamber() && odometry.opt.get_heading() > -50 && odometry.opt.get_heading() < 50) {
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
                intake.up();

                if (timer.milliseconds() >= 200) {
                    arm.outtakeSpecimen2();
                    wheelControl.drive(-arm_for, 0, 0, 0, arm_pow);
                }
                if (timer.milliseconds() >= 500){
                    if (deposit_state >= 2 && deposit_state < 5){
                        timer.reset();
                        arm.openClaw();
                        state = State.intakeSample;
                    }
                    else {
                        timer.reset();
                        arm.openClaw();
                        state = State.goToSpecimen;
                    }
                }

                break;

            case goToSpecimen:
                intake.up();
                arm.intakeSpecimen();
                path.follow_pid_to_point(get_specimen_target, 0);

                if (path.at_point(get_specimen_target, 7)) {
                    wheelControl.drive(0, 0, 0, 0, 0.7);
                    timer.reset();
                    state = State.pickupSpecimen;
                }

                break;

            case pickupSpecimen:
                intake.up();

                //need retuning distance for arm
                if (sensors.get_back_dist() >= 2.5) {
                    //maybe doing diagonal instead cause faster
                    wheelControl.drive(0.3, 0, 0, 0, 0.7);
                    timer.reset();
                } else {
                    wheelControl.drive(0, 0, 0, 0, 0);
                    manipulator.closeClaw();

                    if (timer.milliseconds() > 200) {
                        timer.reset();
                        target.y -= 1;
                        state = State.pid;
                    }
                }

                break;

            case intakeSample:
                if (deposit_state > 4){
                    state = State.goToSpecimen;
                }

                path.follow_pid_to_point(new Point(intake_sample_x,intake_sample_y), target_angle_intake);

                arm.toIdlePosition();
                intake.down();
                intake.intake();
                if (timer.milliseconds()>1000) {
                    horizontalSlides.setPosition(horizontal_pos);
                }

                if (timer.milliseconds()>2500){
                    timer.reset();
                    intake.intake();
                    intake.reverseDown();
                    state = State.setupSpitSample;
                }

                break;

            case setupSpitSample:
                intake.intake();
                intake.reverseDown();
                path.follow_pid_to_point(new Point(30, 30), target_angle_spit);
                horizontalSlides.setPosition(-300);

                if (odometry.opt.get_heading()<60 || timer.milliseconds()> 1000){
                    wheelControl.drive(0,0,0,0, 0.00001);
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

                if (timer.milliseconds() >= 600){
                    intake_sample_y -= 8;
                    intake_sample_x += 0.5;
                    target_angle_intake += 4;
                    deposit_state++;

                    horizontalSlides.setPosition(-100);
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
