package org.firstinspires.ftc.teamcode.FinalPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFSimplfied.Path;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Arm;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Alliance;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Sensors;
import org.firstinspires.ftc.teamcode.Experiments.Utils.EventScheduler;
import org.opencv.core.Point;

@Autonomous
@Config
public class SixSpecimenAuto extends OpMode {
    public static double intake_sample_x_1 = 29;
    public static double intake_sample_y_1 = 36.75;
    public static double intake_sample_x_2 = 28.5;
    public static double intake_sample_y_2 = 27.5;
    public static double intake_sample_x_3 = 28;
    public static double intake_sample_y_3 = 19.5;
    Point intake_target;

    public static double first_spit_x = 26;
    public static double first_spit_y = 39;
    public static double first_spit_angle = 20;
    Point first_spit_target = new Point(first_spit_x, first_spit_y);

    public static double dist_thresh = 5;

    public static double horizontal_pos = -450;
    public static double target_angle_intake = 140;
    public static double target_angle_spit = 22;

    public static double power = 1;

    public static double target_x = 42;
    public static double stop_pid_x = 39;
    public static double target_y = 65;
    public Point hang_target;
    public Point deposit_target;

    public static double sample_x = 8;
    public static double sample_y = 120;
    public Point sample_deposit = new Point(sample_x, sample_y);

    public static double set_pos_tolerance = 1;
    public static double ticks_per_inch = 75;

    double intake_state = 0;
    double deposit_state = 0;

    double sub_intake_slide_pos;

    public static double get_specimen_x = 15;
    public static double get_specimen_y = 26;
    Point get_specimen_target = new Point(get_specimen_x, get_specimen_y);
    Point sub_intake = new Point(0, 0);

    public static double deposit_pid_x = 45;
    public static double deposit_pid_y = 80;
    public static double pid_tuner_1 = 15;
    public static double pid_tuner_2 = 20;

    public static double pickup_time_1 = 500;
    public static double pickup_time_2 = 350;

    //Point sub_intake = new Point(0, 0);
    //Point sub_intake_limit = new Point(27.5, 21.3);

    ElapsedTime state_timer;
    EventScheduler event_scheduler;
    Sensors sensors;

    Odometry odometry;
    WheelControl wheelControl;
    VectorField vf;
    Path path;

    State state = State.firstPid;

    Lift lift;
    Intake intake;
    HorizontalSlides horizontalSlides;
    Arm arm;

    Alliance alliance = Alliance.red;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    // pid -> deposit -> goToSpecimen / intakeSample (pick up from lines)
    // goToSpecimen -> pickupSpecimen -> pid
    // intakeSample -> goToSpecimen (if done w/ samples) / setupSpitSample
    // setupSpitSample -> spitSample

    enum State {
        firstPid,
        depositPid,
        subIntake,
        firstSpit,
        goToSpecimen,
        pickupSpecimen,
        deposit,
        intakeSample,
        spitSample,
        depositSample,
        park
    }

    @Override
    public void init() {
        Point[] follow_path = {
                new Point(11,28),
                new Point(10.3, 65.5),
                new Point(19.2, 68.2),
                new Point(36.5, 75)
        };

        hang_target = new Point(target_x, target_y);

        state_timer = new ElapsedTime();
        event_scheduler = new EventScheduler();
        sensors = new Sensors(hardwareMap, telemetry);

        odometry = new Odometry(hardwareMap, 180, 8.875, 65, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);

        vf = new VectorField(wheelControl, odometry);
        path = new Path(follow_path, wheelControl, odometry, telemetry, 0.1, 13, 180, 1);

        wheelControl.change_mode(DcMotor.ZeroPowerBehavior.BRAKE);

        lift = new Lift(hardwareMap);
        intake = new Intake(hardwareMap, sensors);
        horizontalSlides = new HorizontalSlides(hardwareMap);
        arm = new Arm(hardwareMap);
        arm.toAutoStartPosition();
        arm.closeClaw();
        state_timer.reset();
    }

    @Override
    public void init_loop() {
        odometry.opt.update();
        horizontalSlides.update();

        if (gamepad1.left_bumper) {
            alliance = Alliance.blue;
        } else if (gamepad1.right_bumper) {
            alliance = Alliance.red;
        }

        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        /*if (!previousGamepad1.square && currentGamepad1.square) {
            hang_target.y = vf.get_y();
        }*/

        if (!previousGamepad1.dpad_down && currentGamepad1.dpad_down){
            sub_intake.x -= set_pos_tolerance;
        }
        if (!previousGamepad1.dpad_up && currentGamepad1.dpad_up){
            sub_intake.x += set_pos_tolerance;
        }
        if (!previousGamepad1.dpad_left && currentGamepad1.dpad_left){
            sub_intake.y += set_pos_tolerance;
            hang_target = new Point(target_x, target_y+sub_intake.y);
        }
        if (!previousGamepad1.dpad_right && currentGamepad1.dpad_right){
            sub_intake.y -= set_pos_tolerance;
            hang_target = new Point(target_x, target_y+sub_intake.y);
        }

        telemetry.addData("Alliance", alliance);
        telemetry.addLine();
        telemetry.addLine("Sample position (relative to robot)");
        //telemetry.addData("intake lr", hang_target.y);
        telemetry.addData("current point", hang_target);
        telemetry.addData("intake b/f (+ is forward)", sub_intake.x);
        telemetry.addData("intake l/r (+ is right)", -sub_intake.y);
        telemetry.addData("robot position", vf.get_pos());
    }

    public void resetTimers() {
        state_timer.reset();
        event_scheduler.clear();
    }

    @Override
    public void start() {
        resetTimers();
        deposit_target = new Point(deposit_pid_x, deposit_pid_y);
        sub_intake_slide_pos = -ticks_per_inch*sub_intake.x;
    }

    @Override
    public void loop() {
        odometry.opt.update();
        lift.update();
        horizontalSlides.update();

        switch (state) {
            case firstPid:
                // Reverse hang
                horizontalSlides.setPosition(sub_intake_slide_pos);
                arm.backOuttakeSpecimen1();
                lift.toBackChamber1Pos();
                vf.pid_to_point(hang_target, 180, power);
                intake.up();
                intake.stop();
                intake.closeDoor();

                // Sub intake when it's there
                if (sensors.isTouchBack() || state_timer.milliseconds() > 2000 || vf.get_x() > stop_pid_x) {
                    deposit_state++;
                    wheelControl.drive_relative(-0.3, 0, 0, 1);
                    resetTimers();
                    state = State.subIntake;
                }
                break;

            case subIntake: //also back deposit
                // Bit of delay so not having 2 samples in possession
                if (state_timer.milliseconds() < 100) break;
                lift.toBackChamber2Pos();
                arm.backOuttakeSpecimen2();
                intake.down();
                intake.intake();

                if (state_timer.milliseconds() > 500) {
                    arm.openClaw();
                    arm.toIdlePosition();
                    lift.intakeSpecimen();
                }

                // Extend intake slides once at correct position
                if (Math.abs(horizontalSlides.horizontalSlidesMotor.getCurrentPosition() - sub_intake_slide_pos) <= 25) {
                    event_scheduler.createIfNew("Extend intake slides");
                }
                if (event_scheduler.during("Extend intake slides", 300)) {
                    horizontalSlides.setPosition(horizontal_pos);
                }

                // Move on if correct or timer failsafe
                if (intake.hasCorrectSample(false) || state_timer.milliseconds() >= 1500) {
                    intake.stop();
                    intake.up();
                    horizontalSlides.setPosition(0);
                    intake_state++;
                    resetTimers();
                    state = State.firstSpit;
                    break;
                } else {
                    intake.smartIntake(false);
                }
                break;

            case firstSpit:
                // Robot movement logic
                if (vf.at_pose_deg(first_spit_target, 35, 13, 20) || state_timer.milliseconds() > 2000) {
                    intake.reverseDown();
                    intake.reverse();
                    vf.pid_to_point(first_spit_target, 45, 1);
                    if (event_scheduler.during("Spit sample", 300)) {
                        resetTimers();
                        arm.intakeSpecimen();
                        wheelControl.stop();
                        state = State.intakeSample;
                    }
                } else {
                    path.follow_pid_to_point(first_spit_target, first_spit_angle);
                }

                // Controls reversing
                if (state_timer.milliseconds() < 300 && !intake.hasCorrectSample(true)) {
                    intake.reverse();
                    intake.reverseDown();
                } else if (state_timer.milliseconds() < 70) {
                    intake.reverse();
                    intake.reverseDown();
                } else if (state_timer.milliseconds() > 200) {
                    intake.up();
                    intake.intake();
                } else {
                    intake.stop();
                    intake.up();
                }

                // Retract slides to not bump into bar
                // Extend as late as possible
                if (state_timer.milliseconds() < 800) {
                    horizontalSlides.setPosition(0);
                } else {
                    horizontalSlides.setPosition(horizontal_pos);
                }
                break;

            case intakeSample:
                // Get intake target
                if (intake_state == 1) {
                    intake_target = new Point(intake_sample_x_1,intake_sample_y_1);
                } else if (intake_state == 2) {
                    intake_target = new Point(intake_sample_x_2,intake_sample_y_2);
                } else {
                    intake_target = new Point(intake_sample_x_3,intake_sample_y_3);
                }

                // Move to intake positions
                vf.pid_to_point(intake_target, target_angle_intake, 1);

                // Extend slides
                if (state_timer.milliseconds() > 300) {
                    horizontalSlides.setPosition(horizontal_pos);
                    intake.down();
                    intake.intake();
                } else {
                    horizontalSlides.setPosition(0);
                }

                // Stop when has sample or timer failsafe
                if (intake.hasCorrectSample(false) || state_timer.milliseconds() > 1200){
                    intake_state++;
                    resetTimers();
                    state = State.spitSample;
                }

                break;

            case spitSample:
                // PID to spit
                horizontalSlides.setPosition(-300);
                if (Math.abs(vf.get_heading()) > 55) {
                    vf.pid_to_point(new Point(27, 30), target_angle_spit, 1);
                } else {
                    wheelControl.stop();
                }

                // Start spit at some angle
                if (odometry.opt.get_heading() < 75) {
                    intake.reverseDown();
                    intake.reverse();
                    horizontalSlides.setPosition(horizontal_pos);
                    // Stop after 300 ms
                    if (event_scheduler.during("spit", 300)) {
                        resetTimers();
                        intake.down();
                        intake.stop();
                        horizontalSlides.setPosition(0);
                        if (intake_state >= 4) {
                            state = State.goToSpecimen;
                        } else {
                            state = State.intakeSample;
                        }
                    }
                }
                break;

            case goToSpecimen:
                arm.intakeSpecimen();
                arm.openClaw();
                path.follow_pid_to_point(get_specimen_target, 0);

                if (state_timer.milliseconds() > 500){
                    lift.intakeSpecimen();
                }

                if (vf.at_pose_deg(get_specimen_target, 0, 10, 10)) {
                    arm.intakeSpecimen();
                    horizontalSlides.setPosition(0);
                    intake.down();
                    intake.stop();
                    resetTimers();
                    state = State.pickupSpecimen;
                }

                if (state_timer.milliseconds() >= 1500) {
                    arm.intakeSpecimen();
                    horizontalSlides.setPosition(0);
                    intake.down();
                    intake.stop();
                    resetTimers();
                    state = State.pickupSpecimen;
                }

                break;

            case pickupSpecimen:
                wheelControl.drive_relative(-0.7, 0, 0, 1);
                if (vf.get_x() < 12) {
                    if (event_scheduler.during("pickup", 150)) {
                        wheelControl.stop();
                        lift.toHighChamber();
                        arm.closeClaw();
                        arm.outtakeSpecimen1();
                        resetTimers();
                        if (deposit_state >= 6) {
                            state = State.depositSample;
                        } else {
                            state = State.depositPid;
                        }
                    }
                }
                break;

            case depositPid:
                if (state_timer.milliseconds() < 50) break;

                // Two step PID to not crash into bar
                if (vf.get_y()<deposit_target.y-pid_tuner_1) {
                    vf.pid_to_point(new Point(deposit_target.x-pid_tuner_2, deposit_target.y), 0, power);
                } else {
                    vf.pid_to_point(deposit_target, 0, power);
                }

                // Deposit when there
                if (state_timer.milliseconds() > 3000 || sensors.get_front_dist() < dist_thresh) {
                    resetTimers();
                    intake.up();
                    state = State.deposit;
                }

                break;

            case deposit:
                wheelControl.drive_relative(-1, 0, 0, 1);
                if (state_timer.milliseconds() > 100) {
                    lift.setPosition(0);
                    arm.openClaw();
                    arm.outtakeSpecimen1();
                    deposit_target.y -= 2.5;
                    deposit_state++;
                    resetTimers();
                    if (deposit_state < 6) {
                        state = State.goToSpecimen;
                    } else {
                        state = State.park;
                    }
                }
                break;

            case depositSample:
                if (state_timer.milliseconds()>50) {
                    path.follow_pid_to_point(sample_deposit, 100);
                    arm.outtakeSample();
                    lift.toHighBasket();
                }

                if (path.at_point(sample_deposit, 5) || state_timer.milliseconds()>3000){
                    arm.openClaw();
                    resetTimers();
                    state = State.park;
                }
                if (odometry.opt.get_y()>115) {
                    arm.openClaw();
                    resetTimers();
                    state = State.park;
                }
                break;

            case park:
                if (odometry.opt.get_y()<80) {
                    arm.toIdlePosition();
                    lift.setPosition(0);
                }
                if (state_timer.milliseconds()>100) vf.pid_to_point(new Point(13, 48), 90, 1);
                horizontalSlides.setPosition(horizontal_pos);
                break;
        }

        telemetry.addData("X position: ", odometry.opt.get_x());
        telemetry.addData("Y position: ", odometry.opt.get_y());
        telemetry.addData("Heading: ", odometry.opt.get_heading());
        telemetry.addData("D value: ", vf.T);
        telemetry.addData("Intake State", intake_state);
        telemetry.addData("Motor position: ", lift.getPosition());
        telemetry.addData("Horizontal Motor position: ", horizontalSlides.horizontalSlidesMotor.getCurrentPosition());
        telemetry.addData("State: ", state);
        telemetry.addData("Back touch", sensors.isTouchBack());

        lift.update();
        telemetry.update();
        horizontalSlides.update();
    }
}