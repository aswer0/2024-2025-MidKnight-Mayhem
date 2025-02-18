package org.firstinspires.ftc.teamcode.FinalPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.VectorField;
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
    public static double intake_sample_x_1 = 27;
    public static double intake_sample_y_1 = 38;
    public static double intake_sample_x_2 = 27;
    public static double intake_sample_y_2 = 30;
    public static double intake_sample_x_3 = 27;
    public static double intake_sample_y_3 = 22;
    Point intake_target;

    public static double first_spit_x = 27;
    public static double first_spit_y = 44;
    public static double first_spit_angle = 30;

    public static double dist_thresh = 2.5;

    public static double horizontal_pos = -450;
    public static double target_angle_intake = 140;
    public static double target_angle_spit = 22;

    public static double power = 1;

    public static double target_x = 45; //36.2
    public static double target_y = 65;
    public Point hang_target;
    public Point deposit_target;

    public static double set_pos_tolerance = 1;
    public static double ticks_per_inch = 70;

    double intake_state = 0;
    double deposit_state = 0;

    double sub_intake_slide_pos;

    Point get_specimen_target;
    Point sub_intake = new Point(0, 0);
    Point sub_intake_limit = new Point(27.5, 21.3);

    ElapsedTime state_timer;
    EventScheduler event_scheduler;
    Sensors sensors;

    Odometry odometry;
    WheelControl wheelControl;
    VectorField vf;

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
        setupSpitSample,
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
        get_specimen_target = new Point(15, 28);

        state_timer = new ElapsedTime();
        event_scheduler = new EventScheduler();
        sensors = new Sensors(hardwareMap, telemetry);

        odometry = new Odometry(hardwareMap, 180, 8.875, 65, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);

        vf = new VectorField(wheelControl, odometry);

        wheelControl.change_mode(DcMotor.ZeroPowerBehavior.BRAKE);

        lift = new Lift(hardwareMap);
        intake = new Intake(hardwareMap, sensors);
        horizontalSlides = new HorizontalSlides(hardwareMap);
        arm = new Arm(hardwareMap);
        arm.backOuttakeSpecimen1();
        arm.closeClaw();
        intake.up();
        intake.stop();
        intake.closeDoor();
        lift.toBackChamber1Pos();
    }

    @Override
    public void init_loop() {
        if (gamepad1.left_bumper) {
            alliance = Alliance.blue;
        } else if (gamepad1.right_bumper) {
            alliance = Alliance.red;
        }

        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        if (!previousGamepad1.dpad_down && currentGamepad1.dpad_down){
            sub_intake.x -= set_pos_tolerance;
            if (sub_intake.x < -sub_intake_limit.x/2){
                sub_intake.x = -sub_intake_limit.x/2;
            }
        }
        if (!previousGamepad1.dpad_up && currentGamepad1.dpad_up){
            sub_intake.x += set_pos_tolerance;
            if (sub_intake.x > sub_intake_limit.x/2){
                sub_intake.x = sub_intake_limit.x/2;
            }
        }
        if (!previousGamepad1.dpad_left && currentGamepad1.dpad_left){
            sub_intake.y += set_pos_tolerance;
        }
        if (!previousGamepad1.dpad_right && currentGamepad1.dpad_right){
            sub_intake.y -= set_pos_tolerance;
        }

        telemetry.addData("Alliance", alliance);
        telemetry.addLine();
        telemetry.addLine("Sample position (relative to robot)");
        telemetry.addData("intake b/f (+ is forward)", sub_intake.x);
        telemetry.addData("intake l/r (+ is right)", -sub_intake.y);
    }

    public void resetTimers() {
        state_timer.reset();
        event_scheduler.clear();
    }

    @Override
    public void start() {
        resetTimers();
        hang_target = new Point(target_x, target_y+sub_intake.y);
        deposit_target = new Point(hang_target.x, hang_target.y-1.5);
    }

    @Override
    public void loop() {
        odometry.opt.update();
        lift.update();
        horizontalSlides.update();

        switch (state) {
            case firstPid:
                // Reverse hang
                horizontalSlides.setPosition(-sub_intake.x*ticks_per_inch);
                vf.pid_to_point(hang_target, 180, 1);

                // Sub intake when it's there
                if (sensors.isTouchBack() || state_timer.milliseconds() > 2000) {
                    deposit_state++;
                    horizontalSlides.setPosition(-sub_intake.x*ticks_per_inch);
                    wheelControl.stop();
                    lift.toBackChamber2Pos();
                    arm.backOuttakeSpecimen2();
                    resetTimers();
                    state = State.subIntake;
                }
                break;

            case subIntake: //also back deposit
                // Bit of delay so not having 2 samples in possession
                if (state_timer.milliseconds() < 150) break;

                if (state_timer.milliseconds() > 300) {
                    lift.intakeSpecimen();
                    arm.openClaw();
                }

                // Extend intake slides once at correct position
                if (Math.abs(horizontalSlides.horizontalSlidesMotor.getCurrentPosition() + sub_intake.x * ticks_per_inch) <= 25) {
                    event_scheduler.createIfNew("Extend intake slides");
                    intake.down();
                }
                if (event_scheduler.during("Extend intake slides", 300)) {
                    horizontalSlides.setPosition(-450);
                }

                // Move on if correct or timer failsafe
                if (intake.hasCorrectSample(false) || state_timer.milliseconds() >= 1500) {
                    intake.stop();
                    intake.up();
                    intake_state++;
                    state = State.firstSpit;
                    break;
                } else {
                    intake.smartIntake(false);
                }
                break;

            case firstSpit:
                // PID to position
                vf.pid_to_point(new Point(first_spit_x, first_spit_y), first_spit_angle, 0.6);

                // Spit if there
                if (odometry.opt.get_heading()<50 || state_timer.milliseconds() > 2000) {;
                    intake.reverseDown();
                    intake.reverse();
                    if (event_scheduler.during("Spit sample", 300)) {
                        resetTimers();
                        state = State.intakeSample;
                    }
                }

                // Reverse if wrong color
                if (state_timer.milliseconds() < 300 && !intake.hasCorrectSample(true)) {
                    intake.reverse();
                    intake.reverseDown();
                } else {
                    intake.intake();
                    intake.up();
                }

                // Retract slides to not bump into bar
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
                if (state_timer.milliseconds() < 2000 && !vf.at_pose_deg(intake_target, target_angle_intake, 2, 5)) {
                    vf.pid_to_point(intake_target, target_angle_intake, 1);
                } else if (event_scheduler.during("intake", 0, 500)) {
                    wheelControl.drive_relative(-0.2, -0.1, 0, 1);
                }

                // Extend slides
                if (state_timer.milliseconds() > 500) {
                    horizontalSlides.setPosition(horizontal_pos);
                    intake.down();
                    intake.intake();
                }

                // Stop when has sample or timer failsafe
                if (intake.hasCorrectSample(false) || state_timer.milliseconds() > 2500){
                    intake_state++;
                    arm.toIdlePosition();
                    resetTimers();
                    state = State.setupSpitSample;
                }

                break;

            case setupSpitSample:
                // PID to spit
                vf.pid_to_point(new Point(30, 30), target_angle_spit, 0.5);
                horizontalSlides.setPosition(-200);

                if (odometry.opt.get_heading()<45 || state_timer.milliseconds() > 1000){
                    resetTimers();
                    horizontalSlides.setPosition(horizontal_pos);
                    intake.reverseDown();
                    intake.reverse();
                    state = State.spitSample;
                }

                break;

            case spitSample:
                // Transition to pick up specimen or intake more
                if (state_timer.milliseconds() >= 300){
                    horizontalSlides.setPosition(0);
                    resetTimers();
                    if (intake_state >= 4) {
                        intake.down();
                        intake.stop();
                        horizontalSlides.setPosition(0);
                        state = State.goToSpecimen;
                    } else {
                        arm.toIdlePosition();
                        state = State.intakeSample;
                    }
                }

                break;

            case goToSpecimen:
                vf.pid_to_point(get_specimen_target, 0, 1);

                if (state_timer.milliseconds() > 500){
                    lift.intakeSpecimen();
                    arm.intakeSpecimen();
                }

                if (vf.at_pose_deg(get_specimen_target, 0, 4, 4)) {
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
                // PID to pickup specimen and correct y
                if (state_timer.milliseconds() < 1500 || sensors.get_front_dist() > dist_thresh) {
                    vf.pid_to_point(new Point(0, get_specimen_target.y), 0, 0.3);
                } else {
                    wheelControl.stop();
                    arm.closeClaw();
                    lift.toHighChamber();
                    arm.outtakeSpecimen1();
                    resetTimers();
                    state = State.depositPid;
                }

                break;

            case depositPid:
                // Two step PID to not crash into bar
                if (vf.get_y()<deposit_target.y-10) {
                    vf.pid_to_point(new Point(deposit_target.x-15, deposit_target.y), 0, 1);
                } else {
                    vf.pid_to_point(deposit_target, 0, 1);
                }

                // Deposit when there
                if (state_timer.milliseconds() > 2500 || sensors.get_front_dist() < dist_thresh) {
                    resetTimers();
                    intake.up();
                    state = State.deposit;
                }

                break;

            case deposit:
                wheelControl.drive_relative(0.1, -0.9, 0, 1);

                // Decide what to do next
                if (state_timer.milliseconds() >= 100) {
                    lift.setPosition(0);
                    arm.openClaw();
                    arm.outtakeSpecimen1();
                    deposit_state++;
                    resetTimers();
                    if (deposit_state < 6) {
                        state = State.goToSpecimen;
                    } else {
                        state = State.park;
                    }
                }
                break;

            case park:
                vf.pid_to_point(new Point(24, 48), 0, 1);
                horizontalSlides.setPosition(0);
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