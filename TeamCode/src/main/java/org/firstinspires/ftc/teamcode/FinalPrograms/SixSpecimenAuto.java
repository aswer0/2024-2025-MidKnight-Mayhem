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
    public static double sample_x = 24.5;
    public static double sample_y = 40;

    public static double intake_sample_x = 27;
    public static double intake_sample_y = 38;

    public static double first_spit_x = 27;
    public static double first_spit_y = 44;
    public static double first_spit_angle = 40;

    public static double pos = 650;
    public static double dist_thresh = 2.5;
    public static double intake_dist_thresh = 2.5;

    public static double horizontal_pos = -450;
    public static double target_angle_intake = 140;
    public static double target_angle_spit = 22;

    public static double power = 1;

    public static double target_x = 47; //36.2
    public static double target_y = 65;

    public static double deposit_x = 47;
    public static double deposit_y = 60;

    public static double set_pos_tolerance = 1;
    public static double ticks_per_inch = 70;
    public static double sub_spit_angle = 60;

    double intake_state = 0;
    double deposit_state = 0;

    double sub_intake_slide_pos;

    Point target;
    Point get_specimen_target;
    Point get_sample_target;
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

        target = new Point(target_x, target_y);
        get_specimen_target = new Point(15, 28);
        get_sample_target = new Point(sample_x, sample_y);

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
        arm.toAutoStartPosition();
        arm.closeClaw();
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

        if (!previousGamepad1.dpad_left && currentGamepad1.dpad_left){
            sub_intake.x -= set_pos_tolerance;
            if (sub_intake.x < -sub_intake_limit.x/2){
                sub_intake.x = -sub_intake_limit.x/2;
            }
        }
        if (!previousGamepad1.dpad_right && currentGamepad1.dpad_right){
            sub_intake.x += set_pos_tolerance;
            if (sub_intake.x > sub_intake_limit.x/2){
                sub_intake.x = sub_intake_limit.x/2;
            }
        }
        if (!previousGamepad1.dpad_up && currentGamepad1.dpad_up){
            sub_intake.y += set_pos_tolerance;
        }
        if (!previousGamepad1.dpad_down && currentGamepad1.dpad_down){
            sub_intake.y -= set_pos_tolerance;
        }

        telemetry.addData("Alliance", alliance);
        telemetry.addLine();
        telemetry.addData("intake x", sub_intake.x);
        telemetry.addData("intake y", sub_intake.y);

    }

    public void resetTimers() {
        state_timer.reset();
        event_scheduler.clear();
    }

    @Override
    public void start() {
        resetTimers();
    }

    @Override
    public void loop() {
        intake.closeDoor();
        odometry.opt.update();
        lift.update();
        horizontalSlides.update();

        switch (state) {
            case firstPid:
                intake.up();
                intake.stop();
                arm.closeClaw();
                lift.toBackChamber1Pos();
                arm.backOuttakeSpecimen1();
                horizontalSlides.setPosition(-sub_intake.x*ticks_per_inch);

                if (state_timer.milliseconds() >= 100) {
                    vf.pid_to_point(new Point(target_x, target_y + sub_intake.y), 180, 1);
                }

                if (sensors.isTouchBack() || state_timer.milliseconds() > 3000) {
                    deposit_state++;
                    resetTimers();
                    sub_intake_slide_pos = -sub_intake.x*ticks_per_inch;
                    state = State.subIntake;
                }
                break;

            case subIntake: //also back deposit
                wheelControl.stop();
                lift.toBackChamber2Pos();
                arm.backOuttakeSpecimen2();
                horizontalSlides.setPosition(sub_intake_slide_pos);

                if (state_timer.milliseconds() < 150) break;

                if (state_timer.milliseconds() > 250) arm.openClaw();

                if (Math.abs(horizontalSlides.horizontalSlidesMotor.getCurrentPosition() + sub_intake.x * ticks_per_inch) <= 25) {
                    event_scheduler.createIfNew("Extend intake slides");
                    intake.down();
                }

                if (event_scheduler.during("Extend intake slides", 200)) {
                    sub_intake_slide_pos = horizontal_pos;
                }

                if (intake.hasCorrectSample(false) || state_timer.milliseconds() >= 3000) {
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
                if (state_timer.milliseconds() < 200) {
                    lift.intakeSpecimen();
                    wheelControl.drive_relative(0.5, 0, -0.5, 1);
                } else {
                    vf.pid_to_point(new Point(first_spit_x, first_spit_y), first_spit_angle, 0.5);
                }
                //vf.at_angle(first_spit_angle, 5) && vf.at_point(new Point(first_spit_x, first_spit_y), 5)
                if (odometry.opt.get_heading()<70 || state_timer.milliseconds() > 2000) {;
                    intake.reverseDown();
                    intake.reverse();
                    if (event_scheduler.during("Spit sample", 300)) {
                        resetTimers();
                        state = State.intakeSample;
                    }
                } else {
                    if (state_timer.milliseconds() < 800) {
                        horizontalSlides.setPosition(0);
                    } else {
                        horizontalSlides.setPosition(horizontal_pos);
                    }
                    intake.up();
                    intake.stop();
                }

                break;

            case intakeSample:
                if (state_timer.milliseconds() < 1000) {
                    vf.pid_to_point(new Point(intake_sample_x,intake_sample_y), target_angle_intake, 0.7);
                    if (state_timer.milliseconds() > 500) {
                        horizontalSlides.setPosition(horizontal_pos);
                        intake.down();
                        intake.intake();
                    }
                } else {
                    horizontalSlides.setPosition(horizontal_pos);
                    intake.down();
                    intake.intake();
                    if (state_timer.milliseconds() < 1500) {
                        wheelControl.drive_relative(-0.2, -0.1, 0, 1);
                    }

                    if (intake.hasCorrectSample(false) || state_timer.milliseconds() > 2000){ //original 2500
                        intake_state++;
                        resetTimers();
                        state = State.setupSpitSample;
                    }
                }

                break;

            case setupSpitSample:
                arm.toIdlePosition();
                intake.reverseDown();

                vf.pid_to_point(new Point(30, 30), target_angle_spit, 0.6);
                horizontalSlides.setPosition(-200);

                if (odometry.opt.get_heading()<40 || state_timer.milliseconds()> 1000){ //original 1000
                    resetTimers();
                    state = State.spitSample;
                }

                break;

            case spitSample:
                wheelControl.stop();
                horizontalSlides.setPosition(horizontal_pos);
                intake.reverseDown();
                intake.reverse();

                if (state_timer.milliseconds() >= 300){
                    intake_sample_y -= 8;
                    //intake_sample_x += 0.5;

                    horizontalSlides.setPosition(0);
                    resetTimers();
                    if (intake_state >= 4) {
                        state = State.goToSpecimen;
                    } else {
                        state = State.intakeSample;
                    }
                }

                break;

            case goToSpecimen:
                intake.down();
                intake.stop();
                horizontalSlides.setPosition(0);

                vf.pid_to_point(get_specimen_target, 0, 1);

                if (state_timer.milliseconds() > 500){
                    lift.intakeSpecimen();
                    arm.intakeSpecimen();
                }

                if (vf.at_angle_deg(0, 10) && vf.at_point(get_sample_target,6)) { //4
                    resetTimers();
                    state = State.pickupSpecimen;
                }

                if (deposit_state == 1) {
                    if (state_timer.milliseconds() >= 1000) {
                        resetTimers();
                        state = State.pickupSpecimen;
                    }
                } else {
                    if (state_timer.milliseconds() >= 1500) {
                        resetTimers();
                        state = State.pickupSpecimen;
                    }
                }

                break;

            case pickupSpecimen:
                arm.intakeSpecimen();
                horizontalSlides.setPosition(0);
                intake.down();
                intake.stop();

                if (state_timer.milliseconds() < 2000 || sensors.get_front_dist() < dist_thresh) {
                    //vf.pid_to_point(new Point(0, get_specimen_target.y), 0, 0.5);
                    wheelControl.drive_relative(-0.2,0,0,1);
                } else {
                    wheelControl.stop();
                    arm.closeClaw();

                    resetTimers();
                    //target.y -= 1.5; // this has to be tuned better for more space on the right
                    state = State.depositPid;
                }

                break;

            case depositPid:
                //target_x = 46;
                //target_y = 72;
                if (state_timer.milliseconds() >= 100){
                    lift.toHighChamber();
                    arm.outtakeSpecimen1();
                }
                //path.follow_pid_to_point(target, 0);
                if (vf.get_y()<deposit_y-10) {
                    vf.pid_to_point(new Point(deposit_x - 20, deposit_y), 0, 1);
                } else {
                    vf.pid_to_point(new Point(deposit_x, deposit_y), 0, 1);
                }

                if (state_timer.milliseconds() > 3000 || sensors.get_front_dist() < 2.5) {
                    resetTimers();
                    state = State.deposit;
                }

                break;

            case deposit:
                wheelControl.drive_relative(0.1, -0.2, 0, 1);

                intake.stop();
                intake.down();

                arm.openClaw();
                arm.outtakeSpecimen1();

                if (state_timer.milliseconds() >= 125) {
                    lift.setPosition(0);
                }

                if (state_timer.milliseconds() >= 400){
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
                if (state_timer.milliseconds()>100) horizontalSlides.setPosition(horizontal_pos);
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