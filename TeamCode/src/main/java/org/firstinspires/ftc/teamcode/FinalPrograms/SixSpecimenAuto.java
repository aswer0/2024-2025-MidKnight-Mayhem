package org.firstinspires.ftc.teamcode.FinalPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFSimplfied.Path;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Arm;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Alliance;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Sensors;
import org.opencv.core.Point;

@Autonomous
@Config
public class SixSpecimenAuto extends OpMode {
    public static double intake_sample_x = 27;
    public static double intake_sample_y = 34;

    public static double first_spit_x = 24;
    public static double first_spit_y = 44;
    public static double first_spit_angle = 70;

    public static double pos = 650;
    public static double dist_thresh = 2.5;
    public static double intake_dist_thresh = 2.5;

    public static double horizontal_pos = -450;
    public static double target_angle_intake = 140;
    public static double target_angle_spit = 22;

    public static double power = 1;
    public static double gvf_speed = 0.1;
    public static double gvf_thresh = 13;

    public static double target_x = 50; //36.2
    public static double target_y = 90;

    public static double set_pos_tolerance = 1;
    public static double ticks_per_inch = 25;
    public static double sub_spit_angle = 60;
    double intake_state = 0;

    Point target;
    Point get_specimen_target;
    Point sub_intake = new Point(0, 0);
    Point sub_intake_limit = new Point(27.5, 21.3);

    ElapsedTime timer;
    ElapsedTime intakeShakeTimer;
    Sensors sensors;

    Odometry odometry;
    WheelControl wheelControl;
    Path path;

    State state = State.pid;

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
        pid,
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
        get_specimen_target = new Point(12.875, 28);

        timer = new ElapsedTime();
        intakeShakeTimer = new ElapsedTime();
        sensors = new Sensors(hardwareMap, telemetry);

        odometry = new Odometry(hardwareMap, 180, 7.875, 72, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);

        path = new Path(follow_path, wheelControl, odometry, telemetry, gvf_speed, gvf_thresh, 180, power);

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
            sub_intake.y -= set_pos_tolerance;
        }
        if (!previousGamepad1.dpad_down && currentGamepad1.dpad_down){
            sub_intake.y -= set_pos_tolerance;
        }

        telemetry.addData("Alliance", alliance);
        telemetry.addLine();
        telemetry.addData("intake x", sub_intake.x);
        telemetry.addData("intake y", sub_intake.y);

    }

    @Override
    public void start() {
        timer.reset();
        intakeShakeTimer.reset();
    }

    @Override
    public void loop() {
        intake.closeDoor();
        odometry.opt.update();

        switch (state) {
            case pid:
                intake.up();
                intake.stop();
                arm.closeClaw();

                lift.toBackChamber1Pos();

                if (intake_state == 0){
                    arm.backOuttakeSpecimen1();
                    horizontalSlides.setPosition(-sub_intake.x*ticks_per_inch);
                    if (timer.milliseconds() >= 100){
                        path.follow_pid_to_point(new Point(target_x, target_y+sub_intake.y), 180); //+ sub_intake.x
                    }
                    if (odometry.opt.get_x()>(target_x-12) && sensors.get_back_dist()<1){
                        state = State.subIntake;
                    }
                }
                else{
                    if (timer.milliseconds() >= 100){
                        arm.outtakeSpecimen1();
                    }
                    //path.follow_pid_to_point(target, 0);
                    if (odometry.opt.get_y()<target_y-36) {
                        path.follow_pid_to_point(new Point(target_x - 12, target_y), 0);
                    } else {
                        path.follow_pid_to_point(target, 0);
                    }
                    if (sensors.get_front_dist() <= dist_thresh && odometry.opt.get_heading() > -50 && odometry.opt.get_heading() < 50) {
                        intake_state++;
                        timer.reset();
                        state = State.deposit;
                    }
                }

                //if (intake_state >= 1){
                //    path.update(0);
                //}
                if (sensors.get_front_dist() <= dist_thresh && odometry.opt.get_heading() > -50 && odometry.opt.get_heading() < 50) {
                    intake_state++;
                    timer.reset();
                    if (intake_state == 1){
                        state = State.subIntake;
                    }
                    else{
                        state = State.deposit;
                    }
                }
                if (timer.milliseconds() > 5000) {
                    timer.reset();
                    intake_state++;
                    if (intake_state == 1){
                        state = State.subIntake;
                    }
                    else{
                        state = State.deposit;
                    }
                }

                break;

            case subIntake: //also back deposit
                lift.toBackChamber2Pos();
                arm.backOuttakeSpecimen2();
                horizontalSlides.setPosition(sub_intake.y*ticks_per_inch);

                if (Math.abs(lift.getPosition()-lift.backChamber1Pos)<10 || timer.milliseconds()>500) {
                    arm.openClaw();
                    if (Math.abs(horizontalSlides.horizontalSlidesMotor.getCurrentPosition() - sub_intake.y * ticks_per_inch) <= 10) {
                        intake.down();
                        intake.intake();
                    }

                    if (intake.hasCorrectSample(false) || timer.milliseconds() >= 2000) {
                        intake.stop();
                        intake.up();
                        horizontalSlides.setPosition(0);
                        timer.reset();
                        state =State.firstSpit;
                    }
                }
//                if (horizontalSlides.horizontalSlidesMotor.getCurrentPosition() <= 10 && intake.hasCorrectSample(false)){
//                    timer.reset();
//                    state = State.deposit;
//                }

                break;

            case firstSpit:
                path.follow_pid_to_point(new Point(first_spit_x,first_spit_y), first_spit_angle);
                if (timer.milliseconds()>1200) { //path.at_point(new Point(first_spit_x,first_spit_y),5) ||
                    horizontalSlides.setPosition(horizontal_pos);
                    if (horizontalSlides.getPosition()<horizontal_pos+50) {
                        intake.reverseDown();
                        intake.reverse();
                        if (timer.milliseconds()>1200+500) {
                            state = State.intakeSample;
                            intake_state++;
                            horizontalSlides.setPosition(0);
                            timer.reset();
                        }
                    }
                }

                break;

            case deposit:
                intake.stop();
                intake.down();

                arm.openClaw();
                arm.outtakeSpecimen1();

                if (timer.milliseconds() >= 125) {
                    lift.setPosition(100);
                }

                if (timer.milliseconds() >= 400){
                    // im not sure if it will intake 3 times then move to go to specimen state, check this
                    if (intake_state >= 2 && intake_state < 6){
                        timer.reset();
                        intakeShakeTimer.reset();
                        state = State.intakeSample;
                    }
                    else {
                        timer.reset();
                        state = State.goToSpecimen;
                    }
                }

                break;

            case goToSpecimen:
                intake.down();
                intake.stop();
                horizontalSlides.setPosition(0);

                if (intake_state == 1){
                    path.follow_pid_to_point(get_specimen_target, sub_spit_angle);
                }
                else{
                    path.follow_pid_to_point(get_specimen_target, 0);
                }

                if (timer.milliseconds() > 500){
                    lift.intakeSpecimen();
                    arm.intakeSpecimen();
                }

                if (path.at_point(get_specimen_target, 5)) { //4
                    wheelControl.drive(0, 0, 0, 0, 0.7);
                    timer.reset();
                    state = State.pickupSpecimen;
                }

                break;

            case pickupSpecimen:
                arm.intakeSpecimen();
                horizontalSlides.setPosition(0);

                if (intake_state == 1){
                    if (intake.hasCorrectSample(false)){
                        intake.reverseDown();
                        intake.reverse();
                    }
                    path.follow_pid_to_point(get_specimen_target, 0);
                }
                else{
                    intake.down();
                    intake.stop();
                }

                if (!intake.hasCorrectSample(false)){
                    if (sensors.get_back_dist() >= intake_dist_thresh || timer.milliseconds() < 670) {
                        wheelControl.drive(0.5, 0, 0, 0, 0.7);
                    } else {
                        wheelControl.drive(0, 0, 0, 0, 0);
                        arm.closeClaw();

                        timer.reset();
                        target.y -= 2.5; // this has to be tuned better for more space on the right
                        state = State.pid;
                    }
                }

                break;

            case intakeSample:
                if (intake_state > 5){
                    state = State.goToSpecimen;
                }

                path.follow_pid_to_point(new Point(intake_sample_x,intake_sample_y), target_angle_intake);

                intake.down();
                intake.intake();

                if (timer.milliseconds()>1000) {
                    horizontalSlides.setPosition(horizontal_pos);

                    if (intake.hasCorrectSample(false) || timer.milliseconds() >= 2000){ //original 2500
                        timer.reset();
                        state = State.setupSpitSample;
                    }
                }

                break;

            case setupSpitSample:
                path.set_original_hp();
                arm.toIdlePosition();
                intake.reverseDown();

                path.follow_pid_to_point(new Point(30, 30), target_angle_spit);
                horizontalSlides.setPosition(0);

                if (odometry.opt.get_heading()<60 || timer.milliseconds()> 1000){ //original 1000
                    wheelControl.drive(0,0,0,0, 0);
                    timer.reset();
                    state = State.spitSample;
                }

                break;

            case spitSample:
                if (timer.milliseconds()>0) horizontalSlides.setPosition(-470);
                intake.reverseDown();
                intake.reverse();

                if (timer.milliseconds() >= 600){
                    intake_sample_y -= 8;
                    //intake_sample_x += 0.5;
                    intake_state++;

                    horizontalSlides.setPosition(-100);
                    timer.reset();
                    state = State.intakeSample;
                }

                break;

            case park:
                path.follow_pid_to_point(new Point(24, 48), 60);
                if (timer.milliseconds()>100) horizontalSlides.setPosition(horizontal_pos);

                break;
        }

        telemetry.addData("X position: ", odometry.opt.get_x());
        telemetry.addData("Y position: ", odometry.opt.get_y());
        telemetry.addData("Heading: ", odometry.opt.get_heading());
        telemetry.addData("D value: ", path.get_d());
        telemetry.addData("Deposit State", intake_state);
        telemetry.addData("Motor position: ", lift.getPosition());
        telemetry.addData("Horizontal Motor position: ", horizontalSlides.horizontalSlidesMotor.getCurrentPosition());
        telemetry.addData("State: ", state);
        telemetry.addData("Back Distance", sensors.get_back_dist());

        lift.update();
        telemetry.update();
        horizontalSlides.update();
    }
}
