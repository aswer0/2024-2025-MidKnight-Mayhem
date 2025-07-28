package org.firstinspires.ftc.teamcode.FinalPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFSimplfied.Path;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Intake.HorizontalSlides;
import org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Outtake.Arm;
import org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Alliance;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Sensors;
import org.opencv.core.Point;

@Autonomous
@Config
public class FivePlusOneAuto extends OpMode {
    public static double sample_x = 8;
    public static double sample_y = 120;

    public static double intake_sample_x1 = 28;
    public static double intake_sample_y1 = 34.75;
    public static double intake_sample_x2 = 27.5;
    public static double intake_sample_y2 = 34;
    public static double intake_sample_x3 = 26.8;
    public static double intake_sample_y3 = 16.5;
    public static double sweep_sample_x = 32;
    public static double sweep_sample_y = 34;

    public static double clampValue = -180;


    public static double pos = 650;
    public static double dist_thresh = 2.5;
    public static double intake_dist_thresh = 2.5;

    public static double horizontal_pos = -450;
    public static double target_angle_intake1 = 136;
    public static double target_angle_spit = 22;

    public static double power = 1;
    public static double gvf_speed = 0.1;
    public static double gvf_thresh = 13;

    public static double target_x = 50; //36.2
    public static double target_y = 90;
    double intake_state = 0;

    Point target;
    Point get_specimen_target;
    Point preload_sample;
    //Point get_sample_target;

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
        setupSpitSample,
        depositPreloadSample,
        park
    }

    @Override
    public void init() {
        intake_sample_x1 = 27;
        sweep_sample_x = 29;
        Point[] follow_path = {
                new Point(11,28),
                new Point(10.3, 65.5),
                new Point(19.2, 68.2),
                new Point(36.5, 75)
        };

        target = new Point(target_x, target_y);
        get_specimen_target = new Point(14.875, 30);
        preload_sample = new Point(sample_x, sample_y); //8 120
        //get_sample_target = new Point(sample_x, sample_y);

        timer = new ElapsedTime();
        intakeShakeTimer = new ElapsedTime();
        sensors = new Sensors(hardwareMap, telemetry);

        odometry = new Odometry(hardwareMap, 0, 7.875, 66, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);

        path = new Path(follow_path, wheelControl, odometry, telemetry, gvf_speed, gvf_thresh, 180, power);

        wheelControl.change_mode(DcMotor.ZeroPowerBehavior.BRAKE);

        lift = new Lift(hardwareMap);
        intake = new Intake(hardwareMap, sensors);
        horizontalSlides = new HorizontalSlides(hardwareMap);
        arm = new Arm(hardwareMap);
        arm.toSpecAutoStartPosition();
        arm.closeClaw();
    }

    @Override
    public void init_loop() {
        if (gamepad1.left_bumper) {
            alliance = Alliance.blue;
        } else if (gamepad1.right_bumper) {
            alliance = Alliance.red;
        }

        telemetry.addData("Alliance", alliance);
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

                lift.toHighChamber();

                if (intake_state == 0){
                    arm.outtakeSpecimen1();
                    if (timer.milliseconds() >= 150){
                        path.follow_pid_to_point(target, 0);
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
                }

                //if (intake_state >= 1){
                //    path.update(0);
                //}
                if (sensors.get_front_dist() <= dist_thresh && odometry.opt.get_heading() > -50 && odometry.opt.get_heading() < 50) {
                    intake_state++;
                    timer.reset();
                    state = State.deposit;
                }

                if (timer.milliseconds() > 5000) {
                    timer.reset();
                    intake_state++;
                    state = State.deposit;
                }

                break;

            case deposit:
                //wheelControl.drive(0,0,0,0,0);
                intake.stop();
                intake.down();

                arm.openClaw();
                //arm.outtakeSpecimen1();

                if (timer.milliseconds() >= 125) {
                    lift.setPosition(0);
                    arm.outtakeSpecimen2();
                }

                if (timer.milliseconds() >= 300){
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
                path.follow_pid_to_point(get_specimen_target, 0);

                if (timer.milliseconds() > 700){
                    lift.intakeSpecimen();
                    arm.intakeSpecimen();
                }

                if (path.at_point(get_specimen_target, 6)) { //4
                    wheelControl.drive(0, 0, 0, 0, 0.7);
                    timer.reset();
                    state = State.pickupSpecimen;
                }

                break;

            case pickupSpecimen:
                intake.down();
                intake.stop();
                arm.intakeSpecimen();
                horizontalSlides.setPosition(0);

                if (intake_state >=5 && timer.milliseconds()<400) {
                    wheelControl.drive(0.5,0,0,0,0.7);
                } else if (timer.milliseconds() < 600) { //sensors.get_back_dist() >= intake_dist_thresh &&
                    wheelControl.drive(0.5,0,0,0,0.7);
                } else {
                    wheelControl.drive(0, 0, 0, 0, 0);
                    arm.closeClaw();

                    timer.reset();
                    target.y -= 2.5; // this has to be tuned better for more space on the right

                    /*==================================*/
                    if (intake_state >= 8){
                        state = State.depositPreloadSample;
                    }
                    else{
                        state = State.pid;
                    }
                    /*==================================*/
                }

                break;

            case intakeSample:
                int time;
                if (intake_state > 4){
                    state = State.goToSpecimen;
                } if (intake_state==2 || intake_state==3) {
                    path.follow_pid_to_point(new Point(intake_sample_x1, intake_sample_y1), target_angle_intake1);
                } if (intake_state==4) {
                    path.follow_pid_to_point(new Point(intake_sample_x3, intake_sample_y3), target_angle_intake1);
                }

                //intake.down();
                intake.intake();

                if (intake_state >= 3){
                    time = 500;
                }
                else{
                    time = 900;
                }
                if (timer.milliseconds()>time) {
                    if (horizontalSlides.getPosition()<clampValue) {
                        intake.down();
                    } else {
                        intake.reverseDown();
                    }
                    horizontalSlides.setPosition(horizontal_pos);

                    if (intake.hasCorrectSample(false)){ //original 2500
                        timer.reset();
                        state = State.setupSpitSample;
                    }

                    if (intake_state >=3 && timer.milliseconds() >= 1500) {
                        timer.reset();
                        state = State.setupSpitSample;
                    } else if (timer.milliseconds()>=1900) {
                        timer.reset();
                        state = State.setupSpitSample;
                    }
                }

                break;

            case setupSpitSample:
                path.set_original_hp();
                arm.toSpecIdlePosition();
                //intake.reverseDown();
                intake.down();

                path.follow_pid_to_point(new Point(30, 30), target_angle_spit);
                horizontalSlides.setPosition(horizontal_pos);

                if (odometry.opt.get_heading()<70 || timer.milliseconds()> 900){ //original 1000
                    wheelControl.drive(0,0,0,0, 0);
                    timer.reset();
                    horizontalSlides.setPosition(horizontal_pos);
                    intake.reverseDown();
                    intake.reverse();
                    state = State.spitSample;
                }

                break;

            case spitSample:
                if (timer.milliseconds()>0) horizontalSlides.setPosition(horizontal_pos);
                intake.reverseDown();
                if (horizontalSlides.getPosition()<-100) intake.reverse();

                if (timer.milliseconds() >= 300){
                    intake_sample_y1 -= 7.25; //8
                    intake_sample_x1 += 1.75;
                    target_angle_intake1 += 3.5;
                    intake_state++;

                    horizontalSlides.setPosition(-100);
                    timer.reset();
                    state = State.intakeSample;
                }

                break;

            /*==================================*/
            case depositPreloadSample:
                if (timer.milliseconds()>50) {
                    path.follow_pid_to_point(preload_sample, 110);
                    arm.outtakeSample();
                    lift.toHighBasket();
                }

                if (path.at_point(preload_sample, 5) || timer.milliseconds()>3000){
                    wheelControl.drive(0,0,0,0,0);
                    arm.openClaw();
                    state = State.park;
                }
                if (odometry.opt.get_y()>115) arm.openClaw();
                break;
            /*==================================*/

            case park:
                if (odometry.opt.get_y()<80) {
                    arm.toIdlePosition();
                    lift.setPosition(0);
                }
                if (timer.milliseconds()>100) path.follow_pid_to_point(new Point(12, 48), 90);
                horizontalSlides.setPosition(horizontal_pos);

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
        //telemetry.addData("Back Distance", sensors.get_back_dist());
        telemetry.addData("timer", timer.milliseconds());

        lift.update();
        telemetry.update();
        horizontalSlides.update();
    }

    public void stop() {
        arm.openClaw();
    }
}