package org.firstinspires.ftc.teamcode.FinalPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.BCPath;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFSimplfied.BezierCurve;
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

@Config
@Autonomous
public class NathansFivePushSpecYay extends OpMode {
    public static double sample_x = 8;
    public static double sample_y = 120;

    public static double clampValue = -180;

    public static double pos = 650;
    public static double dist_thresh = 2.5;

    public static double target_x = 50; //36.2
    public static double target_y = 90;
    public static double intake_state = 0;

    public static double power = 1;
    public static double at_end_dist = 5;
    public static double gvf_speed = 0.1;
    public static double gvf_thresh = 13;

    Point target;
    Point get_specimen_target;
    Point preload_sample;

    ElapsedTime timer;
    ElapsedTime intakeShakeTimer;
    Sensors sensors;

    Odometry odometry;
    WheelControl wheelControl;
    Path path;

    Point[] follow_path1 = {
            new Point(23.15,46.75),
            new Point(17.42, 34.4),
            new Point(116.66, 29.11),
            new Point(10.8, 24.26),
    };
    Point[] follow_path2 = {
            new Point(10.8,24.26),
            new Point(110.9, 28),
            new Point(44.55, 8.38),
            new Point(8.8, 14.77),
    };
    Point[] follow_path3 = {
            new Point(12.13,14.33),
            new Point(110.9, 11.69),
            new Point(36.8, 7.28),
            new Point(36.83, 7.28),
    };
    BezierCurve[] follow_path = {
            new BezierCurve(follow_path1),
            new BezierCurve(follow_path2),
            new BezierCurve(follow_path3),
    };

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
        gvfPush,
        gvfPathChange,
        depositPreloadSample,
        park
    }

    @Override
    public void init() {
        target = new Point(target_x, target_y);
        get_specimen_target = new Point(14.875, 30);
        preload_sample = new Point(sample_x, sample_y);

        timer = new ElapsedTime();
        intakeShakeTimer = new ElapsedTime();
        sensors = new Sensors(hardwareMap, telemetry);

        odometry = new Odometry(hardwareMap, 0, 7.875, 66, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);

        path = new Path(follow_path1, wheelControl, odometry, telemetry, gvf_speed, gvf_thresh, 180, power);

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
                        path.pid_to_point(target, 0);
                    }
                }
                else{
                    if (timer.milliseconds() >= 100){
                        arm.outtakeSpecimen1();
                    }
                    //path.follow_pid_to_point(target, 0);
                    if (odometry.opt.get_y()<target_y-36) {
                        path.pid_to_point(new Point(target_x - 12, target_y), 0);
                    } else {
                        path.pid_to_point(target, 0);
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
                intake.stop();
                intake.down();

                arm.openClaw();

                if (timer.milliseconds() >= 125) {
                    lift.setPosition(0);
                    arm.outtakeSpecimen2();
                }

                if (timer.milliseconds() >= 300){
                    // im not sure if it will intake 3 times then move to go to specimen state, check this
                    if (intake_state >= 2 && intake_state <= 5){
                        timer.reset();
                        intakeShakeTimer.reset();

                        path.set_path(follow_path[(int)(intake_state-2)]);
                        state = State.gvfPush;
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
                path.pid_to_point(get_specimen_target, 0);

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

            case gvfPush:
                path.update(0);

                if (path.at_point(at_end_dist)){
                    intake_state++;
                    if (intake_state == 5){
                        state = State.pickupSpecimen;
                    }
                    else{
                        state = State.deposit;
                    }
                }
                break;

            /*==================================*/
            case depositPreloadSample:
                if (timer.milliseconds()>50) {
                    path.pid_to_point(preload_sample, 110);
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
                if (timer.milliseconds()>100) path.pid_to_point(new Point(12, 48), 90);

                break;
        }

        telemetry.addData("X position: ", odometry.opt.get_x());
        telemetry.addData("Y position: ", odometry.opt.get_y());
        telemetry.addData("Heading: ", odometry.opt.get_heading());
        telemetry.addData("Deposit State", intake_state);
        telemetry.addData("Motor position: ", lift.getPosition());
        telemetry.addData("Horizontal Motor position: ", horizontalSlides.horizontalSlidesMotor.getCurrentPosition());
        telemetry.addData("State: ", state);
        telemetry.addData("timer", timer.milliseconds());

        lift.update();
        telemetry.update();
        horizontalSlides.update();
    }

    public void stop() {
        arm.openClaw();
    }
}