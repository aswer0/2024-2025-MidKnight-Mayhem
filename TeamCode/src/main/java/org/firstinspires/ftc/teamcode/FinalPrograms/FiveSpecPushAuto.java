package org.firstinspires.ftc.teamcode.FinalPrograms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.BCPath;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVFSimplfied.Path;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Intake.HorizontalSlides;
import org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Outtake.Arm;
import org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Alliance;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Sensors;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.opencv.core.Point;

@Autonomous
@Config
public class FiveSpecPushAuto extends OpMode {
    // region -- Constants --
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

    Point target;
    Point get_specimen_target;
    Point preload_sample;

    BCPath[] pushPaths = {
        new BCPath(new Point[][] {
                {
                        new Point(8, 40),
                        new Point(45, 40),
                        new Point(50, 30)
                }
        }),
        new BCPath(new Point[][] {
                {
                        new Point(8, 30),
                        new Point(45, 35),
                        new Point(50, 20)
                }
        }),
        new BCPath(new Point[][] {
                {
                        new Point(8, 20),
                        new Point(45, 25),
                        new Point(50, 10)
                }
        })
    };

    //Point get_sample_target;
    // endregion
    // region -- State --
    // pidToBar -> deposit -> goToSpecimen / intakeSample (pick up from lines)
    // goToSpecimen -> pickupSpecimen -> pidToBar
    // intakeSample -> goToSpecimen (if done w/ samples) / setupSpitSample
    // setupSpitSample -> spitSample

    enum State {
        pidToBar,
        goToSpecimen,
        pickupSpecimen,
        deposit,
        goToPushSample,
        pushSampleBack,
        park
    }

    ElapsedTime timer;
    ElapsedTime intakeShakeTimer;
    Alliance alliance;
    State state;
    Sensors sensors;
    Odometry odometry;

    int intakeState = 0;
    int depositState = 0;

    TelemetryPacket tp;
    // endregion
    // region -- Control --
    WheelControl wheelControl;
    Path path;
    VectorField gvfPath;

    Lift lift;
    Intake intake;
    HorizontalSlides horizontalSlides;
    Arm arm;
    // endregion
    // region -- Logic --
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

        alliance = Alliance.red;
        state = State.pidToBar;
        timer = new ElapsedTime();
        intakeShakeTimer = new ElapsedTime();
        sensors = new Sensors(hardwareMap, telemetry);
        odometry = new Odometry(hardwareMap, 0, 7.875, 66, "OTOS");

        wheelControl = new WheelControl(hardwareMap, odometry);
        wheelControl.change_mode(DcMotor.ZeroPowerBehavior.BRAKE);
        path = new Path(follow_path, wheelControl, odometry, telemetry, gvf_speed, gvf_thresh, 180, power);
        gvfPath = new VectorField(wheelControl, odometry);

        lift = new Lift(hardwareMap);
        intake = new Intake(hardwareMap, sensors);
        horizontalSlides = new HorizontalSlides(hardwareMap);
        arm = new Arm(hardwareMap);
        arm.toAutoStartPosition();
        arm.closeClaw();
        intake.closeDoor();
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
        odometry.opt.update();
        tp = new TelemetryPacket();

        switch (state) {
            case pidToBar:
                intake.up();
                intake.stop();
                arm.closeClaw();

                lift.toHighChamber();

                if (intakeState == 0 || intakeState >= 4){
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

                if (sensors.get_front_dist() <= dist_thresh && odometry.opt.get_heading() > -50 && odometry.opt.get_heading() < 50) {
                    timer.reset();
                    depositState++;
                    state = State.deposit;
                }

                if (intakeState == 0 && timer.milliseconds() > 1000) {
                    timer.reset();
                    depositState++;
                    state = State.deposit;
                } else if (timer.milliseconds() > 1800) {
                    timer.reset();
                    depositState++;
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
                    if (depositState == 2){
                        timer.reset();
                        intakeShakeTimer.reset();
                        gvfPath.setPath(pushPaths[0], 0, false);
                        intakeState++;
                        state = State.goToPushSample;
                    } else {
                        timer.reset();
                        state = State.goToSpecimen;
                    }
                }

                break;

            case goToPushSample:
                if (depositState == 2 && timer.milliseconds() < 500) {
                    wheelControl.drive_relative(-1, 1, 0, 1);
                } else {
                    gvfPath.move();
                }
                if (Math.abs(gvfPath.path.final_point.y-odometry.opt.get_y()) < 3) {
                    timer.reset();
                    state = State.pushSampleBack;
                }
                break;

            case pushSampleBack:
                wheelControl.drive_relative(-1, 0, 0, 1);
                if (odometry.opt.get_x() < 42) {
                    if (intakeState >= 2 && intakeState <= 3) {
                        gvfPath.setPath(pushPaths[intakeState-1], 0, false);
                        intakeState++;
                        state = State.goToPushSample;
                        timer.reset();
                    } else {
                        state = State.goToSpecimen;
                        timer.reset();
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

                if (timer.milliseconds() < 600) { //sensors.get_back_dist() >= intake_dist_thresh &&
                    wheelControl.drive(0.5,0,0,0,0.7);
                } else {
                    wheelControl.drive(0, 0, 0, 0, 0);
                    arm.closeClaw();

                    timer.reset();
                    target.y -= 1; // this has to be tuned better for more space on the right

                    state = State.pidToBar;
                    /*==================================*/
                }

                break;

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
        telemetry.addData("Fwd dist: ", sensors.get_front_dist());
        telemetry.addData("Heading: ", odometry.opt.get_heading());
        telemetry.addData("D value: ", path.get_d());
        telemetry.addData("Deposit State", intakeState);
        telemetry.addData("Motor position: ", lift.getPosition());
        telemetry.addData("Horizontal Motor position: ", horizontalSlides.horizontalSlidesMotor.getCurrentPosition());
        telemetry.addData("State: ", state);
        //telemetry.addData("Back Distance", sensors.get_back_dist());
        telemetry.addData("timer", timer.milliseconds());

        tp.put("X position: ", odometry.opt.get_x());
        tp.put("Y position: ", odometry.opt.get_y());
        tp.put("Fwd dist: ", sensors.get_front_dist());
        tp.put("Heading: ", odometry.opt.get_heading());
        tp.put("D value: ", path.get_d());
        tp.put("Deposit State", intakeState);
        tp.put("Motor position: ", lift.getPosition());
        tp.put("Horizontal Motor position: ", horizontalSlides.horizontalSlidesMotor.getCurrentPosition());
        tp.put("State: ", state);
        //tp.put("Back Distance", sensors.get_back_dist());
        tp.put("timer", timer.milliseconds());

        lift.update();
        telemetry.update();
        horizontalSlides.update();
        (FtcDashboard.getInstance()).sendTelemetryPacket(tp);
    }

    public void stop() {
        arm.openClaw();
    }
    // endregion
}