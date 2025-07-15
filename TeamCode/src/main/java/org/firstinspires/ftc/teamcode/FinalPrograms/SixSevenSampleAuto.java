package org.firstinspires.ftc.teamcode.FinalPrograms;

import static org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Outtake.Lift.highBasketPos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.BCPath;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Cameras.SampleFinder;
import org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Cameras.Vision;
import org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Intake.HorizontalSlides;
import org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Outtake.Arm;
import org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Alliance;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Sensors;
import org.opencv.core.Point;

@Config
@Autonomous
public class SixSevenSampleAuto extends OpMode {
    public static double samplePointX=17;
    public static double samplePointY=119.25;

    public static double intakeX1=24;
    public static double intakeY1=124;
    public static double intake_angle1 = 155;

    public static double intakeX2=24;
    public static double intakeY2=125;
    public static double intake_angle2 = 180;

    public static double intakeX3=42;
    public static double intakeY3=120.25;
    public static double intake_angle3 = 260;


    Point deposit_point;
    Point intake_point1;
    Point intake_point2;
    Point intake_point3;
    double pos = -500;
    public static double pid_max_power = 0.825;

    ElapsedTime timer;
    Sensors sensors;

    Odometry odometry;
    WheelControl wheelControl;
    VectorField vf;
    Vision vision;
    SampleFinder processor;
    BCPath path;
    //Path path;

    State state = State.pid;
    Lift lift;
    Arm arm;
    HorizontalSlides intakeSlides;
    Intake intake;

    Alliance alliance = Alliance.red;
    int intakeState=0;

    enum State {
        pid,
        deposit_sample,
        setup_intake_sample,
        intake_sample,
        subIntake,
        visionIntake,
        park
    }

    @Override
    public void init() {
        deposit_point = new Point(samplePointX, samplePointY); //125.8
        intake_point1 = new Point(intakeX1, intakeY1);
        intake_point2 = new Point(intakeX2, intakeY2);
        intake_point3 = new Point(intakeX3, intakeY3);

        Point[][] follow_path = {{
                new Point(17, 118),
                new Point(37, 109),
                new Point(52, 110),
                new Point(54, 88),
        }};

        path = new BCPath(follow_path);

        timer = new ElapsedTime();
        sensors = new Sensors(hardwareMap, telemetry);

        odometry = new Odometry(hardwareMap, 90, 7.25, 96+7.7, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);

        vf = new VectorField(wheelControl, odometry);
        vf.setPath(path, 90, false);

        wheelControl.change_mode(DcMotor.ZeroPowerBehavior.BRAKE);

        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap, sensors);
        intakeSlides = new HorizontalSlides(hardwareMap);

        vision = new Vision(odometry, telemetry, vf, hardwareMap, intakeSlides, intake, alliance);
        vision.reset();
        processor = vision.get_processor();

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
    }

    @Override
    public void loop() {
        intake.closeDoor();
        odometry.opt.update();
        lift.update();
        telemetry.update();
        intakeSlides.update();

        switch(state) {
            case pid:
                intake.up();
                intake.stop();
                intakeSlides.setPosition(0);
                vf.pid_to_point(deposit_point, 135, pid_max_power);

                if (intakeState==0) {
                    arm.closeClaw();
                    if (timer.milliseconds() >= 300) {
                        lift.toHighBasket();
                    }

                    if (timer.milliseconds() >= 850) {
                        timer.reset();
                        intakeState++;
                        state = State.deposit_sample;
                    }
                } else {
                    boolean transfer;
                    if (intakeSlides.getPosition()>-20 || timer.milliseconds()>3000) {
                        lift.intakeSample();
                        arm.intakeSample();
                        transfer=true;
                    } else {
                        timer.reset();
                        transfer = false;
                    }
                    if (transfer) {
                        if (timer.milliseconds()>250) {
                            arm.closeClaw();
                            if (timer.milliseconds()>550) {
                                lift.toHighBasket();
                                if (lift.getCurrentPos() > (highBasketPos-700) && vf.at_point(deposit_point, 1)) {
                                    timer.reset();
                                    intakeState++;
                                    state = State.deposit_sample;
                                }
                            }
                        }
                    }
                }

                break;

            case deposit_sample: //and move to intake
                wheelControl.drive(0,0,0,0,0);
                arm.outtakeSample();

                if (timer.milliseconds() >= 600){
                    arm.openClaw();
                }
                if (timer.milliseconds() >= 875){
                    arm.toIdlePosition();

                    /* make this 5 for 5 sample and park */
                    if (intakeState == 6){
                        vf.setPath(path, -90, false);
                        timer.reset();
                        state = State.park;
                    }
                    else if (intakeState>=4) {
                        vf.setPath(path, 90, false);
                        timer.reset();
                        state = State.subIntake;
                    }
                    else{
                        if (intakeState == 2){
                            pos=-450;
                        }
                        if (intakeState == 3){
                            pos=-180;
                        }

                        state = State.intake_sample;
                        timer.reset();
                    }
                }

                break;

            case intake_sample:
                if (odometry.opt.get_x()>(samplePointX+4) || odometry.opt.get_y()<(samplePointY-4)) {
                    lift.intakeSample();
                }
                intakeSlides.setPosition(pos);
                intake.intake();

                switch (intakeState) {
                    case 1:
                        vf.pid_to_point(intake_point1, intake_angle1, pid_max_power);
                        if (vf.at_point(intake_point1,3)) intake.down();
                        break;
                    case 2:
                        vf.pid_to_point(intake_point2, intake_angle2, pid_max_power);
                        if (vf.at_point(intake_point2,3)) intake.down();

                        break;
                    case 3:
                        vf.pid_to_point(intake_point3, intake_angle3, pid_max_power);
                        if (vf.at_point(intake_point3,3)) intake.down();

                        break;
                }

                if (timer.milliseconds() >= 1500){
                    intake.down();
                }

                if (intake.hasCorrectSample(true)) {
                    timer.reset();
                    state = State.pid;
                }

                if (timer.milliseconds()>5000) {
                    timer.reset();
                    state = State.pid;
                }

                break;

            case subIntake:
                arm.toIdlePosition();
                lift.intakeSample();
                vf.move();
                pid_max_power = 0.1;

                if (sensors.isTouchBack()){
                    pid_max_power = 0.9;
                    wheelControl.stop();
                    vision.reset();
                    state = State.visionIntake;
                }
                break;

            case visionIntake:
                if (vision.get_sample(pid_max_power, 90, 0.85, 'x')){
                    vision.reset();
                    timer.reset();
                    state = State.pid;
                }
                break;

            case park:
                arm.toTeleStartPosition();
                if (timer.milliseconds()<5000) vf.move();
                lift.toHighChamber();
                if (timer.milliseconds()>5000) {
                    wheelControl.drive_relative(0.2,0,0,1);
                }
                break;
        }

        telemetry.addData("state", state);
        telemetry.addData("x", odometry.opt.get_x());
        telemetry.addData("y", odometry.opt.get_y());
        telemetry.addData("target position", vision.get_target_position());
        telemetry.addData("touch button", sensors.isTouchBack());
        telemetry.addData("has correct sample", intake.hasCorrectSample(true));
    }
}