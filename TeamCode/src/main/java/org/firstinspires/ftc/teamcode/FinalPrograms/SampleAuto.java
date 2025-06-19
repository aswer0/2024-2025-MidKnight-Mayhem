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
public class SampleAuto extends OpMode {
    public static double samplePointX=18;
    public static double samplePointY=125;

    Point deposit_point;
    Point intake_point1;
    Point intake_point2;
    Point intake_point3;
    double pos = -500;
    double pid_max_power = 1;

    ElapsedTime timer;
    Sensors sensors;

    Odometry odometry;
    WheelControl wheelControl;
    VectorField vf;
    Vision vision;
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
        subIntake,
        visionIntake,
        park,
        test
    }

    @Override
    public void init() {
        deposit_point = new Point(samplePointX, samplePointY); //125.8

        Point[][] follow_path = {{
                new Point(11, 128.7),
                new Point(38, 121.8),
                new Point(65, 125),
                new Point(60, 98.3),
        }};

        BCPath path = new BCPath(follow_path);

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

                    if (timer.milliseconds() >= 2000) {
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
                        if (timer.milliseconds()>150) {
                            arm.closeClaw();
                            if (timer.milliseconds()>450) {
                                lift.toHighBasket();
                                if (lift.getCurrentPos() > (highBasketPos-700)) {
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

                if (timer.milliseconds() >= 1000){
                    arm.openClaw();
                }
                if (timer.milliseconds() >= 1200){
                    arm.toIdlePosition();
                    timer.reset();
                    state = State.subIntake;
                }

                break;

            case subIntake:
                arm.toIdlePosition();
                lift.intakeSample();
                vf.move();

                if (vf.at_end(0.75)){
                    state = State.visionIntake;
                }

            case visionIntake:
                if (vision.get_sample(0.7, 90, 0.75, 'x')){
                    vision.reset();
                    state = State.pid;
                }
//
//            case park:
//                if (timer.milliseconds()<5000) vf.move();
//                lift.toHighChamber();
//                if (timer.milliseconds()>5000) {
//                    arm.toTeleStartPosition();
//                    wheelControl.drive_relative(0.2,0,0,1);
//                }
//                break;
        }
        telemetry.addData("state", state);
        telemetry.addData("x", odometry.opt.get_x());
        telemetry.addData("y", odometry.opt.get_y());
        telemetry.addData("has correct sample", intake.hasCorrectSample(true));
    }
}
