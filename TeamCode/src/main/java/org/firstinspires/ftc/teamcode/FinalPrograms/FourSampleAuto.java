package org.firstinspires.ftc.teamcode.FinalPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.BCPath;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.VectorField;
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
public class FourSampleAuto extends OpMode {
    public static double samplePointX=16;
    public static double samplePointY=144-16;

    public static double intakeX1=16;
    public static double intakeY1=117;
    public static double intake_angle1 = 170;

    public static double intakeX2=16;
    public static double intakeY2=117;
    public static double intake_angle2 = 180;

    public static double intakeX3=24;
    public static double intakeY3=119;
    public static double intake_angle3 = 225;


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
        park
    }

    @Override
    public void init() {
        deposit_point = new Point(samplePointX, samplePointY); //125.8
        intake_point1 = new Point(intakeX1, intakeY1);
        intake_point2 = new Point(intakeX2, intakeY2);
        intake_point3 = new Point(intakeX3, intakeY3);

        Point[][] follow_path = {{
                new Point(11, 128.7),
                new Point(38, 121.8),
                new Point(65, 125),
                new Point(76.8, 98.3),
        }};

        BCPath path = new BCPath(follow_path);

        timer = new ElapsedTime();
        sensors = new Sensors(hardwareMap, telemetry);

        odometry = new Odometry(hardwareMap, 90, 7.25, 96+7.7, "OTOS");
        wheelControl = new WheelControl(hardwareMap, odometry);

        vf = new VectorField(wheelControl, odometry);
        vf.setPath(path, -90, false);

        wheelControl.change_mode(DcMotor.ZeroPowerBehavior.BRAKE);

        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap, sensors);
        intakeSlides = new HorizontalSlides(hardwareMap);

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

                    if (timer.milliseconds() >= 2500) {
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
                        timer.reset();
                    } else {
                        transfer = false;
                    }
                    if (transfer) {
                        if (timer.milliseconds()>150) {
                            arm.closeClaw();
                            if (timer.milliseconds()>150+300) {
                                lift.toHighBasket();
                                if (timer.milliseconds()>150+2500) {
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
                arm.outtakeSample();

                if (timer.milliseconds() >= 250){
                    arm.openClaw();
                }
                if (timer.milliseconds() >= 500){
                    arm.toIdlePosition();
                    lift.intakeSample();

                    if (intakeState==4) {
                        state = State.park;
                        timer.reset();
                    }

                    switch (intakeState) {
                        case 1:
                            vf.pid_to_point(intake_point1, intake_angle1, pid_max_power);
                            break;
                        case 2:
                            vf.pid_to_point(intake_point2, intake_angle2, pid_max_power);
                            break;
                        case 3:
                            vf.pid_to_point(intake_point3, intake_angle3, pid_max_power);
                            break;
                    }
                }
                if (timer.milliseconds() >= 2000){
                    state = State.intake_sample;
                    timer.reset();
                }

                break;

           case intake_sample:
                intakeSlides.setPosition(pos);
                intake.intake();

                if (timer.milliseconds() >= 500){
                    intake.down();
                    wheelControl.drive(0,-0.2,0,Math.toRadians(odometry.opt.get_heading()),1);
                }

                if (intake.hasCorrectSample(true)) {
                    timer.reset();
                    state = State.pid;
                }

                break;

           case park:
                vf.move();

                lift.toHighChamber();
                break;
        }
    }
}
