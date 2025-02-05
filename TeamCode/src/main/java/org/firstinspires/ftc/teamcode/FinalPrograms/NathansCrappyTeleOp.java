package org.firstinspires.ftc.teamcode.FinalPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Manipulator;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Alliance;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Sensors;
import org.opencv.core.Point;

import java.util.List;

@TeleOp
@Config
public class NathansCrappyTeleOp extends OpMode {
    double lastTime = getRuntime();

    ElapsedTime intakeTimer = new ElapsedTime();
    ElapsedTime speceminTimer;
    Odometry odometry;
    WheelControl drive;
    Alliance alliance = Alliance.red;

    Lift outtakeSlides;
    Arm arm;
    boolean clawOpen = true;
    Manipulator oldClaw;
    ElapsedTime outtakeTimer;
    OuttakeState outtakeState = OuttakeState.idle;
    SpeceminState specimenState = SpeceminState.idle;
    HangState hangState = HangState.hanging1;

    boolean newOuttakeState = true;
    boolean releaseSpec=false;
    boolean outtakeSpecimen=false;
    boolean releaseSample = false;
    boolean grabSample = false;

    boolean isAndrewMode = true;
    boolean pidSample = false;

    double drivePower = 1;

    public static double target_x = 50;
    public static double target_y = 90;
    public static double sample_x = 16;
    public static double sample_y = 127.5;
    public static double sample_angle = 135;

    double turnPower = 0.8;

    Intake intake;
    HorizontalSlides intakeSlides;

    Sensors sensors;
    Path path;

    List<LynxModule> allHubs;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    public static boolean retractIntakeOnSample;
    public static boolean disableSmart;

    @Override
    public void init() {
        Point[] follow_path = {
                new Point(11, 28),
                new Point(10, 65),
                new Point(37, 70),
                new Point(38, 50),
                new Point(41.6, 80.7)
        };

        odometry = new Odometry(hardwareMap, 0, 0, 0, "OTOS");
        drive = new WheelControl(hardwareMap, odometry);
        outtakeSlides = new Lift(hardwareMap, false);
        outtakeSlides.brakeSlides(true);
        arm = new Arm(hardwareMap);
        oldClaw = new Manipulator(hardwareMap);
        speceminTimer = new ElapsedTime();
        outtakeTimer = new ElapsedTime();
        path = new Path(follow_path, drive, odometry, telemetry, 0.1, 13, 180, 1);

        intake = new Intake(hardwareMap, new Sensors(hardwareMap,telemetry));
        intakeSlides = new HorizontalSlides(hardwareMap);
        gamepad2.setLedColor(1,1,0,Gamepad.LED_DURATION_CONTINUOUS);
        gamepad1.setLedColor(1,0,0,Gamepad.LED_DURATION_CONTINUOUS);
        sensors = new Sensors(hardwareMap, telemetry);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
    @Override
    public void start() {
        arm.toIdlePosition();
        arm.openClaw();
        oldClaw.openClaw();
        outtakeTimer.reset();
        intake.up();
    }
    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);

        // Updates
        odometry.opt.update();
        outtakeSlides.update();
        intakeSlides.update();
        intake.hasCorrectSample(true);

        if (isAndrewMode){
            if (gamepad1.left_bumper) {
                drivePower=0.3;
            }
            else if (previousGamepad1.left_bumper && !currentGamepad1.left_bumper){
                drivePower=1;
            }
        }
        else{
            if (gamepad1.left_bumper) {
                drivePower=0.3;
            }
            if (gamepad1.right_bumper) {
                drivePower=1;
            }
        }

        if (gamepad1.share && !previousGamepad1.share){
            isAndrewMode = !isAndrewMode;
        }
        if(gamepad1.options && !previousGamepad1.options && alliance == Alliance.blue) {
            gamepad1.setLedColor(1,0,0,Gamepad.LED_DURATION_CONTINUOUS);
            odometry.opt.setPos(odometry.opt.get_x(), odometry.opt.get_y(), 0);
            alliance = Alliance.red;
            intake.alliance = alliance;
        } else if (gamepad1.options && !previousGamepad1.options) {
            gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
            odometry.opt.setPos(odometry.opt.get_x(), odometry.opt.get_y(), 0);
            alliance = Alliance.blue;
            intake.alliance = alliance;
        }
        if(gamepad2.options) { //reset outtake sldies
            outtakeSlides.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            outtakeSlides.leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Drive
        drive.correction_drive(gamepad1.left_stick_y, 1.1*gamepad1.left_stick_x, -gamepad1.right_stick_x*Math.abs(gamepad1.right_stick_x), Math.toRadians(odometry.opt.get_heading()), drivePower);

        if (gamepad1.triangle && !previousGamepad1.triangle){
            odometry.opt.calibrate_imu();
            odometry.opt.calibrate_tracking();
            odometry.opt.setPos(7.875, 6.625, 0);
        }

        if (gamepad1.square && !previousGamepad1.square){
            if (specimenState == SpeceminState.idle){
                specimenState = SpeceminState.pid;
            }
            else{
                specimenState = SpeceminState.idle;
            }
        }

        if (gamepad1.circle && !previousGamepad1.circle){
            pidSample = !pidSample;
        }
        if (pidSample){
            path.follow_pid_to_point(new Point(sample_x, sample_y), sample_angle);
        }

        if (Math.abs(-gamepad2.left_stick_y) > 0.4) {
            outtakeSlides.trySetPower(-gamepad2.left_stick_y*1);
        } else if (outtakeSlides.leftSlide.getPower() != 0 && outtakeSlides.getState() == Lift.State.userControlled) {
            outtakeSlides.trySetPower(0);
        }

        switch (specimenState){
            case idle:
                speceminTimer.reset();
                break;

            case pid:
                intake.up();
                intake.stop();
                arm.closeClaw();

                outtakeSlides.toHighChamber();
                if (odometry.opt.get_y()<target_y-36) {
                    path.follow_pid_to_point(new Point(target_x-12, target_y), 0);
                } else {
                    path.follow_pid_to_point(new Point(target_x, target_y), 0);
                }

                if (speceminTimer.milliseconds() >= 100){
                    arm.outtakeSpecimen1();
                }
                if (sensors.get_front_dist() <= 2.5 && odometry.opt.get_heading() > -50 && odometry.opt.get_heading() < 50) {
                    specimenState = SpeceminState.deposit;
                }
                if (speceminTimer.milliseconds() > 5000) {
                    specimenState = SpeceminState.deposit;
                }

                break;

            case deposit:
                intake.stop();
                intake.down();

                arm.openClaw();
                arm.outtakeSpecimen1();
                drive.drive(0, -0.3, 0, 0, 1);

                if (speceminTimer.milliseconds() >= 125) {
                    outtakeSlides.setPosition(100);
                }

                if (speceminTimer.milliseconds() >= 400){
                    speceminTimer.reset();
                    specimenState = SpeceminState.goToSpecimen;
                }

                break;

            case goToSpecimen:
                intake.down();
                intake.stop();
                path.follow_pid_to_point(new Point(12.875, 28), 0);

                if (speceminTimer.milliseconds() > 500){
                    outtakeSlides.intakeSpecimen();
                    arm.intakeSpecimen();
                }

                if (path.at_point(new Point(13, 31), 3)) { //4
                    drive.drive(0, 0, 0, 0, 0);
                    speceminTimer.reset();
                    specimenState = SpeceminState.pickupSpecimen;
                }

                break;

            case pickupSpecimen:
                intake.down();
                intake.stop();
                arm.intakeSpecimen();

                if (sensors.get_back_dist() >= 2.5 || speceminTimer.milliseconds() < 1100) {
                    drive.drive(0.5, 0, 0, 0, 0.7);
                } else {
                    drive.drive(0, 0, 0, 0, 0);
                    arm.closeClaw();

                    speceminTimer.reset();
                    specimenState = SpeceminState.pid;
                }

                break;

        }

        switch (outtakeState) {
            case idle:
                if (newOuttakeState) {
                    outtakeSlides.setPosition(0);
                    arm.toIdlePosition();
                    arm.openClaw();
                    grabSample = false;
                }

                //manual horizontal extension
                if(Math.abs(-gamepad2.right_stick_y) > 0.1) {
                    intakeSlides.trySetPower(-gamepad2.right_stick_y);
                } else if (intakeSlides.horizontalSlidesMotor.getPower() != 0 && intakeSlides.getState() == HorizontalSlides.State.userControlled) { // in the dead zone and not running to target
                    intakeSlides.trySetPower(0);
                }

                if (currentGamepad2.left_bumper) {
                    intake.sweeperOut();
                } else {
                    intake.sweeperIn();
                }

                //manual intake
                if (currentGamepad2.right_trigger-currentGamepad2.left_trigger>0.7) {
                    if(disableSmart) {
                        intake.intake();
                        intake.closeDoor();
                        intake.down();
                    } else {
                        boolean hasCorrectObject = intake.smartIntake(true);
                        if(hasCorrectObject && retractIntakeOnSample) {
                            intakeSlides.setPosition(0);
                            intake.up();
                            gamepad1.rumble(500);
                            gamepad2.rumble(500);
                        } else {
                            intake.down();
                        }
                    }
                } else if (currentGamepad2.right_trigger-currentGamepad2.left_trigger<-0.7){
                    intake.reverseDown();
                    if (intakeTimer.milliseconds()>50) {
                        intake.reverse();
                    }
                } else {
                    intake.up();
                    intakeTimer.reset();
                    intake.setPower(0);
                    //intake.up();
                }

                newOuttakeState = false;

                if (currentGamepad2.cross) {
                    outtakeState = OuttakeState.intakeSpecimen;
                    outtakeTimer.reset();
                    newOuttakeState = true;
                } if (currentGamepad2.circle) {
                //arm.closeClaw();
                grabSample = true;
                outtakeState = OuttakeState.intakeSample;
                outtakeTimer.reset();
                newOuttakeState = true;
            }

                //if (grabSample && outtakeTimer.milliseconds()>150) {
                //     outtakeState = OuttakeState.outtakeSample;
                //     outtakeTimer.reset();
                //     newOuttakeState = true;
                // }

                break;

            case intakeSpecimen:
                if (newOuttakeState) {
                    outtakeSlides.intakeSpecimen();
                    arm.intakeSpecimen();
                    arm.openClaw();
                    intake.down();
                    outtakeSpecimen = false;
                }

                newOuttakeState = false;

                if (currentGamepad2.square && !previousGamepad2.square) {
                    arm.closeClaw();
                    outtakeTimer.reset();
                    outtakeSpecimen=true;
                }
                if (outtakeSpecimen && outtakeTimer.milliseconds()>=100) {
                    outtakeState = OuttakeState.outtakeSpecimen1;
                    outtakeTimer.reset();
                    newOuttakeState = true;
                }

                break;

            case intakeSample:
                if (newOuttakeState) {
                    outtakeSlides.intakeSample();
                    arm.openClaw();
                    arm.intakeSample();
                }

                newOuttakeState = false;

                if (currentGamepad2.circle && outtakeTimer.milliseconds()>=150) {
                    arm.closeClaw();
                    if (outtakeTimer.milliseconds()>=150+300) {
                        outtakeState = OuttakeState.outtakeSample;
                        outtakeTimer.reset();
                        newOuttakeState = true;
                    }
                }

                break;

            case outtakeSpecimen1:


                if (newOuttakeState) {
                    outtakeSlides.toHighChamber();
                    arm.closeClaw();
                    arm.outtakeSpecimen1();
                    releaseSpec = false;
                }

                newOuttakeState = false;

                if (previousGamepad2.square && !currentGamepad2.square) {
                    arm.openClaw();
                    outtakeTimer.reset();
                    releaseSpec = true;
                }
                if (releaseSpec && outtakeTimer.milliseconds()>=125) {
                    outtakeState = OuttakeState.outakeSpecimen2;
                    outtakeTimer.reset();
                    newOuttakeState = true;
                }

                break;

            case outakeSpecimen2:
                if (newOuttakeState) {
                    outtakeSlides.setPosition(100);
                    arm.openClaw();
                    arm.outtakeSpecimen1();
                }

                newOuttakeState = false;

                if (currentGamepad2.cross) {
                    outtakeState = OuttakeState.intakeSpecimen;
                    outtakeTimer.reset();
                    newOuttakeState = true;
                } else if (currentGamepad2.triangle) {
                    outtakeState = OuttakeState.idle;
                    outtakeTimer.reset();
                    newOuttakeState = true;
                }

                break;

            case outtakeSample:

                if (newOuttakeState) {
                    drivePower=0.5;
                    outtakeSlides.toHighBasket();
                    arm.closeClaw();
                    arm.outtakeSample();
                    releaseSample = false;
                }

                newOuttakeState = false;

                if (previousGamepad2.circle && !currentGamepad2.circle) {
                    arm.openClaw();
                    releaseSample = true;
                    outtakeTimer.reset();
                } if (currentGamepad2.circle) {
                    outtakeTimer.reset();
                }

                if (releaseSample && outtakeTimer.milliseconds()>=175) {
                    arm.toIdlePosition();
                    if (outtakeTimer.milliseconds()>=250) {
                        outtakeState = OuttakeState.idle;
                        outtakeTimer.reset();
                        newOuttakeState = true;
                    }
                }

                break;
        }


        if(!previousGamepad2.right_bumper && currentGamepad2.right_bumper) { // TODO bucket logic
            if (clawOpen) {
                arm.closeClaw();
                oldClaw.closeClaw();
            } else {
                arm.openClaw();
                oldClaw.openClaw();
            }
            clawOpen = !clawOpen;
        }


        if(!previousGamepad2.share && currentGamepad2.share) {
            if(hangState == HangState.hanging1) {
                outtakeSlides.setPosition(2150);
                hangState = HangState.hanging2;
            } else {
                outtakeSlides.setPosition(1500);
                hangState = HangState.hanging1;
            }
        }
        lastTime = getRuntime();
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Is Andrew Mode?", isAndrewMode);
        telemetry.addData("outtake state", outtakeState);
        telemetry.addData("specemen state", specimenState);
        telemetry.addData("X pos", odometry.opt.get_x());
        telemetry.addData("Y pos", odometry.opt.get_y());
        telemetry.addData("Heading", odometry.opt.get_heading());
    }



    enum OuttakeState {
        idle,
        intakeSpecimen,
        intakeSample,
        outtakeSpecimen1,
        outakeSpecimen2,
        outtakeSample,
    }

    enum SpeceminState{
        idle,
        pid,
        deposit,
        goToSpecimen,
        pickupSpecimen
    }

    enum HangState {
        hanging1,
        hanging2
    }
}