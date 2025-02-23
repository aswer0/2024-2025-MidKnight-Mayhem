package org.firstinspires.ftc.teamcode.FinalPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
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

@Config
@TeleOp
public class DanielsCrappyTeleOp extends OpMode {
    public static boolean disableSmart = false;
    public static boolean retractIntakeOnSample = true;
    ElapsedTime intakeTimer = new ElapsedTime();
    Odometry odometry;
    WheelControl drive;
    Alliance alliance = Alliance.red;

    Lift outtakeSlides;
    Arm arm;
    boolean clawOpen = true;
    Manipulator oldClaw;
    ElapsedTime outtakeTimer;
    ElapsedTime rejectTimer;
    OuttakeState outtakeState = OuttakeState.start;
    HangState hangState = HangState.hanging1;
    boolean newOuttakeState = true;
    boolean sampleMode = true;

    boolean releaseSpec=false;
    boolean outtakeSpecimen=false;
    boolean releaseSample = false;
    boolean grabSample = false;

    boolean isAndrewMode = true;
    boolean autoSpecDrive = false;
    AutoSpecDriveState autoSpecDriveState = AutoSpecDriveState.start;

    //auto values
    public static double power = 1;
    public static double target_x = 50; //36.2
    public static double target_y = 92;
    public static double dist_thresh = 2.5;
    public static double intake_dist_thresh = 2.5;
    Point target;
    ElapsedTime autoTimer;
    ElapsedTime intakeTime;

    public static double sample_x = 13;
    public static double sample_y = 123.5;
    public static double sample_angle = 135;

    Intake intake;
    HorizontalSlides intakeSlides;

    Sensors sensors;

    List<LynxModule> allHubs;
    ElapsedTime loopTimer;

    double drivePower=1;
    double turnPower=1;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    Path path;

    @Override
    public void init() {
        Point[] follow_path = {
                new Point(11,28),
                new Point(10.3, 65.5),
                new Point(19.2, 68.2),
                new Point(36.5, 75)
        };

        odometry = new Odometry(hardwareMap, 90, 0, 0, "OTOS");
        drive = new WheelControl(hardwareMap, odometry);
        outtakeSlides = new Lift(hardwareMap, false);
        outtakeSlides.brakeSlides(true);
        arm = new Arm(hardwareMap);
        oldClaw = new Manipulator(hardwareMap);
        outtakeTimer = new ElapsedTime();

        intake = new Intake(hardwareMap, new Sensors(hardwareMap,telemetry));
        intakeSlides = new HorizontalSlides(hardwareMap, false);
        gamepad2.setLedColor(1,1,0,Gamepad.LED_DURATION_CONTINUOUS);
        gamepad1.setLedColor(1,0,0,Gamepad.LED_DURATION_CONTINUOUS);
        sensors = new Sensors(hardwareMap, telemetry);

        //atuo stuff
        path = new Path(follow_path,drive, odometry,telemetry,0.1, 13,180,power);
        target = new Point(target_x, target_y);
        autoTimer = new ElapsedTime();
        intakeTime = new ElapsedTime();

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        loopTimer = new ElapsedTime();
    }
    @Override
    public void init_loop() {
        if(gamepad1.options && !previousGamepad1.options && (alliance == Alliance.blue)) {
            gamepad1.setLedColor(1,0,0,Gamepad.LED_DURATION_CONTINUOUS);
            odometry.opt.setPos(0,0,0);
            alliance = Alliance.red;
            intake.alliance = alliance;
        } else if (gamepad1.options && !previousGamepad1.options) {
            gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
            odometry.opt.setPos(0,0,0);
            alliance = Alliance.blue;
            intake.alliance = alliance;
        }
        if(gamepad1.share && !previousGamepad1.share) {
            sampleMode = !sampleMode;
        }

        previousGamepad1.copy(gamepad1);
        telemetry.addData("Alliance", alliance);
        telemetry.addLine();
        telemetry.addData("sample mode?",sampleMode);
    }
    @Override
    public void start() {
        arm.toIdlePosition();
        arm.openClaw();
        oldClaw.openClaw();
        outtakeTimer.reset();
        intake.up();
        loopTimer.reset();
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
            if (currentGamepad1.left_bumper) {
                drivePower=0.3;
            }else if (previousGamepad1.left_bumper && !currentGamepad1.left_bumper){
                drivePower=1;
            }

            if (intakeSlides.getPosition()<-200) {
                turnPower=0.6;
            } else {
                turnPower=1;
            }
        } else{
            if (gamepad1.left_bumper) {
                drivePower=0.3;
            }if (gamepad1.right_bumper) {
                drivePower=1;
            }
        }

        if (gamepad1.share && !previousGamepad1.share){
            isAndrewMode = !isAndrewMode;
        }
        if(gamepad1.options && !previousGamepad1.options && (alliance == Alliance.blue)) {
            gamepad1.setLedColor(1,0,0,Gamepad.LED_DURATION_CONTINUOUS);
            odometry.opt.setPos(0,0,0);
            alliance = Alliance.red;
            intake.alliance = alliance;
        } else if (gamepad1.options && !previousGamepad1.options) {
            gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
            odometry.opt.setPos(0,0,0);
            alliance = Alliance.blue;
            intake.alliance = alliance;
        }
        if(currentGamepad1.share && !previousGamepad1.share) {
            sampleMode = !sampleMode;
        }
        if(gamepad2.options) { //reset outtake sldies
            outtakeSlides.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            outtakeSlides.leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (!previousGamepad1.triangle && currentGamepad1.triangle) {
            odometry.opt.setPos(sample_x, sample_y, sample_angle);
        }

        if (!previousGamepad1.cross && currentGamepad1.cross && outtakeState == OuttakeState.intakeSpecimen) {
            //odometry.opt.setPos(9,32,0);
            autoSpecDrive=true;
            autoSpecDriveState = AutoSpecDriveState.start;
            outtakeState = OuttakeState.auto;
            gamepad1.rumble(500);
            gamepad2.rumble(500);
            autoTimer.reset();
        }

        //Drive
        if (autoSpecDrive) {
            //failsafe
            if (Math.abs(currentGamepad1.left_stick_x)>0.1 || Math.abs(currentGamepad1.left_stick_y)>0.1) {
                outtakeState= OuttakeState.idle;
                newOuttakeState = true;
                autoSpecDrive=false;
            }
            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
                target.y += 1.5; // this has to be tuned better for more space on the right
            } else if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
                target.y -= 1.5; // this has to be tuned better for more space on the right
            }

            switch (autoSpecDriveState) {
                case start:
                    arm.closeClaw();
                    if (autoTimer.milliseconds()>100) {
                        odometry.opt.setPos(8,34,0);
                        autoTimer.reset();
                        autoSpecDriveState = AutoSpecDriveState.pid;
                    }
                    break;
                case pid:
                    //outtakeState = OuttakeState.outtakeSpecimen1;
                    arm.closeClaw();
                    arm.outtakeSpecimen1();
                    outtakeSlides.toHighChamber();

                    if (odometry.opt.get_y()<target_y-36) {
                        path.follow_pid_to_point(new Point(target_x - 12, target_y), 0);
                    } else {
                        path.follow_pid_to_point(new Point(target_x, target_y-30), 0);
                    }

                    if (sensors.get_front_dist() <= dist_thresh || autoTimer.milliseconds() > 5000) {
                        autoTimer.reset();
                        autoSpecDriveState = AutoSpecDriveState.deposit;
                    }
                    break;

                case deposit:
                    drive.drive_relative(0.1, -0.9, 0, 1);
                    if (autoTimer.milliseconds() >= 150) {
                        outtakeSlides.setPosition(0);
                        arm.outtakeSpecimen2();
                        arm.openClaw();
                    }
                    if (autoTimer.milliseconds()>250) {
                        autoTimer.reset();
                        autoSpecDriveState = AutoSpecDriveState.goToSpecimen;
                    }
                    break;
                case goToSpecimen:
                    intake.down();
                    intakeSlides.setPosition(0);
                    path.follow_pid_to_point(new Point(15.875, 32), 0);

                    if (autoTimer.milliseconds() > 500){
                        outtakeSlides.intakeSpecimen();
                        arm.intakeSpecimen();
                    }
                    if (path.at_point(new Point(15.875, 32), 6) || autoTimer.milliseconds()>4000 || currentGamepad1.square) { //4
                        drive.stop();
                        autoTimer.reset();
                        autoSpecDriveState = AutoSpecDriveState.pickupSpecimen;
                    }
                    break;
                case pickupSpecimen:
                    if (!currentGamepad1.square) {
                        drive.drive(0.5, 0, 0, 0, 0.7);
                    } else {
                        drive.stop();
                        arm.closeClaw();
                        autoTimer.reset();
                        autoSpecDriveState = AutoSpecDriveState.pid;
                    }
                    break;
            }
        } else if (currentGamepad1.circle){ //auto pid sample
            path.follow_pid_to_point(new Point(sample_x, sample_y), sample_angle);
        }else {
            drive.correction_drive(gamepad1.left_stick_y, 1.1 * gamepad1.left_stick_x, -gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x) * turnPower, Math.toRadians(odometry.opt.get_heading()), drivePower);
        }

        //manual outtake slides
        if(Math.abs(-gamepad2.left_stick_y) > 0.4) {
            outtakeSlides.trySetPower(-gamepad2.left_stick_y*1);
        } else if (outtakeSlides.leftSlide.getPower() != 0 && outtakeSlides.getState() == Lift.State.userControlled) {
            outtakeSlides.trySetPower(0);
        }

        switch (outtakeState) {
            case start:
                if (currentGamepad2.triangle) {
                    intakeSlides.setPosition(0);
                    outtakeState = OuttakeState.idle;
                    outtakeTimer.reset();
                    newOuttakeState = true;
                }
                break;
            case idle:
                if (newOuttakeState) {
                    drivePower=1;
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

//                if (currentGamepad2.left_bumper) {
//                    intake.sweeperOut();
//                } else {
//                    intake.sweeperIn();
//                }

                if(!previousGamepad2.right_stick_button && currentGamepad2.right_stick_button) {
                    if(rejectTimer == null) rejectTimer = new ElapsedTime();
                    rejectTimer.reset();
                }
                if(!previousGamepad2.left_bumper && currentGamepad2.left_bumper) {
                    if(rejectTimer == null) rejectTimer = new ElapsedTime();
                    rejectTimer.reset();
                }
                //manual intake
                if(rejectTimer != null && rejectTimer.milliseconds() < 70) {
                    intake.reverseDown();
                    intake.reverse();
                } else if (rejectTimer != null && rejectTimer.milliseconds() < 200) {
                    intake.reverseDown();
                    intake.intake();
                } else if (currentGamepad2.right_trigger-currentGamepad2.left_trigger>0.7) {
                    if(disableSmart) {
                        intake.intake();
                        intake.closeDoor();
                        intake.down();
                    }else {
                        if (!intake.hasCorrectSample(sampleMode)) {
                            intakeTime.reset();
                            boolean hasCorrectObject = intake.smartIntake(sampleMode);
                            if (hasCorrectObject && retractIntakeOnSample) {
                                intakeSlides.setPosition(0);
                            }
                        } else if (intakeTime.milliseconds()<600){
                            intake.smartIntake(sampleMode);
                        }else {
                            gamepad1.rumble(500);
                            gamepad2.rumble(500);
                            intakeSlides.setPosition(0);
                        }
                    }
                } else if (currentGamepad2.right_trigger-currentGamepad2.left_trigger<-0.7){
                    intake.openDoor();
                    intake.reverseDown();
                    if (intakeTimer.milliseconds()>50) {
                        intake.setPower(1);
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
                } if (currentGamepad2.circle) { // || currentGamepad2.left_bumper
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
                    intakeSlides.setPosition(0);
                    arm.intakeSpecimen();
                    arm.openClaw();
                    intake.down();
                    outtakeSpecimen = false;
                }

                newOuttakeState = false;

                if (currentGamepad2.triangle) {
                    outtakeState = OuttakeState.idle;
                    outtakeTimer.reset();
                    newOuttakeState = true;
                }

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
                    intake.stop();
                    arm.closeClaw();
                    intake.openDoor();
                    if (outtakeTimer.milliseconds()>=150+300) {
                        outtakeState = OuttakeState.outtakeSample;
                        outtakeTimer.reset();
                        newOuttakeState = true;
                    }
                } else {
                    intake.closeDoor();
                    intake.intake();
                }

//                if (currentGamepad2.left_bumper && outtakeTimer.milliseconds()>=150) {
//                    arm.closeClaw();
//                    if (outtakeTimer.milliseconds()>=150+300) {
//                        outtakeState = OuttakeState.specOut;
//                        outtakeTimer.reset();
//                        newOuttakeState = true;
//                    }
//                }

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

                if (releaseSample && outtakeTimer.milliseconds()>=250) {
                    arm.toIdlePosition();
                    if (outtakeTimer.milliseconds()>=250+100) {
                        arm.toIdlePosition();


                        if (currentGamepad2.triangle) {
                            intake.closeDoor();
                            outtakeState = OuttakeState.idle;
                            outtakeTimer.reset();
                            newOuttakeState = true;
                        }
                    }
                }

                break;

            case auto:
                break;

            case specOut:
                if (newOuttakeState) {
                    arm.closeClaw();
                    outtakeSlides.setPosition(800);
                    arm.outtakeSample();
                    releaseSample = false;
                }

                if (outtakeSlides.getPosition()>700) outtakeSlides.setPosition(0);

                newOuttakeState = false;

                if (previousGamepad2.left_bumper && !currentGamepad2.left_bumper) {
                    arm.openClaw();
                    releaseSample = true;
                    outtakeTimer.reset();
                } if (currentGamepad2.left_bumper) {
                outtakeTimer.reset();
            }
                if (releaseSample && outtakeTimer.milliseconds()>=250) {
                    arm.toIdlePosition();
                    if (outtakeTimer.milliseconds() >= 250 + 100) {
                        outtakeState = OuttakeState.idle;
                        outtakeTimer.reset();
                        newOuttakeState = true;
                    }
                }

                break;
        }


        if(!previousGamepad2.right_bumper && currentGamepad2.right_bumper) {
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
                outtakeSlides.setPosition(2450);
                hangState = HangState.hanging2;
            } else {
                outtakeSlides.setPosition(1500);
                hangState = HangState.hanging1;
            }
        }
        telemetry.addData("Alliance", alliance);
        telemetry.addLine();
        telemetry.addData("sample mode?",sampleMode);
        telemetry.addLine();
        telemetry.addData("Is Andrew Mode?", isAndrewMode);
        telemetry.addData("state", outtakeState);
        telemetry.addLine();
        telemetry.addData("auto spec drive?", autoSpecDrive);
        telemetry.addData("spec drive state", autoSpecDriveState);
        telemetry.addLine();
        telemetry.addData("x", odometry.opt.get_x());
        telemetry.addData("y", odometry.opt.get_y());
        telemetry.addData("heading", odometry.opt.get_heading());
        telemetry.addLine();
        telemetry.addData("sample mode?",sampleMode);
        telemetry.addLine();
        telemetry.addData("Loop time (ms)", loopTimer.milliseconds());
        loopTimer.reset();
    }



    enum OuttakeState {
        start,
        idle,
        intakeSpecimen,
        intakeSample,
        outtakeSpecimen1,
        outakeSpecimen2,
        outtakeSample,
        auto,
        specOut
    }

    enum HangState {
        hanging1,
        hanging2
    }

    enum AutoSpecDriveState {
        start,
        pid,
        goToSpecimen,
        pickupSpecimen,
        deposit
    }
}
