package org.firstinspires.ftc.teamcode.FinalPrograms;

import static org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Outtake.Lift.hangHigh;
import static org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Outtake.Lift.hangLow;
import static org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Outtake.Lift.highBasketPos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
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

import java.util.List;

@TeleOp
@Config
/*
CONTROLS:
left joystick x                 strafe
left joystick y                 forward/backward
right joystick x                turning
share button                    toggle sample/specimen mode
options button                  reset field-orientation/toggle alliance

dpad up                         hang
right trigger                   extend intake slides
left trigger                    retract intake slides
right bumper                    intake
cross button                    reverse intake

Sample Mode:
left bumper                     transfer failsafe/deposit/retract failsafe
TODO: circle button             auto drive to basket (WIP) - need to tune

Specimen Mode:ƒ
TODO: left bumper               grab spec/deposit spec/retract arm (WIP)
TODO: circle button             auto specimen drive sequence (WIP)

 */
public class A_COMPETITION_TELEOP extends OpMode {
    Odometry odometry;
    WheelControl drive;
    Path path;
    Alliance alliance = Alliance.red;

    State state = State.start;
    ElapsedTime stateTimer;
    boolean newState = true;

    IntakeState intakeState = IntakeState.ready;
    ElapsedTime intakeTimer;

    Lift outtakeSlides;
    Arm arm;

    Intake intake;
    HorizontalSlides intakeSlides;

    Sensors sensors;
    List<LynxModule> allHubs;
    HangState hangState = HangState.hanging1;

    boolean autoSampleDrive = false;
    boolean sampleMode = true;
    boolean outtakeSpec=false;
    boolean releaseSpec=false;

    public static int inTime=60;
    public static int spitTime=60;
    public static int transferTime1=175;
    public static int transferTime2=500;

    public static double MAX_DRIVE_POWER=1;
    double drivePower=MAX_DRIVE_POWER;
    double turnPower=1;

    public static double sampleX=18;
    public static double sampleY=124;
    public static double sampleAngle=135;
    Point sampleTarget = new Point(sampleX, sampleY);

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    @Override
    public void init() {
        Point[] follow_path = {
                new Point(11,28),
                new Point(10.3, 65.5),
                new Point(19.2, 68.2),
                new Point(36.5, 75)
        };

        odometry = new Odometry(hardwareMap, 0, 100, 100, "OTOS");
        drive = new WheelControl(hardwareMap, odometry);
        path = new Path(follow_path, drive, odometry, telemetry,0.1,13,180,1);
        outtakeSlides = new Lift(hardwareMap, false);
        outtakeSlides.brakeSlides(true);
        arm = new Arm(hardwareMap);

        intake = new Intake(hardwareMap, new Sensors(hardwareMap,telemetry));
        intakeSlides = new HorizontalSlides(hardwareMap, false);
        gamepad1.setLedColor(1,0,0,Gamepad.LED_DURATION_CONTINUOUS);
        sensors = new Sensors(hardwareMap, telemetry);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        stateTimer = new ElapsedTime();
        intakeTimer = new ElapsedTime();
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

        previousGamepad1.copy(gamepad1);
        telemetry.addData("",alliance);
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        if(currentGamepad1.options && !previousGamepad1.options && (alliance == Alliance.blue)) {
            gamepad1.setLedColor(1,0,0,Gamepad.LED_DURATION_CONTINUOUS);
            odometry.opt.setPos(0,0,0);
            alliance = Alliance.red;
            intake.alliance = alliance;
        } else if (currentGamepad1.options && !previousGamepad1.options) {
            gamepad1.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
            odometry.opt.setPos(0,0,0);
            alliance = Alliance.blue;
            intake.alliance = alliance;
        }
        if (currentGamepad1.share && !previousGamepad1.share) {
            sampleMode = !sampleMode;
        }

        // Updates
        odometry.opt.update();
        outtakeSlides.update();
        intakeSlides.update();
        intake.hasCorrectSample(true);

        if (autoSampleDrive) {
            path.follow_pid_to_point(sampleTarget,sampleAngle);
            if (Math.abs(currentGamepad1.left_stick_x)>0.1 || Math.abs(currentGamepad1.left_stick_y)>0.1 || Math.abs(currentGamepad1.right_stick_x)>0.1) {
                autoSampleDrive=false;
            }
        } else {
            //drive.drive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x*turnPower,0, drivePower);
            drive.drive(gamepad1.left_stick_y, 1.2 * gamepad1.left_stick_x, -gamepad1.right_stick_x * turnPower, Math.toRadians(odometry.opt.get_heading()), drivePower);
        }

        switch (state) {
            case start:
                if (currentGamepad1.right_bumper) {
                    arm.toTeleStartPosition();
                } if (currentGamepad1.left_bumper) {
                    intakeSlides.setPosition(0);
                    intake.setReadyPos();
                    state = State.ready;
                    stateTimer.reset();
                    newState = true;
                }
                break;

            case ready:
                if (newState) {
                    drivePower=MAX_DRIVE_POWER;
                    outtakeSlides.setPosition(0);
                    arm.toIdlePosition();
                    arm.openClaw();
                }
                newState=false;

                if (!sampleMode && currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                    stateTimer.reset();
                    newState = true;
                    state = State.intakeSpec;
                }

                switch (intakeState) {
                    case ready:
                        //manual slides control
                        double slidePower=currentGamepad1.right_trigger-currentGamepad1.left_trigger;
                        if(Math.abs(slidePower) > 0.1) {
                            intakeSlides.trySetPower(slidePower);
                        } else if (intakeSlides.horizontalSlidesMotor.getPower() != 0 && intakeSlides.getState() == HorizontalSlides.State.userControlled) { // in the dead zone and not running to target
                            intakeSlides.trySetPower(0);
                        }

                        //intake
                        if (currentGamepad1.right_bumper) {
                            turnPower = 0.6;
                            //move on if detecting sample
                            if (intake.smarterIntake(true)) {
                                intakeState = IntakeState.detected;
                                intakeTimer.reset();
                            }
                        } else if (currentGamepad1.cross) {
                            intake.reverseDown();
                            intake.reverse();
                        }else {
                            intake.up();
                            intakeTimer.reset();
                            intake.setPower(0);

                            if (intakeSlides.getPosition()<-350) { //slower turning when slides extended
                                turnPower = 0.6; //0.3+0.7*(1+intakeSlides.getPosition()/650)
                            } else {
                                turnPower = 1;
                            }
                        }

                        //manual retract
                        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                            intakeState = IntakeState.retract;
                            stateTimer.reset();
                            intakeTimer.reset();
                        }
                        break;
                    case detected: //spit out briefly in case there is two samples
                        intakeSlides.setPosition(0);
                        intake.closeDoor();
                        intake.reverseDown();
                        if (intakeTimer.milliseconds()<inTime) {
                            intake.intake();
                        } else if (intakeTimer.milliseconds()< (inTime+spitTime)) {
                            intake.slowReverse();
                        } else {
                            intakeState = IntakeState.retract;
                            stateTimer.reset();
                            intakeTimer.reset();
                        }
                        break;
                    case retract:
                        intake.intake();
                        intake.up();
                        intakeSlides.setPosition(0);
                        if (intakeSlides.getPosition()>-20 || currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                            intakeState = IntakeState.transfer;
                        }
                        break;
                    case transfer:
                        intake.stop();
                        intakeState = IntakeState.ready;
                        if (sampleMode) {
                            state = State.transfer;
                            stateTimer.reset();
                        }
                        break;
                }
                break;

            case transfer:
                if (newState) {
                    outtakeSlides.intakeSample();
                    arm.openClaw();
                    arm.intakeSample();
                }
                newState=false;

                if (currentGamepad1.circle) autoSampleDrive = true;

                if (stateTimer.milliseconds()<transferTime1) {
                    arm.intakeSample();
                    intake.closeDoor();
                    intake.intake();
                } else if (stateTimer.milliseconds()<transferTime2) {
                    arm.intakeSample();
                    intake.stop();
                    intake.openDoor();
                    arm.closeClaw();
                } else {
                    intakeSlides.setPosition(-150);
                    intake.reverse();
                    outtakeSlides.toHighBasket();
                    if (outtakeSlides.getCurrentPos()>(highBasketPos-700)) {
                        turnPower=0.6;
                        arm.outtakeSample();
                        intake.closeDoor();
                        state = State.outtakeSample;
                        stateTimer.reset();
                    }
                }
                break;
            case outtakeSample:
                intakeSlides.setPosition(-300);
                intake.stop();
                if (currentGamepad1.circle) autoSampleDrive = true;
                if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) { //rising edge detector
                    autoSampleDrive=false;
                    odometry.opt.setPos(sampleX, sampleY,odometry.opt.get_heading()); //(0,0,225)
                    arm.openClaw();
                    stateTimer.reset();
                    state = State.retract;
                }
                break;
            case retract:
                intakeSlides.setPosition(0);
                if (stateTimer.milliseconds()>100) {
                    arm.toIdlePosition();
                    if (odometry.opt.get_x()>(sampleX+4) || odometry.opt.get_y()<(sampleY-4)) { //auto retract to not lvl 4 hang
                        turnPower = 1;
                        state = State.ready;
                        newState=true;
                    } if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) { //failsafe
                        turnPower = 1;
                        state = State.ready;
                        newState=true;
                    }
                }
                break;

            case intakeSpec:
                if (newState) {
                    outtakeSlides.intakeSpecimen();
                    intakeSlides.setPosition(0);
                    arm.intakeSpecimen();
                    arm.openClaw();
                    intake.down();
                    outtakeSpec=false;
                }
                newState = false;

                if (currentGamepad1.right_trigger>0.3 || currentGamepad1.right_bumper) { //break out of spec cycle loop
                    state = State.ready;
                    stateTimer.reset();
                    newState = true;
                }

                if (currentGamepad1.square && !previousGamepad1.square) {
                    arm.closeClaw();
                    stateTimer.reset();
                    outtakeSpec=true;
                }
                if (outtakeSpec && stateTimer.milliseconds()>100) {
                    state = State.outtakeSpec;
                    stateTimer.reset();
                    newState = true;
                }
                break;

            case outtakeSpec:
                if (newState) {
                    outtakeSlides.toHighChamber();
                    arm.closeClaw();
                    arm.outtakeSpecimen1();
                    releaseSpec = false;
                }
                newState=false;

                if (previousGamepad1.left_bumper && !currentGamepad1.left_bumper) {
                    odometry.opt.setPos(40,odometry.opt.get_y());
                    arm.openClaw();
                    stateTimer.reset();
                    releaseSpec=true;
                }
                if (releaseSpec && stateTimer.milliseconds()>125) {
                    arm.openClaw();
                    arm.outtakeSpecimen2();
                    outtakeSlides.setPosition(0);

                    if (currentGamepad1.right_trigger>0.3 || currentGamepad1.right_bumper) {
                        state = State.ready;
                        stateTimer.reset();
                        newState = true;
                    }
                    if (odometry.opt.get_x()<24) {
                        state = State.intakeSpec;
                        stateTimer.reset();
                        newState = true;
                    }
                }
                break;
        }

        if(!previousGamepad1.dpad_up && currentGamepad1.dpad_up) {
            turnPower = 1;
            if(hangState == HangState.hanging1) {
                outtakeSlides.setPosition(hangHigh);
                hangState = HangState.hanging2;
            } else {
                outtakeSlides.setPosition(hangLow);
                hangState = HangState.hanging1;
            }
        }

        telemetry.addData("",alliance);
        telemetry.addData("Sample Mode", sampleMode);
        //telemetry.addData("intakeState", intakeState);
        //telemetry.addData("drive power", drivePower);
        //telemetry.addData("turn speed", turnPower);
        //telemetry.addData("intake slides pos", intakeSlides.getPosition());
        //telemetry.addData("heading", odometry.opt.get_heading());
    }

    enum State {
        start,
        ready,
        transfer,
        outtakeSample,
        retract,
        intakeSpec,
        outtakeSpec
    }

    enum IntakeState {
        ready,
        detected,
        retract,
        transfer
    }

    enum HangState {
        hanging1,
        hanging2
    }
}
