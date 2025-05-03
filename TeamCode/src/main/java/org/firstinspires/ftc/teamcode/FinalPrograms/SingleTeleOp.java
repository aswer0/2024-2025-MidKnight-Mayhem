package org.firstinspires.ftc.teamcode.FinalPrograms;

import static org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Lift.highBasketPos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Arm;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Alliance;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Sensors;

import java.util.List;

@TeleOp
@Config
public class SingleTeleOp extends OpMode {
    Odometry odometry;
    WheelControl drive;
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

    public static double MAX_DRIVE_POWER=0.5;

    double drivePower=MAX_DRIVE_POWER;
    double turnPower=1;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, 90, 0, 0, "OTOS");
        drive = new WheelControl(hardwareMap, odometry);
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
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        // Updates
        odometry.opt.update();
        outtakeSlides.update();
        intakeSlides.update();
        intake.hasCorrectSample(true);

        drive.drive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x*turnPower,0, drivePower);

        switch (state) {
            case start:
                if (currentGamepad1.left_bumper) {
                    intakeSlides.setPosition(0);
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

                //slower when slides extended
                //if (intakeSlides.getPosition()<-250) drivePower = MAX_DRIVE_POWER*1; //(0.6+0.4*(1+intakeSlides.getPosition()/650))

                switch (intakeState) {
                    case ready:
                        //manual slides control
                        double slidePower=currentGamepad1.right_trigger-currentGamepad1.left_trigger;
                        if(Math.abs(slidePower) > 0.1) {
                            intakeSlides.trySetPower(slidePower);
                        } else if (intakeSlides.horizontalSlidesMotor.getPower() != 0 && intakeSlides.getState() == HorizontalSlides.State.userControlled) { // in the dead zone and not running to target
                            intakeSlides.trySetPower(0);
                        }
                        //preset horizontal extension
                        if (currentGamepad1.circle && !previousGamepad1.circle) {
                            intakeSlides.setPosition(-400);
                        }
                        //intake
                        if (currentGamepad1.right_bumper) {
                            turnPower=0.6;
                            intake.down();
                            intake.smartIntake(true);
                            intake.closeDoor();
                            //move on if detecting sample
                            if (intake.hasCorrectSample(true)) {
                                intakeState = IntakeState.detected;
                                intakeTimer.reset();
                            }
                        } else {
                            intake.up();
                            intakeTimer.reset();
                            intake.setPower(0);

                            if (intakeSlides.getPosition()<-350) { //slower turning when slides extended
                                turnPower = 0.6; //0.3+0.7*(1+intakeSlides.getPosition()/650)
                            } else {
                                turnPower = 1;
                            }
                        }
                        //failsafe retract
                        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                            intakeState = IntakeState.retract;
                            stateTimer.reset();
                            intakeTimer.reset();
                        }
                        break;
                    case detected: //spit out briefly in case there is two samples
                        intake.closeDoor();
                        intake.reverseDown();
                        if (intakeTimer.milliseconds()<70) {
                            intake.reverse();
                        } else if (intakeTimer.milliseconds()<150) {
                            intake.intake();
                        } else {
                            intakeState = IntakeState.retract;
                            stateTimer.reset();
                            intakeTimer.reset();
                        }
                        break;
                    case retract:
                        intake.stop();
                        intake.up();
                        intakeSlides.setPosition(0);
                        if (intakeSlides.getPosition()>-10 || currentGamepad1.left_bumper && !previousGamepad1.left_bumper) intakeState = IntakeState.transfer;
                        break;
                    case transfer:
                        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                            state = State.transfer;
                            intakeState = IntakeState.ready;
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

                if (stateTimer.milliseconds()<200) {
                    arm.intakeSample();
                    intake.closeDoor();
                    intake.intake();
                } else if (stateTimer.milliseconds()<600) {
                    arm.intakeSample();
                    intake.stop();
                    intake.openDoor();
                    arm.closeClaw();
                } else {
                    outtakeSlides.toHighBasket();
                    if (outtakeSlides.getPosition()>(highBasketPos-400)) {
                        //arm.halfOpenClaw(); //LRG depo method (sideways)
                        arm.outtakeSample();
                        intake.closeDoor();
                        state = State.outtake;
                        stateTimer.reset();
                    }
                }
                break;
            case outtake:
                if (!currentGamepad1.left_bumper && previousGamepad1.left_bumper) { //falling edge detector
                    turnPower=0.6;
                    odometry.opt.setPos(0,0,225);
                    arm.openClaw();
                    stateTimer.reset();
                    state = State.retract;
                }
                break;
            case retract:
                if (stateTimer.milliseconds()>100) {
                    arm.toIdlePosition();
                    if (odometry.opt.get_x()>4 || odometry.opt.get_y()>4) { //auto retract to not lvl 4 hang
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
        }
        telemetry.addData("state", state);
        telemetry.addData("intake state", intakeState);
        telemetry.addData("drive power", drivePower);
        telemetry.addData("turn speed", turnPower);
        telemetry.addData("intake slides pos", intakeSlides.getPosition());
    }

    enum State {
        start,
        ready,
        transfer,
        outtake,
        retract
    }

    enum IntakeState {
        ready,
        detected,
        retract,
        transfer
    }
}
