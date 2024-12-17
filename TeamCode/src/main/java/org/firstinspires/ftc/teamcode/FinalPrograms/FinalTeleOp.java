package org.firstinspires.ftc.teamcode.FinalPrograms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Manipulator;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Alliance;

import java.util.List;

@TeleOp
public class FinalTeleOp extends OpMode {
    double lastTime = getRuntime();

    ElapsedTime intakeTimer = new ElapsedTime();
    Odometry odometry;
    WheelControl drive;
    Alliance alliance = Alliance.red;

    Lift outtakeSlides;
    Manipulator manipulator;
    boolean clawOpen = true;
    double autoGrabGracePeriod = 0;
    double autoDepositLimit = Double.POSITIVE_INFINITY;
    OuttakeState outtakeState = OuttakeState.inactive;
    double outtakeLastPosition = 0;
    double outtakeDepositBy = 0.5;

    Intake intake;
    HorizontalSlides intakeSlides;
    IntakingState intakeState = IntakingState.inactive;



    List<LynxModule> allHubs;

    double drivePower=1;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    State previousState = new State();

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, 180, 0, 0, "OTOS");
        drive = new WheelControl(hardwareMap, odometry);
        outtakeSlides = new Lift(hardwareMap, false);
        outtakeSlides.brakeSlides(true);

        intake = new Intake(hardwareMap);
        intakeSlides = new HorizontalSlides(hardwareMap);
        manipulator = new Manipulator(hardwareMap);
        manipulator.openClaw();

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public void loop() {
        // TODO beginHang, intake by LM2,
        // TODO outtake control by LM1
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);

        // Updates
        odometry.opt.update();
        outtakeSlides.update();
        intakeSlides.update();
        //intake.update(); TODO LM2 Automation
        // The user controlled part
        State currentState = new State();
        currentState.driveX = 1.1*gamepad1.left_stick_x; // drive X
        currentState.driveY = gamepad1.left_stick_y; // Drive Y
        currentState.rotate = -gamepad1.right_stick_x*Math.abs(gamepad1.right_stick_x)*0.8; // Drive rotate
        currentState.reAlignFieldOriented = gamepad1.options; // realign Field Oriented
        currentState.toggleAlliance = gamepad1.share;

        currentState.toLowChamber = gamepad2.cross; // toLowChamber
        currentState.toHighChamber = gamepad2.square; // high chamber
        currentState.toLowBasket = gamepad2.circle; // low basket
        currentState.toHighBasket = gamepad2.triangle; // high basket
        currentState.toggleOuttake = gamepad2.right_bumper; // toggle outtake
        currentState.outtakeSlidesInput = -gamepad2.left_stick_y; // outtake slides
        currentState.resetOuttakeSlides = gamepad2.options; // reset Outtake

        currentState.intakeSlidesInput = -gamepad2.right_stick_y; // intake slides
        currentState.intakeInput = gamepad2.right_trigger-gamepad2.left_trigger; // intake input
        currentState.depositSpecimen = gamepad2.left_bumper; // deposit specimen
        currentState.startHang = gamepad2.share;
        // Drive
        drive.drive(currentState.driveY, currentState.driveX, currentState.rotate, Math.toRadians(odometry.opt.get_heading()), drivePower);
        if(!previousState.reAlignFieldOriented && currentState.reAlignFieldOriented) {
            odometry.opt.setPos(odometry.opt.get_x(), odometry.opt.get_y(), 180);
        }
        if(!previousState.resetOuttakeSlides && currentState.resetOuttakeSlides) {
            outtakeSlides.leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            outtakeSlides.leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        for(LynxModule test: allHubs){
            test.clearBulkCache();
        }
        switch(outtakeState) {
            case inactive:
                // Outtake presets
                // TODO set outtake presets
                if(!previousState.toLowBasket && currentState.toLowBasket) {
                    outtakeSlides.toLowBasket();
                } else if(!previousState.toHighBasket && currentState.toHighBasket) {
                    outtakeSlides.toHighBasket();
                } else if (!previousState.toLowChamber && currentState.toLowChamber) {
                    outtakeSlides.toLowChamber();
                } else if (!previousState.toHighChamber && currentState.toHighChamber) {
                    outtakeSlides.toHighChamber();
                }
                if(Math.abs(currentState.outtakeSlidesInput) > 0.4) {
                    outtakeSlides.trySetPower(currentState.outtakeSlidesInput*1);
                } else if (outtakeSlides.leftSlide.getPower() != 0 && outtakeSlides.getState() == Lift.State.userControlled) {
                    outtakeSlides.trySetPower(0);
                }

                if(!previousState.toggleOuttake && currentState.toggleOuttake) { // TODO bucket logic
                    if (clawOpen) {
                        manipulator.closeClaw();
                    } else {
                        manipulator.openClaw();;
                        autoGrabGracePeriod = getRuntime() + 0.25;
                    }
                    clawOpen = !clawOpen;
                }
                if(!previousState.depositSpecimen && currentState.depositSpecimen && !clawOpen) {
                    autoDepositLimit = outtakeSlides.leftSlide.getCurrentPosition() - 300;
                    outtakeLastPosition = outtakeSlides.leftSlide.getCurrentPosition();
                    outtakeState = OuttakeState.depositingSpecimen;
                }
                if(!previousState.startHang && currentState.startHang && getRuntime() > 80) {
                    outtakeState = OuttakeState.hangingStage1;
                }
                break;
            case depositingSpecimen:
                // Custom outtake input
                TelemetryPacket packet = new TelemetryPacket();
                packet.put("Derivative", (outtakeSlides.leftSlide.getCurrentPosition() - outtakeLastPosition)/(getRuntime() - lastTime));
                packet.put("Seconds", outtakeDepositBy);
                (FtcDashboard.getInstance()).sendTelemetryPacket(packet);
                outtakeSlides.setPower(-0.35);
                if((outtakeSlides.leftSlide.getCurrentPosition() - outtakeLastPosition)/(getRuntime() - lastTime) > -300) {
                    outtakeDepositBy -= (getRuntime() - lastTime);
                }
                if(Math.abs(currentState.outtakeSlidesInput) > 0.6 || (!previousState.depositSpecimen && currentState.depositSpecimen)) {
                    outtakeState = OuttakeState.inactive;
                }
                if(outtakeSlides.leftSlide.getCurrentPosition() < autoDepositLimit) {
                    outtakeState = OuttakeState.inactive;
                }
                if(outtakeDepositBy < 0 || currentState.toggleOuttake) {
                    outtakeDepositBy = 1;
                    manipulator.openClaw();
                    clawOpen = true;
                    outtakeState = OuttakeState.inactive;
                }
                outtakeLastPosition = outtakeSlides.leftSlide.getCurrentPosition();

                break;
            case hangingStage1:
                outtakeSlides.setPosition(0);
                break;
        }
        // Autograb (only when the slides are low enough) TODO by lm2
//        if(manipulator.clawHasObject() && clawOpen
//                && outtakeSlides.leftSlide.getCurrentPosition() < 200
//                && autoGrabGracePeriod - getRuntime() < 0) {
//            manipulator.closeClaw();
//            autoGrabGracePeriod = getRuntime() + 0.25;
//            clawOpen = false;
//        }

        //manual horizontal extension
        if(Math.abs(previousState.intakeSlidesInput) > 0.1) {
            intakeSlides.trySetPower(previousState.intakeSlidesInput);
        } else if (intakeSlides.horizontalSlidesMotor.getPower() != 0 && intakeSlides.getState() == HorizontalSlides.State.userControlled) { // in the dead zone and not running to target
            intakeSlides.trySetPower(0);
        }

        //manual intake
        if (currentState.intakeInput>0.7) {
            intake.down();
            intake.intake();
        } else if (currentState.intakeInput<-0.7){
            intake.down();
            if (intakeTimer.milliseconds()>150) {
                intake.reverse();
            }
        } else {
            intakeTimer.reset();
            intake.setPower(0);
            intake.up();
        }

        //Intake
        // TODO uncomment after LM1
//        switch(intakeState) {
//            case inactive:
//                intakeSlides.setPosition(0);
//                intake.setPivot(0); //TODO intake retracted and not retracted position
//                intake.intaking = false;
//                if(!previousState.toggleIntake && currentState.toggleIntake) {
//                    intakeSlides.setPosition(30); // only on the transition
//                    intakeState = IntakingState.userControlled;
//                }
//                break;
//            case userControlled:
//                intake.setPivot(10);
//                if(currentState.intakeSlidesInput > 0.1) {
//                    intakeSlides.setPower(currentState.intakeSlidesInput*0.5);
//                }
//                if(intake.hasCorrectObject) {
//                    intakeState = IntakingState.retractSlides;
//                }
//                if(!previousState.toggleIntake && currentState.toggleIntake) {
//                    intakeState = IntakingState.inactive;
//                }
//                break;
//            case retractSlides:
//                intakeSlides.setPosition(0);
//                outtakeSlides.setPosition(0); // TODO reject all outtake inputs
//                if(intakeSlides.horizontalSlidesMotor.getCurrentPosition() < 10
//                        && outtakeSlides.rightSlide.getCurrentPosition() < 10
//                        && outtakeSlides.leftSlide.getCurrentPosition() < 10) {
//                    intakeState = IntakingState.transfer;
//                }
//                break;
//            case transfer:
//                intake.setPivot(0);
//                if(!intake.hasCorrectObject ) {// The object's in the bucket
//                    intakeState = IntakingState.inactive; // TODO maybe add a pause
//                }
//                break;
//        }
        lastTime = getRuntime();
        previousState = currentState;
    }

    public void outtakeControl() {

    }
    private static class State {
        public double driveX = 0;
        public double driveY = 0;
        public double rotate = 0;
        public boolean reAlignFieldOriented = false;
        public boolean toggleAlliance = false;
        // Preset Outtake Slides
        public boolean toLowChamber = false;
        public boolean toHighChamber = false;
        public boolean toLowBasket = false;
        public boolean toHighBasket = false;
        // Outtake stuff
        public boolean toggleOuttake = false; // will either be claw or bucket based on context
        // Slides custom input; claw is automatically controlled
        public double outtakeSlidesInput = 0; // Done so they act in one direction if one is pressed, but cancel each other out
        public boolean resetOuttakeSlides = false;

        // Intake
        public double intakeSlidesInput = 0;
        public double intakeInput = 0;
        public boolean depositSpecimen = false;

        public boolean startHang = false; // Will only be available at endgame to prevent mistakes. Hang will be done based on stages. (e.g. press it again to go 1st to 2nd level then third. After, reset the hang.)
    }

    enum IntakingState {
        inactive, // zero power; intake + slides fully retracted; dpad input is inactive
        userControlled, // transitions when toggle intake is on
        retractSlides, // horiz and vert at the same time
        transfer // rotate the pivot & reverse the intake
    }

    enum OuttakeState {
        inactive,
        depositingSpecimen, // break if it is 300 below, or derivative is zero for 0.5 seconds.
        hangingStage1
    }
    
}
