package org.firstinspires.ftc.teamcode.FinalPrograms;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Manipulator;

import java.util.List;

@TeleOp
public class FinalTeleOp extends OpMode {
    ElapsedTime intakeTimer = new ElapsedTime();
    Odometry odometry;
    WheelControl drive;

    Lift outtakeSlides;
    Manipulator manipulator;
    boolean clawOpen = false;
    double autoGrabGracePeriod = 0;

    Intake intake;
    HorizontalSlides intakeSlides;
    IntakingState intakeState = IntakingState.inactive;



    List<LynxModule> allHubs;

    double drivePower=1;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    State previousState = new State(0,0,0,false, false, false, false, false, 0,0, 0, false);

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, 0, 0, 0, "BL", "FR", "FL");
        drive = new WheelControl(hardwareMap, odometry);
        outtakeSlides = new Lift(hardwareMap);

        intake = new Intake(hardwareMap);
        intakeSlides = new HorizontalSlides(hardwareMap);
        manipulator = new Manipulator(hardwareMap);

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
        odometry.update();
        outtakeSlides.update();
        //intakeSlides.update(); FIXME
        //intake.update(); TODO LM2 Automation

        // The user controlled part
        State currentState = new State(1.1*gamepad1.left_stick_x, // drive X
                gamepad1.left_stick_y, // Drive Y
                -gamepad1.right_stick_x, // Drive rotate
                gamepad2.cross, // toLowChamber
                gamepad2.square, // high chamber
                gamepad2.circle, // low basket
                gamepad2.triangle, // high basket
                gamepad2.right_bumper, // toggle outtake
                -gamepad2.left_stick_y, // outtake slides
                gamepad1.right_trigger - gamepad1.left_trigger, // intake slides
                (gamepad1.right_bumper ? 1 : 0) - (gamepad1.left_bumper ? 1 : 0), // intake input
                gamepad2.share); // start hang

        // Drive
        drive.drive(currentState.driveY, currentState.driveX, currentState.rotate, 0, drivePower);

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

        if(!previousState.toggleOuttake && currentState.toggleOuttake) { // TODO bucket logic
            if (clawOpen) {
                manipulator.closeClaw();
            } else {
                //outtakeSlides.setPosition(outtakeSlides.leftSlide.getCurrentPosition() - 150);
                manipulator.openClaw();
                autoGrabGracePeriod = getRuntime() + 0.25;
            }
            clawOpen = !clawOpen;
        }
        // Custom outtake input

        if(Math.abs(currentState.outtakeSlidesInput) > 0.4) {
            outtakeSlides.setPower(currentState.outtakeSlidesInput*0.5);
        } else if (outtakeSlides.leftSlide.getPower() != 0 && outtakeSlides.getState() == Lift.State.userControlled) {
            outtakeSlides.setPower(0);
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
            intakeSlides.setPower(previousState.intakeSlidesInput*0.5);
        } else {
            intakeSlides.setPower(0);
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
        previousState = new State(currentState);
    }

    public void outtakeControl() {

    }
    private static class State {
        public double driveX;
        public double driveY;
        public double rotate;
        // Preset Outtake Slides
        public boolean toLowChamber;
        public boolean toHighChamber;
        public boolean toLowBasket;
        public boolean toHighBasket;
        // Outtake stuff
        public boolean toggleOuttake; // will either be claw or bucket based on context
        // Slides custom input; claw is automatically controlled
        public double outtakeSlidesInput; // Done so they act in one direction if one is pressed, but cancel each other out

        // Intake
        public double intakeSlidesInput;
        public double intakeInput;

        public boolean startHang; // Will only be available at endgame to prevent mistakes. Hang will be done based on stages. (e.g. press it again to go 1st to 2nd level then third. After, reset the hang.)
        public State(double driveX,
                     double driveY,
                     double rotate,
                     boolean toLowChamber,
                     boolean toHighChamber,
                     boolean toLowBasket,
                     boolean toHighBucket,
                     boolean toggleOuttake,
                     double outtakeSlidesInput,
                     double intakeSlidesInput,
                     double intakeInput,
                      boolean startHang) {
            this.driveX = driveX;
            this.driveY = driveY;
            this.rotate = rotate;
            this.toLowChamber = toLowChamber;
            this.toHighChamber = toHighChamber;
            this.toLowBasket = toLowBasket;
            this.toHighBasket = toHighBucket;
            this.toggleOuttake = toggleOuttake;
            this.outtakeSlidesInput = outtakeSlidesInput;
            this.intakeSlidesInput = intakeSlidesInput;
            this.intakeInput = intakeInput;
            this.startHang = startHang;
        }

        public State(State state) {
            this(state.driveX, state.driveY, state.rotate, state.toLowChamber, state.toHighChamber, state.toLowBasket, state.toHighBasket, state.toggleOuttake, state.outtakeSlidesInput, state.intakeSlidesInput, state.intakeInput, state.startHang);
        }
    }

    enum IntakingState {
        inactive, // zero power; intake + slides fully retracted; dpad input is inactive
        userControlled, // transitions when toggle intake is on
        retractSlides, // horiz and vert at the same time
        transfer // rotate the pivot & reverse the intake
    }
    
}
