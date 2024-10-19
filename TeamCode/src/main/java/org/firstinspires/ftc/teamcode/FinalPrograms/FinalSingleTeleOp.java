package org.firstinspires.ftc.teamcode.FinalPrograms;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Lift;

import java.util.List;

@TeleOp
public class FinalSingleTeleOp extends OpMode {
    Odometry odometry;
    WheelControl drive;
    Lift lift;

    List<LynxModule> allHubs;

    double drivePower=1;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    State previousState;

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, 0, 0, 0, "BL", "FR", "FL");
        drive = new WheelControl(hardwareMap, odometry);
        lift = new Lift(hardwareMap);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public void loop() {
        // TODO toggleOuttake/ toggleIntake, beginHang
        // Updates
        odometry.update();
        lift.update();
        // The user controlled part
        State currentState = new State(gamepad1.right_stick_x,
                gamepad1.right_stick_y,
                gamepad1.left_stick_x,
                gamepad2.a,
                gamepad2.b,
                gamepad2.x,
                gamepad2.y,
                gamepad2.left_bumper || gamepad2.right_bumper,
                (gamepad2.dpad_up ? 1 : 0)  - (gamepad2.dpad_down ? 1 : 0),
                (gamepad1.dpad_up ? 1 : 0)  - (gamepad1.dpad_down ? 1 : 0),
                gamepad1.a,
                gamepad1.b);
        // Drive
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        drive.drive(-gamepad1.left_stick_y, 1.1*gamepad1.left_stick_x, gamepad1.right_stick_x*Math.abs(gamepad1.right_stick_x), 0, drivePower);

        // Outtake presets
        // TODO set appropriate presets
        if(!previousState.toLowBasket && currentState.toLowBasket) {
            lift.setPosition(100);
        } else if(!previousState.toHighBasket && currentState.toHighBasket) {
            lift.setPosition(100);
        } else if (!previousState.toLowChamber && currentState.toLowChamber) {
            lift.setPosition(100);

        } else if (!previousState.toHighChamber && currentState.toHighChamber) {
            lift.setPosition(100);
        }
        // Custom outtake input
        // TODO set a multiplier and config dead-zones
        if(currentState.outtakeSlidesInput > 0.1) {
            lift.setPower(currentState.outtakeSlidesInput);
        }

        previousState = new State(currentState);
    }

    public void outtakeControl() {

    }
}

class State {
    public double driveX; // = gamepad1.right_stick_x;;
    public double driveY; // = gamepad1.right_stick_y;
    public double rotate; // = gamepad1.left_stick_x;
    // Preset Outtake slides
    public boolean toLowChamber; // = gamepad2.a;
    public boolean toHighChamber; // = gamepad2.b;
    public boolean toLowBasket; // = gamepad2.x;
    public boolean toHighBasket; // = gamepad2.y;
    // Outtake stuff
    public boolean toggleOuttake; // = gamepad2.left_bumper || gamepad2.right_bumper; // will either be claw or bucket based on context
    // Slides custom input; claw is automatically controlled
    public double outtakeSlidesInput; // = (gamepad2.dpad_up ? 1 : 0)  - (gamepad2.dpad_down ? 1 : 0); // Done so they act in one direction if one is pressed, but cancel each other out
    public double intakeSlidesInput; // = (gamepad1.dpad_up ? 1 : 0)  - (gamepad1.dpad_down ? 1 : 0);
    // Intake
    public boolean toggleIntake; // = gamepad1.a; // Toggle intake spin. Automatically extends if it is not. Its transfer process is automatic.
    // Hang
    public boolean startHang; // = gamepad1.b; // Will only be available at endgame to prevent mistakes. Hang will be done based on stages. (e.g. press it again to go 1st to 2nd level then third. After, reset the hang.)
    public State(double driveX, double driveY, double rotate, boolean toLowChamber, boolean toHighChamber, boolean toLowBasket, boolean toHighBucket, boolean toggleOuttake, double outtakeSlidesInput, double intakeSlidesInput, boolean toggleIntake, boolean startHang) {
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
        this.toggleIntake = toggleIntake;
        this.startHang = startHang;
    }

    public State(State state) {
        this(state.driveX, state.driveY, state.rotate, state.toLowChamber, state.toHighChamber, state.toLowBasket, state.toHighBasket, state.toggleOuttake, state.outtakeSlidesInput, state.intakeSlidesInput, state.toggleIntake, state.startHang);
    }
}