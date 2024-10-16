package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class stuff extends OpMode {
    enum DrivingStates {
        userControlled,


       /*intakeSample,
        driveSample,
        outtakeSample,
        returnSample,*/


    }

    enum intakeStates {

    }

    @Override
    public void init() {
    }
    @Override
    public void start() {

    }
    @Override
    public void loop() {

        // Keybinds
        // Driving
        double driveX = gamepad1.right_stick_x;;
        double driveY = gamepad1.right_stick_y;
        double rotate = gamepad1.left_stick_x;
        // Preset Outtake slides
        boolean toLowChamber = gamepad2.a;
        boolean toHighChamber = gamepad2.b;
        boolean toLowBasket = gamepad2.x;
        boolean toHighBucket = gamepad2.y;
        // Slides custom input
        double outtakeSlidesInput = (gamepad2.dpad_up ? 1 : 0)  - (gamepad2.dpad_down ? 1 : 0); // Done so they act in one direction if one is pressed, but cancel each other out
        double intakeSlidesInput = (gamepad1.dpad_up ? 1 : 0)  - (gamepad1.dpad_down ? 1 : 0);
        // Intake
        boolean toggleIntake = gamepad1.a; // Intake will spin when it is sufficiently extended, and will automatically transfer. The intake will not extend when an element is possessed.
        // Hang
        boolean startHang = gamepad1.b; // Will only be available at the last 30 seconds. Hang will be done based on stages. (e.g. press it again to go 1st to 2nd level, etc.)



        // TODO: implement driving

        // TODO: implement hang subsys

        // TODO:
    }

}
