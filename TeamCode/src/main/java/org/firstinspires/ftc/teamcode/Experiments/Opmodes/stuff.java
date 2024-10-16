package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Hang;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Outtake.Manipulator;

/*
Config:

Motors:
BR, FR, BL, FL (Drive)
horizontalSlidesMotor, intakeMotor (Intake)
hangLeft, hangRight
SlideLeft, SlideRight


 */
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

    // Driving
    WheelControl wheelControl;
    Odometry odometry;

    Hang hang;

    HorizontalSlides slides;
    Intake intake;

    Lift lift;
    Manipulator manipulator;

    @Override
    public void init() {
        wheelControl = new WheelControl(hardwareMap, odometry);

    }
    @Override
    public void loop() {

        // Keybinds Driver 1 is responsible for intake/hang, driver 2 is responsible for outtake.
        // Driving (Both drivers being able to drive may be possible)
        double driveX = gamepad1.right_stick_x;;
        double driveY = gamepad1.right_stick_y;
        double rotate = gamepad1.left_stick_x;
        // Preset Outtake slides
        boolean toLowChamber = gamepad2.a;
        boolean toHighChamber = gamepad2.b;
        boolean toLowBasket = gamepad2.x;
        boolean toHighBucket = gamepad2.y;
        // Outtake stuff
        boolean toggleOuttake = gamepad2.left_bumper || gamepad2.right_bumper; // will either be claw or bucket based on context
        // Slides custom input; claw is automatically controlled
        double outtakeSlidesInput = (gamepad2.dpad_up ? 1 : 0)  - (gamepad2.dpad_down ? 1 : 0); // Done so they act in one direction if one is pressed, but cancel each other out
        double intakeSlidesInput = (gamepad1.dpad_up ? 1 : 0)  - (gamepad1.dpad_down ? 1 : 0);
        // Intake
        boolean toggleIntake = gamepad1.a; // Toggle intake spin. Automatically extends if it is not. Its transfer process is automatic.
        // Hang
        boolean startHang = gamepad1.b; // Will only be available at endgame to prevent mistakes. Hang will be done based on stages. (e.g. press it again to go 1st to 2nd level then third. After, reset the hang.)



        // TODO: implement driving

        // TODO: implement hang subsys

        // TODO:
    }

}
