package org.firstinspires.ftc.teamcode.FinalPrograms;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;

import java.util.List;

@TeleOp
public class FinalSingleTeleOp extends OpMode {
    Odometry odometry;
    WheelControl drive;

    List<LynxModule> allHubs;

    double drivePower=1;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, 0, 0, 0, "BL", "FR", "FL");
        drive = new WheelControl(hardwareMap, odometry);

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        drive.drive(-gamepad1.left_stick_y, 1.1*gamepad1.left_stick_x, gamepad1.right_stick_x*Math.abs(gamepad1.right_stick_x), 0, drivePower);
    }

    public void outtakeControl() {

    }
}
