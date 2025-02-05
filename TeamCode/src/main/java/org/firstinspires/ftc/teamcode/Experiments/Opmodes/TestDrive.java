package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;

import java.util.List;

@TeleOp
@Disabled
public class TestDrive extends OpMode {
    Odometry odometry;
    WheelControl drive;

    List<LynxModule> allHubs;

    int bulkReadMode = 0;
    int loopCounter = 0;
    double totalLoopTime;
    double averageLoopTime;

    double powerLevel=1;

    ElapsedTime timer;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    @Override
    public void init() {
        odometry = new Odometry(hardwareMap, 0, 0, 0, "BL", "FR", "FL");
        drive = new WheelControl(hardwareMap, odometry);

        allHubs = hardwareMap.getAll(LynxModule.class);

        timer = new ElapsedTime();
    }

    @Override
    public void init_loop() {
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) bulkReadMode -= 1;
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) bulkReadMode += 1;

        if (bulkReadMode < 0) bulkReadMode = 0;
        if (bulkReadMode > 1) bulkReadMode = 1;

        switch (bulkReadMode) {
            case 0:
                for (LynxModule hub : allHubs) {
                    hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
                }
                break;
            case 1:
                for (LynxModule hub : allHubs) {
                    hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
                }
                break;
        }

        telemetry.addData("Bulk Read Mode", bulkReadMode);
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) powerLevel -= 0.1;
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) powerLevel += 0.1;

        if (gamepad1.a) drive.BL.setPower(0.5);
        if (gamepad1.b) drive.BR.setPower(0.5);
        if (gamepad1.x) drive.FL.setPower(0.5);
        if (gamepad1.y) drive.FR.setPower(0.5);

        odometry.update();
        drive.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x, 0, powerLevel);

        loopCounter += 1;
        totalLoopTime += timer.milliseconds();
        averageLoopTime = totalLoopTime / loopCounter;
        timer.reset();

        telemetry.addData("Bulk Read Mode", bulkReadMode);
        telemetry.addData("Average Loop Time (ms)", averageLoopTime);

        telemetry.addData("power level", powerLevel);

        telemetry.addData("xPos", odometry.getxPos());
        telemetry.addData("yPos", odometry.getyPos());
        telemetry.addData("heading", odometry.getHeading());
    }
}
