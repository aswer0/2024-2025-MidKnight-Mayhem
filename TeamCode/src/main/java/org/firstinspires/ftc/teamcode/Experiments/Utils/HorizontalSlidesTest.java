package org.firstinspires.ftc.teamcode.Experiments.Utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.DriveCorrection;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.GVF.VectorField;
import org.firstinspires.ftc.teamcode.Experiments.Utils.utils;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.Odometry;
import org.firstinspires.ftc.teamcode.Experiments.Drivetrain.WheelControl;
import org.firstinspires.ftc.teamcode.Experiments.Utils.PIDController;
import org.firstinspires.ftc.teamcode.FinalPrograms.Subsystems.Intake.HorizontalSlides;
import org.opencv.core.Point;

@TeleOp
@Config
public class HorizontalSlidesTest extends OpMode {
    HorizontalSlides horizontalSlides;
    ElapsedTime timer;

    double powerLevel = 1;
    public static double pos = -500;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    State state = State.setPos;

    enum State {
        setPos,
        back,
    }

    @Override
    public void init(){
        horizontalSlides = new HorizontalSlides(hardwareMap);
        timer = new ElapsedTime();
    }

    @Override
    public void init_loop(){
        timer.reset();
    }

    @Override
    public void loop() {
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
            timer.reset();
            state = State.setPos;
        }

        horizontalSlides.update();

        switch (state){
            case setPos:
                horizontalSlides.setPosition(pos);
                if (timer.milliseconds() >= 5000){
                    timer.reset();
                    state = State.back;
                }
                break;

            case back:
                horizontalSlides.setPosition(0);
                break;
        }

        telemetry.addData("position", horizontalSlides.getPosition());
        telemetry.addData("state", state);
        telemetry.addData("timer", timer.milliseconds());

        telemetry.update();
    }
}