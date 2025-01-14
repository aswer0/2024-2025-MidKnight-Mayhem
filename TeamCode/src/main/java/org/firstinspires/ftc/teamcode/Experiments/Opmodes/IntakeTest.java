package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.HorizontalSlides;
import org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.Experiments.Utils.Sensors;

@TeleOp
public class IntakeTest extends OpMode {
    //CRServo servo1;
    //CRServo servo2;

    HorizontalSlides intakeSlides;
    Intake intake;

    ElapsedTime intakeTimer;

    public void init(){
        //servo1 = hardwareMap.get(CRServo.class, "1");
        //servo2 = hardwareMap.get(CRServo.class, "2");
        intakeSlides = new HorizontalSlides(hardwareMap);
        intake = new Intake(hardwareMap, new Sensors(hardwareMap, telemetry));

        intakeTimer = new ElapsedTime();
    }

    public void loop(){
        //double y_pos1 = gamepad1.left_stick_y;
        //double y_pos2 = gamepad1.right_stick_y;
        //servo1.setPower(y_pos1);
        //servo2.setPower(-y_pos1);

        //manual horizontal extension
        if(Math.abs(-gamepad2.right_stick_y) > 0.1) {
            intakeSlides.trySetPower(-gamepad2.right_stick_y*0.5);
        }

        //manual intake
        if (gamepad2.right_trigger-gamepad2.left_trigger>0.7) {
            intake.intake();
            intake.down();
        } else if (gamepad2.right_trigger-gamepad2.left_trigger<-0.7){
            intake.down();
            if (intakeTimer.milliseconds()>150) {
                intake.reverse();
            }
        } else {
            intakeTimer.reset();
            intake.setPower(0);
            intake.up();
        }

    }

}
