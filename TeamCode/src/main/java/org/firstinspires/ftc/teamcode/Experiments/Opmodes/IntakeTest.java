package org.firstinspires.ftc.teamcode.Experiments.Opmodes;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class IntakeTest extends OpMode {

    CRServo servo1;
    CRServo servo2;

    public void init(){
        servo1 = hardwareMap.get(CRServo.class, "1");
        servo2 = hardwareMap.get(CRServo.class, "2");
    }

    public void loop(){
        double y_pos = gamepad1.left_stick_y;

        servo1.setPower(y_pos);
        servo2.setPower(-y_pos);

        telemetry.addData("y position joystick", y_pos);
    }

}
