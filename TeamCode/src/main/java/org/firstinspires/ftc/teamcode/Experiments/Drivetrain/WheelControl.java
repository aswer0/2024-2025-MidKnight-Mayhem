package org.firstinspires.ftc.teamcode.Experiments.Drivetrain;

import android.sax.StartElementListener;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Experiments.Utils.utils;

//
public class WheelControl {
    public DcMotorEx BR;
    public DcMotorEx BL;
    public DcMotorEx FR;
    public DcMotorEx FL;
    private VoltageSensor voltageSensor;

    Odometry odometry;
    DriveCorrection driveCorrection;
    double target_angle = -180;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    public WheelControl(HardwareMap hardwareMap, Odometry odometry) {
        this.BR = hardwareMap.get(DcMotorEx.class, "BR");
        this.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.FR = hardwareMap.get(DcMotorEx.class, "FR");
        this.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.BL = hardwareMap.get(DcMotorEx.class, "BL");
        this.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.FL = hardwareMap.get(DcMotorEx.class, "FL");
        this.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.BL.setDirection(DcMotorEx.Direction.REVERSE);
        this.FL.setDirection(DcMotorEx.Direction.REVERSE);

        this.odometry = odometry;
        this.voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        this.driveCorrection = new DriveCorrection(odometry);
    }
    public void setPowers(double BL, double BR, double FL, double FR, double power) {
        double max = 1; // max motor power

        max = Math.max(BL, max);
        max = Math.max(BR, max);
        max = Math.max(FL, max);
        max = Math.max(FR, max); // Detect the motor with the most power
        this.BL.setPower(power * (BL/max));
        this.BR.setPower(power * (BR/max));
        this.FL.setPower(power * (FL/max));
        this.FR.setPower(power * (FR/max)); // We divide all values by the maximum one so they do not reach one.
    }

    /**
     *
     * @param forward Y component of the vector (robot oriented)
     * @param right   X component of the vector (robot oriented)
     * @param rotate  Rotation velocity (radians)
     * @param angle   The angle for where to rotate the thing. Get from odometry. (field oriented)
     */
    public void drive(double forward, double right, double rotate, double angle, double power) {
        double newX = right*Math.cos(angle) - forward*Math.sin(angle);
        double newY = right*Math.sin(angle) + forward*Math.cos(angle);

        double BLPower = newY + newX + rotate;
        double BRPower = newY - newX - rotate;
        double FLPower = newY - newX + rotate;
        double FRPower = newY + newX - rotate;
        //setPowers(BLPower, BRPower, FLPower, FRPower, power);

        double max = 1;
        max = Math.max(BLPower, max);
        max = Math.max(BRPower, max);
        max = Math.max(FLPower, max);
        max = Math.max(FRPower, max); // Detect the motor with the most power
        if (!(BLPower==0)) {
            this.BL.setPower(power * (BLPower/max) + 0.06*BLPower/Math.abs(BLPower));
        } else {
            this.BL.setPower(0);
        }
        if (!(BRPower==0)) {
            this.BR.setPower(power * (BRPower/max) + 0.06*BRPower/Math.abs(BRPower));
        } else {
            this.BR.setPower(0);
        }if (!(FLPower==0)) {
            this.FL.setPower(power * (FLPower/max) + 0.06*FLPower/Math.abs(FLPower));
        } else {
            this.FL.setPower(0);
        }if (!(FRPower==0)) {
            this.FR.setPower(power * (FRPower/max) + 0.06*FRPower/Math.abs(FRPower));
        } else {
            this.FR.setPower(0);
        }
    }

    public void change_mode(DcMotor.ZeroPowerBehavior mode){
        if (mode == DcMotor.ZeroPowerBehavior.BRAKE){
            this.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            this.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        else if (mode == DcMotor.ZeroPowerBehavior.FLOAT){
            this.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            this.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            this.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            this.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

   public void correction_drive(Gamepad gamepad1, double powerLevel, Telemetry telemetry){
       if (gamepad1.right_stick_x != 0){
           this.change_mode(DcMotor.ZeroPowerBehavior.BRAKE);

           this.drive(
                   -gamepad1.left_stick_y, gamepad1.left_stick_x,
                   driveCorrection.set_correction(gamepad1.right_stick_x), 0,
                   powerLevel
           );
       }
       else{
           this.change_mode(DcMotor.ZeroPowerBehavior.BRAKE);

           this.drive(
                   -gamepad1.left_stick_y, gamepad1.left_stick_x,
                   driveCorrection.stable_correction(gamepad1.right_stick_x), 0,
                   powerLevel
           );
       }
   }
}
