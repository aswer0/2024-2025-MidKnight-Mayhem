package org.firstinspires.ftc.teamcode.Experiments.Subsystems.Intake;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeLogic {
    Intake intake;
    HorizontalSlides horizontalSlides;
    private enum State {
        userControlled,
        retractIntake,
        transfer
    }
    State state = State.userControlled;
    public IntakeLogic(HardwareMap hardwareMap) {
        intake = new Intake(hardwareMap);
        horizontalSlides = new HorizontalSlides(hardwareMap);
    }

    public void update(boolean possessingObject) {
        horizontalSlides.update();
        intake.update(possessingObject);
        // TODO auto-transfer
        switch(state) {
            case userControlled:
                if(intake.hasObject) state = State.retractIntake;
                break;
            case retractIntake:
                horizontalSlides.setPosition(0);
                if(horizontalSlides.horizontalSlidesMotor.getCurrentPosition() < 10) state = State.transfer;
                break;
            case transfer:
                horizontalSlides.setPosition(0);
                intake.intakePivot.setPosition(0);
                if(intake.intakePivot.getPosition() < 10) state = State.userControlled;
                break;
        }
    }
}
