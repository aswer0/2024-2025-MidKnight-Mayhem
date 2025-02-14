package org.firstinspires.ftc.teamcode.Experiments.Utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ActionTimer {
    boolean running;
    ElapsedTime timer;

    public ActionTimer() {
        running = false;
        timer = new ElapsedTime();
    }

    // Only starts timer the first time this is called after reset
    public void startFirstOnly() {
        if (!running) {
            timer.reset();
            running = true;
        }
    }

    public void reset() {
        running = false;
        timer.reset();
    }

    public double milliseconds() throws Exception {
        if (!running) throw new Exception("Timer is stopped");
        return timer.milliseconds();
    }

    public double seconds() throws Exception {
        if (!running) throw new Exception("Timer is stopped");
        return timer.seconds();
    }

    public boolean overMilliseconds(double milliseconds) {
        if (!running) return false;
        return timer.milliseconds() >= milliseconds;
    }
}
