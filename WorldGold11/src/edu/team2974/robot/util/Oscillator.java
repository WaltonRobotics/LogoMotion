package edu.team2974.robot.util;

import edu.team2974.robot.arm.Arm;

class Oscillator extends AutoFunction
{

    Arm arm;

    private Oscillator(Arm arm) {
        this.arm = arm;
    }

    public void run() {
        //stops oscillating to start "fresh"
        arm.setOscillating(false);
        //repeatedly call oscillation method- note that periodicity (not calling too often) is handled in
        //the oscillate method itself, so this loop doesn't have to have any sort of timer delay.
        while (isRunning()) {
            arm.armOscillate();
        }
    }

    public boolean atCorrectHeight() {
        return false;
    }

    public void stop() {
        super.stop();
    }
}
