package edu.team2974.robot.util;

import edu.team2974.robot.arm.Arm;

/**
 *
 */
public class GoToHeight extends AutoFunction
{

    double goalHeight;
    private boolean farFromTarget;
    private boolean atHeight;
    Arm _arm;

    /**
     *
     * @param height
     * @param _arm
     */
    public GoToHeight(double height, Arm _arm) {
        this._arm = _arm;
        goalHeight = height;
        farFromTarget = true;
        atHeight = false;
    }

    /**
     *
     * @return
     */
    public boolean atCorrectHeight() {
        return atHeight;
    }

    /**
     *
     */
    public void run() {
        _arm.getForkliftEncoder().reset();
        _arm.getForkliftEncoder().start();
        while (isRunning()) {
            while (farFromTarget && isRunning()) {
                _arm.setCurrentHeight(
                        _arm.getCurrentHeight() + getLiftDistance());
                _arm.getForkliftEncoder().reset();
                if (Math.abs(_arm.getCurrentHeight() - goalHeight) < 2) {
                    farFromTarget = false;
                    atHeight = true;
                } else if (_arm.getCurrentHeight() < goalHeight) {
                    _arm.forkliftControl(1.0);
                } else {
                    _arm.forkliftControl(-0.2); //1/10 instead of whole unit because of gravity
                }
            }
            while (!farFromTarget && isRunning()) {
                _arm.setCurrentHeight(
                        _arm.getCurrentHeight() + getLiftDistance());
                _arm.getForkliftEncoder().reset();
                if (Math.abs(_arm.getCurrentHeight() - goalHeight) > 2) {
                    farFromTarget = true;
                    atHeight = false;
                } else if (_arm.getCurrentHeight() < goalHeight) {
                    _arm.forkliftControl(0.4);
                } else {
                    _arm.forkliftControl(-0.1); //1/10 instead of whole unit because of gravity
                }
            }
        }
        super.stop();
    }

    /**
     *
     * @return
     */
    public double getLiftDistance() {
        return -_arm.getForkliftEncoder().getDistance();
    }

    /**
     *
     */
    public void stop() {
        super.stop();
    }
}
