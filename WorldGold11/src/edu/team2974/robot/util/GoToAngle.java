package edu.team2974.robot.util;

import edu.team2974.robot.arm.Arm;

/**
 *
 */
public class GoToAngle extends AutoFunction
{

    double goalAngle;
    Arm _arm;

    /**
     *
     * @param angle
     * @param _arm
     */
    public GoToAngle(double angle, Arm _arm) {
        this._arm = _arm;
        goalAngle = angle;
    }

    /**
     *
     */
    public void run() {
        while (isRunning()) {
            _arm.getTimer().delay(0.05);
            if (Math.abs(_arm.getArmAngle() - goalAngle) < 3) {
                if (_arm.getArmAngle() < goalAngle) {
                    _arm.heightenArmSlow();
                } //goes down if the arm is higher than its goal
                else {
                    _arm.lowerArmSlow();
                }
            } //goes up if the arm is lower than its goal
            else {
                if (_arm.getArmAngle() < goalAngle) {
                    _arm.heightenArmFast();
                } //goes down if the arm is higher than its goal
                else if (_arm.getArmAngle() > goalAngle) {
                    _arm.lowerArmFast();
                }
            }
        }
    }

    /**
     *
     * @return
     */
    public boolean atCorrectHeight() {
        if (Math.abs(_arm.getArmAngle() - goalAngle) < 0.03) {
            return true;
        }
        return false;
    }

    /**
     *
     */
    public void stop() {
        super.stop();
    }
}
