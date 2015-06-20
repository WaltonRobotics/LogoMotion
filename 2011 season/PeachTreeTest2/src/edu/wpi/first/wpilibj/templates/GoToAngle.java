package edu.wpi.first.wpilibj.templates;

class GoToAngle extends AutoFunction
{

    double goalAngle;
    Arm _arm;

    public GoToAngle(double angle, Arm _arm) {
        this._arm = _arm;
        goalAngle = angle;
    }

    public void run() {
        while (running) {
            _arm.timer.delay(0.05);
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

    public boolean atCorrectHeight() {
        if (Math.abs(_arm.getArmAngle() - goalAngle) < 0.03) {
            return true;
        }
        return false;
    }
}
