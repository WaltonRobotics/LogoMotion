package edu.wpi.first.wpilibj.templates;

public class GoToHeight extends AutoFunction
{

    double goalHeight;
    private boolean farFromTarget;
    private boolean atHeight;
    Arm _arm;

    public GoToHeight(double height, Arm _arm) {
        this._arm = _arm;
        goalHeight = height;
        farFromTarget = true;
        atHeight = false;
    }

    public boolean atCorrectHeight() {
        return atHeight;
    }

    public void run() {
        _arm.forkliftEncoder.reset();
        _arm.forkliftEncoder.start();
        while (isRunning()) {
            while (farFromTarget && isRunning()) {
                _arm.currentHeight += getLiftDistance();
                _arm.forkliftEncoder.reset();
                if (Math.abs(_arm.currentHeight - goalHeight) < 2) {
                    farFromTarget = false;
                    atHeight = true;
                } else if (_arm.currentHeight < goalHeight) {
                    _arm.forkliftControl(1.0);
                } else {
                    _arm.forkliftControl(-0.2); //1/10 instead of whole unit because of gravity
                }
            }
            while (!farFromTarget && isRunning()) {
                _arm.currentHeight+= getLiftDistance();
                _arm.forkliftEncoder.reset();
                if (Math.abs(_arm.currentHeight - goalHeight) > 2) {
                    farFromTarget = true;
                    atHeight = false;
                } else if (_arm.currentHeight < goalHeight) {
                    _arm.forkliftControl(0.4);
                } else {
                    _arm.forkliftControl(-0.1); //1/10 instead of whole unit because of gravity
                }
            }
        }
        super.stop();
    }

    public double getLiftDistance()
    {
        return -_arm.forkliftEncoder.getDistance();
    }

    public void stop()
    {
        super.stop();
    }
}
