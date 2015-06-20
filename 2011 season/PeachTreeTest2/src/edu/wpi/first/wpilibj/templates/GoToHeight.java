package edu.wpi.first.wpilibj.templates;

public class GoToHeight extends AutoFunction
{

    double goalHeight;
    public boolean moveDirectionUp;
    private boolean farFromTarget;
    private boolean atHeight;
    Arm _arm;

    public GoToHeight(int i, Arm _arm) {
        this._arm = _arm;
        if (0 <= i && i <= 8) {
            goalHeight = _arm.heights[i];
        } else {
            goalHeight = _arm.currentHeight;
        }
        moveDirectionUp = false;
        farFromTarget = true;
        atHeight = false;
    }

    public boolean atCorrectHeight() {
        return atHeight;
    }

    public void run() {
        _arm.forkliftEncoder.reset();
        _arm.forkliftEncoder.start();
        while (running) {
            while (farFromTarget && running) {
                if (moveDirectionUp) {
                    _arm.currentHeight += _arm.forkliftEncoder.getDistance();
                } else {
                    _arm.currentHeight -= _arm.forkliftEncoder.getDistance();
                }
                _arm.forkliftEncoder.reset();
                if (Math.abs(_arm.currentHeight - goalHeight) < 2) {
                    farFromTarget = false;
                    atHeight = true;
                } else if (_arm.currentHeight < goalHeight) {
                    moveDirectionUp = true;
                    _arm.forkliftControl(1.0);
                } else {
                    moveDirectionUp = false;
                    _arm.forkliftControl(-0.2); //1/10 instead of whole unit because of gravity
                }
            }
            while (!farFromTarget && running) {
                if (moveDirectionUp) {
                    _arm.currentHeight += _arm.forkliftEncoder.getDistance();
                } else {
                    _arm.currentHeight -= _arm.forkliftEncoder.getDistance();
                }
                _arm.forkliftEncoder.reset();
                if (Math.abs(_arm.currentHeight - goalHeight) > 2) {
                    farFromTarget = true;
                    atHeight = false;
                } else if (_arm.currentHeight < goalHeight) {
                    moveDirectionUp = true;
                    _arm.forkliftControl(0.4);
                } else {
                    moveDirectionUp = false;
                    _arm.forkliftControl(-0.1); //1/10 instead of whole unit because of gravity
                }
            }
        }
        running = false;
    }
}
