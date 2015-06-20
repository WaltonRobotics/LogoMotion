package edu.wpi.first.wpilibj.templates;

public class TurnThread extends AutoFunction
{

    private int degrees;
    RobotMain robot;

    public TurnThread(int deg, RobotMain robot) {
        this.robot = robot;
        //reduces degrees to its lowest equivalent measured angle
        degrees = degrees % 360;
        running = false;
    }

    public void run() {
        double startAngle = robot.getSensors().getGyroAngle();
        robot.printToScreen("Angle: " + startAngle);
        //will keep track of whether the turn is over and the robot should stop
        boolean done = false;
        boolean turningLeft = false;
        boolean turningRight = false;
        //if being asked to make more than a half turn right, turn left instead
        if (degrees > 180) {
            robot.getRobotDrive().turn(-0.5);
            turningRight = false;
            turningLeft = true;
            done = false;
        }
        if (degrees <= 180 && degrees != 0) {
            robot.getRobotDrive().turn(0.5);
            turningRight = true;
            turningLeft = false;
            done = false;
        } //if being asked to turn 0 degrees, don't turn and set done to true.
        else {
            turningLeft = false;
            turningRight = false;
            done = true;
        }
        //loop that monitors the robot's angle until it is done turning
        while (!done && running) {
            //if the robot is turning left and is farther left than its goal, set done to true
            if (turningLeft && ((robot.getSensors().getGyroAngle() - startAngle) % 360) <= degrees) {
                done = true;
            }
            if (turningRight && ((robot.getSensors().getGyroAngle() - startAngle) % 360) >= degrees) {
                done = true;
            }
        }
        //at this point, done must be true, so stop turning
        robot.getRobotDrive().stop();
        running = false;
    }

    /**
     * Always returns true.
     * @return true
     */
    public boolean atCorrectHeight() {
        return true;
    }
}
