package edu.wpi.first.wpilibj.templates;

public class TurnThread extends AutoFunction
{

    private int degrees;
    RobotMain robot;

    public TurnThread(int deg, RobotMain robot) {
        this.robot = robot;
        //reduces degrees to its lowest equivalent measured angle
        degrees = (deg + 360) % 360;

    }

    public void run() {
        double startAngle = robot.getGyroAngle();
        robot.printToScreen(3,"Start Angle: " + startAngle);
        //will keep track of whether the turn is over and the robot should stop
        boolean turningLeft = false;
        boolean turningRight = false;
        //if being asked to make more than a half turn right, turn left instead
        if (degrees > 180) {
            //was .5

            robot.getRobotDrive().turn(.8);
            turningRight = false;
            turningLeft = true;
            degrees = 360 - degrees;
        }
        else if(degrees <= 180 && degrees != 0) {
            robot.printToScreen("Math works");
            robot.getRobotDrive().turn(-.8);
            turningRight = true;
            turningLeft = false;
        } //if being asked to turn 0 degrees, don't turn and set done to true.
        else {
            turningLeft = false;
            turningRight = false;
            stop();
        }
        //loop that monitors the robot's angle until it is done turning
        int counter = 0;

        while (isRunning()) {

            if(turningRight){
                robot.getRobotDrive().turn(-.8);
            }
            if(turningLeft){
                robot.getRobotDrive().turn(.8);
            }
            //if the robot is turning left and is farther left than its goal, set done to true
            if (turningLeft && (startAngle - robot.getGyroAngle()) >= degrees) {
                stop();
            }
            if (turningRight && (robot.getGyroAngle() - startAngle) >= degrees) {
                stop();
                robot.printToScreen(6,"thread stopped itself");
            }

            robot.printToScreen(4,"Gyro Angle: " + robot.getGyroAngle());
            counter++;
        }
        robot.printToScreen("count "+counter);
        //at this point, done must be true, so stop turning
        robot.getRobotDrive().stop();
        //stop();
    }

    /**
     * Always returns true.
     * @return true
     */
    public boolean atCorrectHeight() {
        return true;
    }

    public void stop()
    {
        super.stop();
    }
}
