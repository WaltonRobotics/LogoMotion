package edu.team2974.robot.control;

import edu.team2974.robot.util.AutoFunction;
import edu.team2974.robot.drive.SpeedSet;
import edu.wpi.first.wpilibj.templates.RobotMain;

/**
 *
 * @author Gil
 */
public class WaltLineTrackOrganized extends AutoFunction
{

    //variables to keep track of which path the robot will follow
    boolean straightPath, leftPath;
    //the robot
    RobotMain robot;
    //contants to represent the 3 sensors
    private final int LEFT = 1, MID = 2, RIGHT = 3;
    //distance from start of line to fork IN INCHES
    final double DISTANCE_TO_FORK = 162.4;
    //distance from beginning of the fork to when it straightens back out IN INCHES
    final double DISTANCE_LEG = 64.0;
    //final stretch to T IN INCHES
    final double FINAL_DISTANCE = 12.0;
    //Distance to end of straight line
    final double STRAIGHT_DISTANCE = 225.9;
    //the time after which the whole thread should stop regardless of its status
    private long stopTime;
    //thread used to turn the robot to follow the fork
    private TurnThread turn;
    //last value of sensors;
    private int last;

    /**
     *
     * @param s
     * @param l
     * @param robot
     */
    public WaltLineTrackOrganized(boolean s, boolean l, RobotMain robot) {
        this.robot = robot;
        straightPath = s;
        leftPath = l; //meaning fork
        //running = false;
        last = 0;
    }

    /**
     *
     */
    public void run() {
        try {
            //calculate time at which to stop the line tracking thread regardless of its status
            stopTime = System.currentTimeMillis() + 12000;
            robot.getMyDog().feed();
            if (straightPath) {
                //follow line until all three sensors are triggered at the T
                while (isRunning() && getOnCount() < 3) {
                    checkTime();
                    lineFollow();
                }
                stop();
            }
//
//            else {
//                //follow line until the fork
//                while (isRunning() && Math.abs(robot.getDriveDistance()) < DISTANCE_TO_FORK) {
//                    checkTime();
//                    lineFollow();
//                }
//                //robot.getRobotDrive().stopDistance(DISTANCE_TO_FORK,3.0);
//
//                robot.getRobotDrive().zero();
//                resetEncoders();
//                robot.getWatchdog().feed();
//                //needs to be 1.5 seconds. 4 for testing purposes
//                double turnSTime = System.currentTimeMillis() + 4000;
//                double turnSpeed = .1;
//                double[] instructions = new double[3];
//                instructions[0] = 0.0;
//                instructions[1] = 0.0;
//                instructions[2] = -turnSpeed;
//                //turn instructions into a speedset, acceleration limit, and then set motor speeds
//                SpeedSet speeds = robot.getRobotDrive().getSpeeds(instructions);
//                //speeds = robot.getRobotDrive().accelerationLimit(speeds);
//                robot.getRobotDrive().setSpeeds(speeds);
//                while (robot.getMiddle().get() && isRunning()) {
//                    robot.getWatchdog().feed();
//                    checkTime();
//                    if(System.currentTimeMillis() >= turnSTime)
//                    {
//                        stop();
//                    }
//                    //fills out the array with speed and turn
//
//                }
//                /*
//                 * old fork code
//                if (isRunning()) {
//                robot.getRobotDrive().stop();
//                }
//                //pause 1 second
//                if (isRunning()) {
//                try {
//                sleep(200);
//                } catch (InterruptedException ie) {
//                }
//                }
//
//                //turn left if going left, right if going right
//                turn(leftPath);
//
//                //pause .1 second
//                if (isRunning()) {
//                try {
//                sleep(100);
//                } catch (InterruptedException ie) {
//                }
//                resetEncoders();
//                robot.getWatchdog().feed();
//                }
//                 *
//                 */
//                //follow the second leg of the fork, until the second turn
//                //last = MID;
//
//                while (isRunning() && Math.abs(robot.getDriveDistance()) < DISTANCE_LEG + 5) {
//                    lineFollow();
//                    checkTime();
//                }
//                /*
//                if (isRunning()) {
//                    //robot.getRobotDrive().stopDistance(DISTANCE_LEG,2.0);
//                    robot.getRobotDrive().stop();
//                    //pause 1 second
//                    try {
//                        sleep(200);
//                    } catch (InterruptedException ie) {
//                    }
//                    turn(!leftPath);
//                }
//                 *
//                 */
//                robot.getWatchdog().feed();
//                //needs to be 1.5 for matches.
//                turnSTime = System.currentTimeMillis() + 4000;
//                instructions[0] = 0.0;
//                instructions[1] = 0.0;
//                instructions[2] = turnSpeed;
//                //turn instructions into a speedset, acceleration limit, and then set motor speeds
//                speeds = robot.getRobotDrive().getSpeeds(instructions);
//                //speeds = robot.getRobotDrive().accelerationLimit(speeds);
//                robot.getRobotDrive().setSpeeds(speeds);
//                while (isRunning() && robot.getMiddle().get()) {
//                    robot.getWatchdog().feed();
//                    checkTime();
//                    if(System.currentTimeMillis() >= turnSTime)
//                    {
//                        stop();
//                    }
//                    //fills out the array with speed and turn
//                }
//                //turn the opposite direction of the first time (to straighten the robot)
//                //
//                /*
//                if (isRunning()) {
//                    try {
//                        sleep(100);
//                    } catch (InterruptedException ie) {
//                    }
//                    resetEncoders();
//                }
//                 *
//                 */
//                robot.getWatchdog().feed();
//                //follow the final segment of the line until the T
//                //note: worked without the condition onCount < 3, but that condition should probably be added in
//                while (isRunning() && Math.abs(robot.getDriveDistance()) < FINAL_DISTANCE) {
//                    checkTime();
//                    lineFollow();
//                }
//                robot.getRobotDrive().stop();
//
//            }
        } catch (Exception e) {
        }
        stop();
    }

    //stops the thread if it has been going on for more than stopTime seconds
    /**
     *
     */
    public void checkTime() {
        if (System.currentTimeMillis() >= stopTime) {
            robot.printToScreen("WLT TIMED OUT!!");
            stop();
        }
        robot.getWatchdog().feed();
    }

    //turns 30 degrees in the specified direction
    /**
     *
     * @param left
     */
    public void turn(boolean left) {
        //create turn thread with correct angle measure
        if (left) {
            turn = new TurnThread(-30, robot);
        } else {
            turn = new TurnThread(30, robot);
        }
        //start turn
        turn.begin();
        //time after which to stop turning regardless of sensor input
        long turnStopTime = System.currentTimeMillis() + 1500;
        //start turning: will stop at 30 degrees, if the turn goes on for more than 1.5 seconds, or
        //if the whole line tracking thread has timed out (30 seconds).
        while (turn.isRunning()) {
            checkTime();
            if (System.currentTimeMillis() >= turnStopTime) {
                robot.printToScreen("second Turn TIMED OUT!!");
                turn.stop();
                break;
            }
            robot.getMyDog().feed();
        }
        robot.getRobotDrive().stop();
    }

    /**
     * Always returns true.
     * @return true
     */
    public boolean atCorrectHeight() {
        return true;
    }

    //stops all thread action
    /**
     *
     */
    public void stop() {
        robot.printToScreen("WLT stop called");
        //stop turn thread
        if (turn != null) {
            turn.stop();
            turn = null;
        }
        robot.getRobotDrive().zero();
        super.stop();
        robot.printToScreen("WLT called super.stop, stop exiting now");
    }

    //checks which sensor is triggered and sets motor speeds accordingly
    private void lineFollow() {
        //variables to keep track of the way the robot should move at the end of this method
        double speed = 0;
        double turn = 0;
        //the last sensor triggered
        //checks sensors and records which one is triggered in variable "last"
        //all values are inverted because the sensors give false when on the line
        if (!robot.getMiddle().get()) {
            last = MID;
        }
        if (!robot.getLeft().get()) {
            last = LEFT;
        }
        if (!robot.getRight().get()) {
            last = RIGHT;
        }
        //sets speeds based on sensor triggered
        if (last == MID) {
            //go straight
            speed = 0.33;
            turn = 0.0;
        } else if (last == RIGHT) {
            //turn right
            speed = 0.33;
            turn = 0.1;
            //turn=.7;
        } else if (last == LEFT) {
            //turn left
            speed = 0.33;
            turn = -0.1;
            //turn=-.7;
        }
        //a variable used to change turn and speed into a SpeedSet for the drive train
        double[] instructions = new double[3];
        //fills out the array with speed and turn
        instructions[0] = -speed;
        instructions[1] = 0.0;
        instructions[2] = -turn;
        //turn instructions into a speedset, acceleration limit, and then set motor speeds
        SpeedSet speeds = robot.getRobotDrive().getSpeeds(instructions);
        speeds = robot.getRobotDrive().accelerationLimit(speeds);
        robot.getRobotDrive().setSpeeds(speeds);
    }

    //returns the number of sensors on the line
    /**
     *
     * @return
     */
    public int getOnCount() {
        //start with a count of 0
        int onCount = 0;
        //add one for each sensor triggered
        if (!robot.getMiddle().get()) {
            onCount++;
        }
        if (!robot.getLeft().get()) {
            onCount++;
        }
        if (!robot.getRight().get()) {
            onCount++;
        }
        return onCount;
    }

    //resets the encoder values to 0 and starts them counting
    /**
     *
     */
    public void resetEncoders() {
        robot.getLeftEncoder().stop();
        robot.getLeftEncoder().reset();
        robot.getLeftEncoder().start();
    }
}
