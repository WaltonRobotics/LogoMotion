/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

/**
 *
 * @author Gil
 */
public class WaltLineTrackFor6WheelBot extends AutoFunction {

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
    //distance that robot needs to move at the end- it will not actually be going straight, so it is not 12"
    final double FINAL_DISTANCE = 8.0;
    //Distance to end of straight line
    final double STRAIGHT_DISTANCE = 225.9;
    //the distance from the light sensors to the turning center of the robot- needs to be measured
    final double DISTANCE_HALF_ROBOT = 16.0;
    //the time after which the whole thread should stop regardless of its status
    private long stopTime;
    //thread used to turn the robot to follow the fork
    private TurnThread turn;

    public WaltLineTrackFor6WheelBot(boolean s, boolean l, RobotMain robot) {
        this.robot = robot;
        straightPath = s;
        leftPath = l; //meaning fork
        //running = false;
    }

    public void run() {
        //calculate time at which to stop the line tracking thread regardless of its status
        stopTime = System.currentTimeMillis() + 30000;
        robot.myDog.feed();
        if (straightPath) {
            //follow line until all three sensors are triggered at the T
            while (getOnCount() < 3) {
                checkTime();
                lineFollow();
            }
        } else {
            //follow line until the center of the robot is on the fork
            while (isRunning() && Math.abs(robot.getDriveDistance()) < DISTANCE_TO_FORK + DISTANCE_HALF_ROBOT) {
                checkTime();
                lineFollow();
            }
            robot.getRobotDrive().stop();
            //pause 1 second
            try {
                sleep(1000);
            } catch (InterruptedException ie) {
            }
            //turn left if going left, right if going right
            if(leftPath){
                robot.getRobotDrive().turn(.8);
            }
            else {
                robot.getRobotDrive().turn(-.8);
            }
            //wait for middle sensor to hit line
            while(robot.getMiddle().get()){
                checkTime();
            }
            robot.getRobotDrive().stop();
            //pause 1 second
            try {
                sleep(1000);
            } catch (InterruptedException ie) {
            }
            resetEncoders();
            //follow the second leg of the fork, until the second turn
            while (isRunning() && Math.abs(robot.getDriveDistance()) < DISTANCE_LEG) {
                lineFollow();
                checkTime();
            }
            robot.getRobotDrive().stop();
            //pause 1 second
            try {
                sleep(1000);
            } catch (InterruptedException ie) {
            }
            //turn the opposite direction 15 degrees to try to be aiming straight towards the pegs
            turn(!leftPath);
            resetEncoders();
            //follow the final segment of the line until the T
            //note: worked without the condition onCount < 3, but that condition should probably be added in
            while (isRunning() && Math.abs(robot.getDriveDistance()) < FINAL_DISTANCE) {
                checkTime();
                lineFollow();
            }
            robot.getRobotDrive().stop();
        }
    }

    //stops the thread if it has been going on for more than stopTime seconds
    public void checkTime() {
        if (System.currentTimeMillis() >= stopTime) {
            robot.printToScreen("WLT TIMED OUT!!");
            stop();
        }
    }

    //turns 30 degrees in the specified direction
    public void turn(boolean left) {
        //create turn thread with correct angle measure- calculated based on robot size and distance
        //to peg, but needs to be measured with robot and tested
        if (left) {
            turn = new TurnThread(-25, robot);
        } else {
            turn = new TurnThread(25, robot);
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
            robot.myDog.feed();
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
    public void stop() {
        robot.printToScreen("WLT stop called");
        //stop turn thread
        if (turn != null) {
            turn.stop();
            turn = null;
        }
        super.stop();
        robot.printToScreen("WLT called super.stop, stop exiting now");
    }

    //checks which sensor is triggered and sets motor speeds accordingly
    private void lineFollow() {
        //variables to keep track of the way the robot should move at the end of this method
        double speed = .33;
        double turn = 0;
        //the last sensor triggered
        int last = 0;
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
    public void resetEncoders() {
        robot.getLeftEncoder().stop();
        robot.getLeftEncoder().reset();
        robot.getLeftEncoder().start();
    }
}
