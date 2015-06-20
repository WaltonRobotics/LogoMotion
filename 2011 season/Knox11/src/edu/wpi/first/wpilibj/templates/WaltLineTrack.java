package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Timer;

class WaltLineTrack extends AutoFunction {

    boolean straightPath;
    boolean leftPath;
    RobotMain robot;
    private TurnThread turnFork;
    private TurnThread secondTurn;

    WaltLineTrack(boolean s, boolean l, RobotMain robot) {
        this.robot = robot;
        straightPath = s;
        leftPath = l; //meaning fork
        //running = false;

        if (leftPath) {
            turnFork = new TurnThread(-30, robot);
            secondTurn = new TurnThread(30, robot);
        } else {
            turnFork = new TurnThread(30, robot);
            secondTurn = new TurnThread(-30, robot);
        }
    }

    public void run() {
        //intializes the last value which is used to take the last known sensor that was on the line
        int last = 0;
        //the values that will be passed to last if the corresponding snesor is triggered
        int leftValue = 1;
        int midValue = 2;
        int rightValue = 3;
        //counts how many sensors are triggered at one time
        int allOnCount = 0;
        //stops the encoders if they were activated before hand
        robot.getLeftEncoder().stop();
        //resets the distance traveled
        robot.getLeftEncoder().reset();
        //starts them
        robot.getLeftEncoder().start();
        //NOTE!!! Fix before competition field
        //actual values: 162.4,64.0,12.0,226.9/2
        //testing values: /2
        //distance from start of line to fork IN INCHES
        final double DISTANCE_TO_FORK = 162.4;
        //distance from beginning of the fork to when it straightens back out IN INCHES
        final double DISTANCE_LEG = 64.0;
        //final stretch to T IN INCHES
        final double FINAL_DISTANCE = 12.0;
        //Distance to end of straight line
        final double STRAIGHT_DISTANCE = 225.9;
        //forward speed IN JAG INPUT (-1 to 1)
        double speed = 0.0;
        //turn speed IN JAG INPUT (-1 to 1)
        double turn = 0.0;
        //speed values sent to robotDrive
        //Param: Speed amt forward, speed sideways (not used), speed Turn
        double[] instructions = {0.0, 0.0, 0.0};
        //if line tracking has finished or not
        //boolean done = false;
        robot.printToScreen(6, "Line Tracking now");
        SpeedSet speeds;
        //booleans that track whether a the different distances have been reached yet
        boolean forkNotReached = true;
        boolean secondTurnNotReached = true;
        boolean finalNotReached = true;
        //sets turns based on which path in the fork the robot is supposed to travel
        //start of loop that handles line
        long stopTime = System.currentTimeMillis() + 12000;
        while (isRunning()) {
            robot.myDog.feed();
            if (System.currentTimeMillis() >= stopTime) {
                robot.printToScreen("WLT TIMED OUT!!");
                stop();
                break;
            }
            //checks sensors to see if they are on lines. Will need to get rid of "!"
            // at competition if tape is darker than floor. allOnCount is added to if a sensor is on
            // and last is assigned the value of the last sensor triggered.
            if (!robot.getMiddle().get()) {
                last = midValue;
                allOnCount++;
            }
            if (!robot.getLeft().get()) {
                last = leftValue;
                allOnCount++;
            }
            if (!robot.getRight().get()) {
                last = rightValue;
                allOnCount++;
            }
            if (last == midValue) {
                speed = 0.33;
                turn = 0.0;
            } else if (last == rightValue) {
                speed = 0.33;
                turn = 0.1;
                //turn=.7;
            } else if (last == leftValue) {
                speed = 0.33;
                turn = -0.1;
                //turn=-.7;
            } else {
            }
            //Determines which path the robot is following
            //||STRAIGHT_DISTANCE<=robot.getDriveDistance()
            if (straightPath) {
                if (allOnCount == 3
                        || STRAIGHT_DISTANCE <= robot.getDriveDistance()) {
                    robot.getRobotDrive().stop();
                    robot.printToScreen("Robot stopping");
                    stop();
                    //running = false;
                    //done = true;
                    return;
                }
            } else {
                //if the fork hasn't been reached yet, robot does a check whether
                //the encoders have moved enough distance to turn
                if (forkNotReached) {
                    //checks whether the distance traveled by the robot is bigger
                    // or equal to the distance needed to reach the fork
                    if (DISTANCE_TO_FORK <= robot.getDriveDistance()) {
                        robot.printToScreen("End of fork, distance: "
                                + robot.getDriveDistance());
                        //if the fork is reached turns the boolean to false
                        forkNotReached = false;
                        //stops and waits 1 second- need to use sleep rather than delay so that
                        //other threads can keep going, like feeding watchdog in teleop continuous
                        robot.getRobotDrive().stop();
                        try {
                            sleep(1000);
                        } catch (InterruptedException ie) {
                        } 
                        //delayed while the turn is executed
                        long turnStopTime = System.currentTimeMillis() + 1500;

                        turnFork.begin();
                        while (turnFork.isRunning()) {
                            if (System.currentTimeMillis() >= turnStopTime) {
                                robot.printToScreen("second Turn TIMED OUT!!");
                                turnFork.stop();
                                break;
                            }
                            Timer.delay(0.1);
                            robot.myDog.feed();
                        }
                        //stops and waits 1 second- need to use sleep rather than delay so that
                        //other threads can keep going, like feeding watchdog in teleop continuous
                        robot.getRobotDrive().stop();
                        try {
                            sleep(1000);
                        } catch (InterruptedException ie) {
                        } //robot can begin measuring the distance to the next turn
                        robot.getLeftEncoder().reset();
                        //Robot forced to move straight after turn regardless
                        //of what sensor was triggered before the turn
                        speed = 0.25;
                        turn = 0.0;
                    }
                } //checks whether the distance traveled by the robot is bigger
                // or equal to the distance needed to reach the second turn
                else if (secondTurnNotReached) {
                    //checks whether the distance traveled by the robot is bigger
                    // or equal to the distance needed to reach the second turn
                    if (DISTANCE_LEG <= robot.getDriveDistance()) {
                        //if the next turn is reached turns the boolean to false
                        secondTurnNotReached = false;
                        //the last turn is executed and line track code is
                        //delayed while the turn is executed
                        //stops and waits 1 second- need to use sleep rather than delay so that
                        //other threads can keep going, like feeding watchdog in teleop continuous
                        robot.getRobotDrive().stop();
                        try {
                            sleep(1000);
                        } catch (InterruptedException ie) {
                        }
                        long turnStopTime = System.currentTimeMillis() + 1500;

                        secondTurn.begin();
                        while (secondTurn.isRunning()) {
                            if (System.currentTimeMillis() >= turnStopTime) {
                                robot.printToScreen("second turn TIMED OUT!!");
                                secondTurn.stop();
                                break;
                            }
                        }
                        secondTurn.begin();
                        //the turn at the fork is executed and line track code is
                        //delayed while the turn is executed
                        while (secondTurn.isRunning()) {
                            Timer.delay(0.1);
                        }
                        //stops and waits 1 second- need to use sleep rather than delay so that
                        //other threads can keep going, like feeding watchdog in teleop continuous
                        robot.getRobotDrive().stop();
                        try {
                            sleep(1000);
                        } catch (InterruptedException ie) {
                        } //robot can begin measuring the distance to the next turn
                        robot.getLeftEncoder().reset();
                        //Robot forced to move straight after turn regardless
                        //of what sensor was triggered before the turn
                        speed = 0.25;
                        turn = 0.0;
                    }
                } //checks whther the T has been reached
                else if (finalNotReached) {
                    if (allOnCount == 3 || FINAL_DISTANCE <= robot.getDriveDistance()) {
                        robot.getRobotDrive().stop();
                        robot.printToScreen("Robot stopping");
                        stop();
                        //running = false;
                        //done = true;
                        return;
                    }
                }
            }
            //robot.printToScreen(4, "speed: " + speed + " Turn:" + turn);
            //robot.printToScreen(5, "Encoder distance: "+robot.getDriveDistance());

            instructions[0] = -speed;
            instructions[1] = 0.0;
            instructions[2] = -turn;
            speeds = robot.getRobotDrive().getSpeeds(instructions);
            speeds = robot.getRobotDrive().accelerationLimit(speeds);
            robot.getRobotDrive().setSpeeds(speeds);
            allOnCount = 0;
        }
    }

    /**
     * Always returns true.
     * @return true
     */
    public boolean atCorrectHeight() {
        return true;
    }

    public void stop() {
        robot.printToScreen("WLT stop called");
        if (turnFork != null) {
            turnFork.stop();
            turnFork = null;
        }

        if (secondTurn != null) {
            secondTurn.stop();
            secondTurn = null;
        }
        super.stop();
        robot.printToScreen("WLT called super.stop, stop exiting now");
    }
}
