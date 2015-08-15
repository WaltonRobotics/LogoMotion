package edu.team2974.robot.drive;

import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.templates.RobotMain;

/**
 * This class exists to express all of the similarities between the two arcade
 * drives and the tank drive.  It contains the code for processing the
 * motorSpeeds after they are calculated from joystick inputs, then using the
 * speeds to set the controllers. The only parts that are overwritten in
 * subclasses are the constructors and the setSpeedPair() methods that convert
 * the joystick input into raw motor speeds.
 *
 * @author Gil
 */
public class TwoMotorDrive extends DriveTrain
{

    SpeedController leftController; //the left jaguar
    SpeedController rightController; //the right jaguar
    int rightOrientation;
    int leftOrientation;
    RobotMain robot;

    //constructs a WaltTwoMotor drive with appropriate controllers and demo mode
    /**
     *
     * @param channelLeft
     * @param channelRight
     * @param demo
     * @param robot
     */
    public TwoMotorDrive(int channelLeft, int channelRight, boolean demo,
            RobotMain robot) {
        leftController = new Jaguar(channelLeft);
        rightController = new Jaguar(channelRight);
        leftOrientation = 1;
        rightOrientation = 1;
        speedsPrevious = new SpeedPair(0, 0);
        demoMode = demo;
        this.robot = robot;
    }

    //physically stops the robot
    /**
     *
     */
    public void stop() {
        //was divide by 4
        double stopLeftSpeed = -1 * leftController.get() / 2;
        double stopRightSpeed = -1 * rightController.get() / 2;
        SpeedPair stop = new SpeedPair(stopLeftSpeed, stopRightSpeed);
        setSpeeds(stop);
        Timer.delay(.2);
        zero();
    }

    /**
     *
     * @param target
     * @param threshold
     */
    public void stopDistance(double target, double threshold) {
        //new stop method for autonomous where it moves in an inverted direction
        //if the robot has moved too far past its target. The threshold makes it
        // so it doesn't
        double speedLeft = -1 * leftController.get() / 2;
        double speedRight = -1 * rightController.get() / 2;
        zero();
        Timer.delay(.2);
        while (Math.abs(robot.getDriveDistance()) > target + threshold) {
            SpeedPair driveBack = new SpeedPair(speedLeft, speedRight);
            setSpeeds(driveBack);
        }
        zero();
    }

    //gives jags 0
    /**
     *
     */
    public void zero() {
        SpeedPair zero = new SpeedPair(0, 0);
        setSpeeds(zero);
    }

    //turns the robot at the given speed
    //positive is right, negative is left
    /**
     *
     * @param speed
     */
    public void turn(double speed) {
        SpeedPair turning = new SpeedPair(speed, -speed);
        turning.reduce();
        setSpeeds(turning);
    }

    //goes forward at a given speed- positive is forward, negative is backward.
    //Should be between -1 and 1, will interpret all others as full speed.
    /**
     *
     * @param speed
     */
    public void goForward(double speed) {
        SpeedPair forward = new SpeedPair(speed, speed);
        forward.reduce();
        setSpeeds(forward);
    }

    //generates a raw SpeedPair based on movement instructions
    /**
     *
     * @param instructions
     * @return
     */
    public SpeedSet getSpeeds(double[] instructions) {
        double leftSpeed;
        double rightSpeed;
        double forward = instructions[0];
        double turn = instructions[2];

        leftSpeed = forward + turn;
        rightSpeed = forward - turn;

        SpeedPair speeds = new SpeedPair(leftSpeed, rightSpeed);
        return speeds;
    }

    //chain limits the speeds appropriately based on demoMode
    /**
     *
     * @param speeds
     * @return
     */
    public SpeedSet accelerationLimit(SpeedSet speeds) {
        if (demoMode) {
            speeds.limitIncreaseTo(speedsPrevious);
            return speeds;
        } else {
            speeds.limitTo(speedsPrevious);
            return speeds;
        }
    }

    //divides the pair by 2 if we are at a demo or overDrive is not engaged
    /**
     *
     * @param speeds
     * @param overDrive
     * @param underDrive
     * @return
     */
    public SpeedSet handleOverDrive(SpeedSet speeds, boolean overDrive,
            boolean underDrive) {
        //still goes only 3/4 speed with overDrive at a demo
        if (demoMode && overDrive) {
            speeds.divideBy(4 / 3);
            return speeds;
        }

        //goes full speed if overDrive and not at a demo
        if (!demoMode && overDrive) {
            return speeds;
        } else if (!demoMode && underDrive) {
            speeds.divideBy(4);
            return speeds;
        } //if overDrive is not engaged, goes at half speed regardless of demoMode
        else {
            speeds.divideBy(2);
            return speeds;
        }
    }

    //sets the speeds of the motors and updates leftPrev and rightPrev to reflect
    //the new speeds
    /**
     *
     * @param spdsNew
     */
    public void setSpeeds(SpeedSet spdsNew) {
        if (leftController == null || rightController == null) {
            throw new NullPointerException("Null motor provided");
        }

        //casts into a pair
        SpeedPair speedsNew = (SpeedPair) spdsNew;
        leftController.set(speedsNew.left * leftOrientation);
        rightController.set(speedsNew.right * rightOrientation);
        speedsPrevious = speedsNew;
    }

    /**
     *
     * @param right
     */
    public void setInvertedSide(boolean right) {
        if (right) {
            rightOrientation = -1;
        } else {
            leftOrientation = -1;
        }
    }
}
