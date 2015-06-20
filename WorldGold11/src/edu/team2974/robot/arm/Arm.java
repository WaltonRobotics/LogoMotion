package edu.team2974.robot.arm;

import edu.team2974.robot.util.GoToAngle;
import edu.team2974.robot.util.GoToHeight;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;

/**
 *
 * @author Paul
 */
public class Arm
{

    /**
     *
     */
    private SpeedController forkliftJag;
    /**
     * 
     */
    private SpeedController armJag;
    /**
     *
     */
    private Timer timer;
    /**
     *
     */
    private double currentHeight;
    /**
     *
     */
    private Solenoid wrist;
    /**
     *
     */
    private Solenoid claw;
    /**
     *
     */
    private boolean toggleSwitchClaw;
    /**
     *
     */
    private boolean toggleSwitchWrist;
    /**
     *
     */
    private Encoder forkliftEncoder;
    /**
     *
     */
    private double[] heights;
    /**
     * 
     */
    private double oscillateAngle;
    /**
     * 
     */
    private boolean oscillating;
    /**
     *
     */
    private Gyro gyro;
    /**
     * 
     */
    private double gyroOffset;
    /**
     * 
     */
    private double armSpeed;
    /**
     * 
     */
    private final double GYRO_ANGLE;
    /**
     * 
     */
    private static final double FLIP_HEIGHT = 130;
    /**
     * 
     */
    private double lastGyroCorrect;
    /**
     * 
     */
    private boolean flippedDown;
    /**
     * 
     */
    private double lastOscillateTime;

    /**
     * Class Constructor specifying which channel the Jaguar Motor controller
     *      that controls the arm is plugged into.
     * @param forkliftJagChannel an integer that represents which channel to call the
     *      Jaguar motor controller that controls the forklift
     * @param armJagChannel an integer that represents which channel to call the
     *      Jaguar motor controller that controls the arm
     * @param wristChannel
     * @param clawChannel
     * @param gyroChannel
     */
    public Arm(int forkliftJagChannel, int armJagChannel, int wristChannel,
            int clawChannel, int gyroChannel) {
        flippedDown = false;

        timer = new Timer();
        timer.reset();
        timer.start();
        lastGyroCorrect = 0;


        forkliftJag = new Jaguar(forkliftJagChannel);
        armJag = new Jaguar(armJagChannel);

        wrist = new Solenoid(wristChannel);
        claw = new Solenoid(clawChannel);

        currentHeight = 16;
        heights = new double[8];
        heights[0] = 0.0;
        //the start position of the encoder is 16 inches off the ground
        heights[1] = 16.0;
        heights[2] = 24.0;
        heights[3] = 54.0;
        heights[4] = 62.0;
        heights[5] = 92.0;
        heights[6] = 100.0;
        heights[7] = 27.0;

        forkliftEncoder = new Encoder(9, 10);
        forkliftEncoder.start();
        forkliftEncoder.setDistancePerPulse(5 * 3.14 / 360);

        oscillating = false;
        gyro = new Gyro(gyroChannel);
        gyro.reset();
        GYRO_ANGLE = gyro.getAngle();
        gyroOffset = 180;

        armSpeed = 0;
        lastOscillateTime = 0;
    }

    /**
     * Tells the forklift to go up at full speed
     * @param pressedDown a boolean that represents whether the button that controls
     *      the forklift going up is pressed
     */
    public void forkliftUp(boolean pressedDown) {
        if (pressedDown) {
            forkliftJag.set(1);
        } else {
            forkliftJag.set(0);
        }
    }

    //should be called repeatedly by robot main to correct for predictable drift- not a complete substitute for manual reset
    /**
     *
     */
    public void gyroCorrect() {
        if (getTimer().get() - lastGyroCorrect > 6) {
            gyroOffset += 1;
            lastGyroCorrect = getTimer().get();
        }
    }
    //returns the interpreted angle of the arm

    /**
     *
     * @return
     */
    public double getArmAngle() {
        return -gyro.getAngle() + gyroOffset;
    }

    //this method is called when the arm is vertical to reset the gyro (corrects for drift)
    /**
     *
     */
    public void resetArmAngle() {
        gyro.reset();
    }

    /**
     * Tells the forklift to go down at full speed
     * @param pressedDown a boolean that represents whether the button that controls
     *      the forklift going down is pressed
     */
    public void forkliftDown(boolean pressedDown) {
        if (pressedDown) {
            forkliftJag.set(-.5);
        } else {
            forkliftJag.set(0);
        }
    }

    /**
     * Tells the arm to go up at full speed
     * @param speedToGo
     */
    public void armControl(double speedToGo) {
        getArmJag().set(speedToGo / 2.5);
        //START GILS CHANGES- stops oscillation when driver controls arm
        oscillating = false;
        //END GILS CHANGES
    }

    /**
     *
     * @param speedToGo
     */
    public void armControlRaw(double speedToGo) {
        getArmJag().set(speedToGo);
    }

    /**
     *
     * @param speedToGo
     */
    public void forkliftControl(double speedToGo) {
        forkliftJag.set(speedToGo);
    }

    /**
     *
     * @param angle
     */
    public void goToAngle(double angle) {
        oscillating = false;
    }

    /**
     * Turns solenoid on.
     * More will be added once I learn what a solenoid is!
     * @param pushedDown
     */
    public void grabTube(boolean pushedDown) {
        claw.set(pushedDown);
    }

    /**
     *
     * @param pushedDown
     */
    public void flipWrist(boolean pushedDown) {
        wrist.set(pushedDown);
    }

    /**
     * Turns solenoid off.
     * More will be added once I learn what a solenoid is!
     */
    public void releaseTube() {
        claw.set(false);
    }

    /**
     * Increases the adjuster speed so that we have a constant to... DEFY GRAVITY
     */
//    public void increaseAdjustment() {
//        adjusterSpeed = adjusterSpeed + 0.00001;
//    }
    /**
     * Decreases the adjuster speed so that we have a constant to... DEFY GRAVITY
     */
//    public void decreaseAdjustment() {
//        adjusterSpeed = adjusterSpeed - 0.00001;
//    }
    /**
     *
     */
//    public double getAdjusterSpeed() {
//        return adjusterSpeed;
//    }
    public void clawSwitcher() {
        toggleSwitchClaw = !toggleSwitchClaw;
    }

    /**
     *
     */
    public void wristSwitcher() {
        toggleSwitchWrist = !toggleSwitchWrist;
    }

    /**
     *
     */
    public void wristM() {
        wrist.set(toggleSwitchWrist);
        wristSwitcher();
    }

    /**
     *
     */
    public void wristUp() {
        wrist.set(false);
        flippedDown = false;
    }

    /**
     *
     */
    public void wristDown() {
        wrist.set(true);
        flippedDown = true;
    }

    /**
     *
     */
    public void clawM() {
        claw.set(toggleSwitchClaw);
        clawSwitcher();
    }

    /**
     *
     */
    public void wristFlipCheck() {
        if (getArmAngle() > FLIP_HEIGHT && !flippedDown && getArmAngle() < FLIP_HEIGHT + 3) {
            wristDown();
        }
        if (getArmAngle() < FLIP_HEIGHT && flippedDown && getArmAngle() > FLIP_HEIGHT - 3) {
            wristUp();
        }
    }

    /**
     *
     * @param height
     * @return
     */
    public GoToHeight getNewGoToHeightInstance(double height) {
        return new GoToHeight(height, this);
    }

    /**
     *
     * @param degree
     * @return
     */
    public GoToAngle getNewGoToAngleInstance(double degree) {
        return new GoToAngle(degree, this);
    }

    /**
     *
     * @param oscillating
     */
    public synchronized void setOscillating(boolean oscillating) {
        this.oscillating = oscillating;
    }

    /**
     *
     * @return
     */
    public boolean isOscillating() {
        return oscillating;
    }

    //START GILS CHANGES
    //deleted a line in this method that set oscillating = false
    //i believe that was put there when this was used to manually control the arm; now that
    //it is used to control it autonomously, that could have been causing the errors we saw
    /**
     *
     */
    public void heightenArmFast() {
        getArmJag().set(-.7);
        armSpeed = -1;
    }

    //deleted a line in this method that set oscillating = false
    //i believe that was put there when this was used to manually control the arm; now that
    //it is used to control it autonomously, that could have been causing the errors we saw
    /**
     *
     */
    public void lowerArmFast() {
        getArmJag().set(0.4);
        armSpeed = .3;
    }

    /**
     *
     */
    public void heightenArmSlow() {
        getArmJag().set(-.3);
        armSpeed = -1;
    }

    //deleted a line in this method that set oscillating = false
    //i believe that was put there when this was used to manually control the arm; now that
    //it is used to control it autonomously, that could have been causing the errors we saw
    /**
     *
     */
    public void lowerArmSlow() {
        getArmJag().set(0.2);
        armSpeed = .3;
    }

    /**
     *
     */
    public void armOscillate() {

        //if not already oscillating, get a new goal angle and oscillate at that angle'
        if (!oscillating) {
            setOscillating(true);
            //oscillating = true;
            oscillateAngle = getArmAngle();
            lastOscillateTime = getTimer().get();
        } //if already oscillating, change direction based on height relative to goal
        else {
            //get current time
            double thisOscillateTime = getTimer().get();

            //change: only oscillate ten times per second (the huge constant is
            // since timer.get() is in microseconds
            //hopefully this will help not kill the jags
            if (thisOscillateTime - lastOscillateTime > 100000) {

                //goes up if the arm is lower than its goal
                if (getArmAngle() < oscillateAngle) {
                    heightenArmFast();
                } //goes down if the arm is higher than its goal
                else if (getArmAngle() > oscillateAngle) {
                    lowerArmFast();
                }
            }

            //saves the time for comparison next time this method is called
            lastOscillateTime = thisOscillateTime;
        }
    }

    /**
     * @return the timer
     */
    public Timer getTimer() {
        return timer;
    }

    /**
     * @param timer the timer to set
     */
    public void setTimer(Timer timer) {
        this.timer = timer;
    }

    /**
     * @return the forkliftEncoder
     */
    public Encoder getForkliftEncoder() {
        return forkliftEncoder;
    }

    /**
     * @param forkliftEncoder the forkliftEncoder to set
     */
    public void setForkliftEncoder(Encoder forkliftEncoder) {
        this.forkliftEncoder = forkliftEncoder;
    }

    /**
     * @return the currentHeight
     */
    public double getCurrentHeight() {
        return currentHeight;
    }

    /**
     * @param currentHeight the currentHeight to set
     */
    public void setCurrentHeight(double currentHeight) {
        this.currentHeight = currentHeight;
    }

    /**
     * @return the armJag
     */
    public SpeedController getArmJag() {
        return armJag;
    }

    /**
     * @param armJag the armJag to set
     */
    public void setArmJag(SpeedController armJag) {
        this.armJag = armJag;
    }
}
