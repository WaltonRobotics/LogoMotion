/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;

/**
 *
 * @author Paul
 */
public class Arm {

    public SpeedController forkliftJag, armJag;
    public Timer timer;
    public double currentHeight;
    public Solenoid wrist;
    public Solenoid claw;
    public boolean toggleSwitchClaw;
    public boolean toggleSwitchWrist;
    public Encoder armEncoder;
    public double[] heights;
    //private int oscillateCounter;
    private double oscillateAngle;
    private boolean oscillating;
    public Gyro gyro;
    private double gyroOffset;
    private double armSpeed;
    final double GYRO_ANGLE;
    static final double FLIP_HEIGHT = 130;
    double lastGyroCorrect;
    boolean flippedDown;


    /**
     * Class Constructor specifying which channel the Jaguar Motor controller
     *      that controls the arm is plugged into.
     * @param forkliftJagChannel an integer that represents which channel to call the
     *      Jaguar motor controller that controls the forklift
     * @param armJagChannel an integer that represents which channel to call the
     *      Jaguar motor controller that controls the arm
     */
    public Arm(int forkliftJagChannel, int armJagChannel, int wristChannel, int clawChannel, int gyroChannel) {
        flippedDown = false;
        timer = new Timer();
        timer.reset();
        timer.start();
        lastGyroCorrect = 0;
        forkliftJag = new Jaguar(forkliftJagChannel);
        armJag = new Jaguar(armJagChannel);
        timer = new Timer();
        currentHeight = 0;
        wrist = new Solenoid(wristChannel);
        claw = new Solenoid(clawChannel);
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
        armEncoder = new Encoder(9, 10);
        armEncoder.start();
        armEncoder.setDistancePerPulse(5 * 3.14 / 360);
        oscillating = false;
        gyro = new Gyro(gyroChannel);
        gyro.reset();
        GYRO_ANGLE = gyro.getAngle();
        gyroOffset = 180;
        currentHeight = 0;
        //oscillateCounter = 0;
        armSpeed = 0;
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
    public void gyroCorrect(){
        if(timer.get() - lastGyroCorrect > 6){
            gyroOffset += 1;
            lastGyroCorrect = timer.get();
        }
    }
    //returns the interpretted angle of the arm
    public double getArmAngle() {
        return -gyro.getAngle() + gyroOffset;
    }

    //this method is called when the arm is vertical to reset the gyro (corrects for drift)
    public void resetArmAngle() {
        gyro.reset();
    }

    /**
     * Tells the forklift to go up at full speed
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
     * @param pressedDown a boolean that represents whether the button that controls
     *      the arm going up is pressed
     */
    public void armControl(double speedToGo) {
        armJag.set(speedToGo);
    }

    public void forkliftControl(double speedToGo) {
        forkliftJag.set(speedToGo);
        if (speedToGo < 0) {
            currentHeight -= armEncoder.getDistance();
        } else {
            currentHeight += armEncoder.getDistance();
        }
        armEncoder.reset();
        armEncoder.start();

    }

    public void goToAngle(double angle) {
        oscillating = false;


    }

    /**
     * Turns solenoid on.
     * More will be added once I learn what a solenoid is!
     */
    public void grabTube(boolean pushedDown) {
        claw.set(pushedDown);
    }

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
     * @return adjusterSpeed a double that represents the current adjusterSpeed
     */
//    public double getAdjusterSpeed() {
//        return adjusterSpeed;
//    }
    public void clawSwitcher() {
        toggleSwitchClaw = !toggleSwitchClaw;
    }

    public void wristSwitcher() {
        toggleSwitchWrist = !toggleSwitchWrist;
    }

    public void wristM() {
        wrist.set(toggleSwitchWrist);
        wristSwitcher();
    }

    public void wristUp() {
        wrist.set(false);
        flippedDown = false;
    }

    public void wristDown() {
        wrist.set(true);
        flippedDown = true;
    }

    public void clawM() {
        claw.set(toggleSwitchClaw);
        clawSwitcher();
    }

    public void wristFlipCheck() {
        if (getArmAngle() > FLIP_HEIGHT && !flippedDown && getArmAngle() < FLIP_HEIGHT+3){
             wristDown();
        }
        if(getArmAngle() < FLIP_HEIGHT && flippedDown && getArmAngle() > FLIP_HEIGHT-3){
            wristUp();
        }
    }

    /*an abstract class to represent an autonomous function of our robot
     *Any class extending this one should use running as a condition for all of its loops
     *or potential "stuck" points.
     * For example, say the robot is trying to follow a line, and some boolean variable
     * done is used to check whether the robot is at the end of the line.
     *  Then, the basic loop should have "while(!done && running) so that it will terminate in the
     * case that EITHER it reaches the end of the line OR that the robot is telling it to stop.
     * Additionally, all such functions should set running = false; at the end of the run() method
     * and should ONLY be called with the begin() method, not the start() method.
     */
    /*additional note: We need to discuss whether we really want to nest all this in robotMain
    systems of robotMain as paramters- i know we don't want to couple, but it is kind of awkward to
    nest EVERY autonomous thread we want within the RobotMain class
     */
    public abstract class AutoFunction extends Thread {

        public boolean running;

        //sets running to be false, which for a validly written AutoFUnction will stop the thread
        public void stop() {
            running = false;
        }

        //checks whether the thread is currently running
        public boolean isRunning() {
            return running;
        }

        public abstract boolean atCorrectHeight();


        //sets running = true and starts the thread if not already started
        public void begin() {
            try {
                running = true;
                this.start();
            } catch (RuntimeException rte) {
            }
        }
    }

    public class GoToHeight extends AutoFunction {

        double goalHeight;
        public boolean moveDirectionUp;
        private boolean farFromTarget;
        private boolean atHeight;

        public GoToHeight(int i) {
            if (0 <= i && i <= 8) {
                goalHeight = heights[i];
            } else {
                goalHeight = currentHeight;
            }
            moveDirectionUp = false;
            farFromTarget = true;
            atHeight = false;
        }

        public boolean atCorrectHeight(){
            return atHeight;
        }

        public void run() {
            armEncoder.reset();
            armEncoder.start();
            while (running) {
                while (farFromTarget) {

                    if (moveDirectionUp) {
                        currentHeight += armEncoder.getDistance();
                    } else {
                        currentHeight -= armEncoder.getDistance();
                    }

                    armEncoder.reset();
                    if (currentHeight < goalHeight) {
                        moveDirectionUp = true;
                        forkliftControl(1.0);
                    }
                    else {
                        moveDirectionUp = false;
                        forkliftControl(-.2); //1/10 instead of whole unit because of gravity
                    }
                    if (Math.abs(currentHeight - goalHeight) < 2) {
                        farFromTarget = false;
                        atHeight = true;
                    }
                }
                while (!farFromTarget) {
                    if (moveDirectionUp) {
                        currentHeight += armEncoder.getDistance();
                    } else {
                        currentHeight -= armEncoder.getDistance();
                    }

                    armEncoder.reset();
                    if (currentHeight < goalHeight) {
                        moveDirectionUp = true;
                        forkliftControl(0.4);
                    } else {
                        moveDirectionUp = false;
                        forkliftControl(-.1); //1/10 instead of whole unit because of gravity
                    }
                    if (Math.abs(currentHeight - goalHeight) > 2) {
                        farFromTarget = true;
                        atHeight = false;
                    }
                }
            }
            running = false;
        }
    }

    public boolean isOscillating() {
        return oscillating;
    }

    public void heightenArm() {
        oscillating = false;
        armJag.set(-1);
        armSpeed = -1;
    }

    public void lowerArm() {
        oscillating = false;
        armJag.set(0.4);
        armSpeed = .4;
    }

    public void oscillate() {
        //oscillateCounter++;
        if (!oscillating) {
            oscillating = true;
            oscillateAngle = getArmAngle();
        }
        if (oscillating && Math.abs((getArmAngle() - oscillateAngle)) > 2) {
            /**
            if(oscillateCounter>=100)
            {
            oscillateAngle=gyro.getAngle();
            oscillateCounter=0;
            }
             * */
        }
        if (getArmAngle() < oscillateAngle) {
            heightenArm();
        } else if (getArmAngle() > oscillateAngle) {
            lowerArm();
        }
    }
}
