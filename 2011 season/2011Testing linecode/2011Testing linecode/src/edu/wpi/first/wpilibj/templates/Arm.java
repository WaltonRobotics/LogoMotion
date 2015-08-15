/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Timer;

/**
 *
 * @author Paul
 */
public class Arm {

    public SpeedController forkliftJag, armJag;
    public Timer timer;
    public int currentHeight;

    /**
     * Class Constructor specifying which channel the Jaguar Motor controller
     *      that controls the arm is plugged into.
     * @param forkliftJagChannel an integer that represents which channel to call the
     *      Jaguar motor controller that controls the forklift
     * @param armJagChannel an integer that represents which channel to call the
     *      Jaguar motor controller that controls the arm
     */
    public Arm(int forkliftJagChannel, int armJagChannel) {
        forkliftJag = new Jaguar(forkliftJagChannel);
        armJag = new Jaguar(armJagChannel);
        timer = new Timer();
        currentHeight = -1;
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
/**
 * Tells the forklift to go up at full speed
 * @param pressedDown a boolean that represents whether the button that controls
 *      the forklift going down is pressed
 */
    public void forkliftDown(boolean pressedDown) {
        if (pressedDown) {
            forkliftJag.set(-1);
        } else {
            forkliftJag.set(0);
        }
    }
/**
 * Tells the arm to go up at full speed
 * @param pressedDown a boolean that represents whether the button that controls
 *      the arm going up is pressed
 */
    public void armUp(boolean pressedDown) {
        if (pressedDown) {
            armJag.set(1);
        } else {
            armJag.set(0);
        }

    }
/**
 * Tells the arm to go down at full speed
 * @param pressedDown a boolean that represents whether the button that controls
 *      the arm going down is pressed
 */
    public void armDown(boolean pressedDown) {
        if (pressedDown) {
            armJag.set(-1);
        } else {
            armJag.set((0));
        }
    }

    /**
     * Adjusts the height of the arm to the given height value
     * @param height the height value that corresponds with the different
     *      possible height locations
     */
    private void goHeight(int height) {
        //How long the Jag needs to run for per height.
        int[] jagTime = new int[8];
        /**
         *The height values are ordered by the lowest number being closest to
         *      the ground
         *jagTime WILL be changed and adjusted after testing to get the height
         *      correct
         */
        /**
         * Height Chart:
         * 0-Ground
         * 1-Bottom Side Peg
         * 2-Bottom Center Peg
         * 3-Feeder Slot
         * 4-Middle Side Peg
         * 5-Middle Center Peg
         * 6-Top Side Peg
         * 7-Top Center Peg
         */
        jagTime[0] = 0;
        jagTime[1] = 100;
        jagTime[2] = 200;
        jagTime[3] = 300;
        jagTime[4] = 400;
        jagTime[5] = 500;
        jagTime[6] = 600;
        jagTime[7] = 700;
        /**
         * The amount of time the Jag moves is the *goal* height minus the
         *      current height.
         */
        int moveAmount = jagTime[height] - jagTime[currentHeight];
        int moveDirection;

        /**
         * If the *goal* height minus the current height is positive, the Jag
         *      runs forwards
         * If the *goal* height minus the current height is negative, the Jag
         *       runs backwards
         * If the *goal* height minus the current height is 0, the Jag does not
         *      move
         */
        if (moveAmount > 0) {
            moveDirection = 1;
        } else if (moveAmount < 0) {
            moveDirection = -1;
        } else {
            moveDirection = 0;
        }
        /**
         * Starts the timer and the jaguar in the given direction
         */
        timer.start();
        forkliftJag.set(moveDirection);
        /**
         * Stops the timer and Jaguar when the timer is equal to or greater than
         *      variable moveAmount
         */
        if (timer.get() >= Math.abs(moveAmount)) {
            forkliftJag.set(0);
            timer.stop();
        }
        currentHeight = height;
    }

    /**
     * To be created.
     */
    public void grabTube() {
    }

    /**
     * To be created.
     */
    public void releaseTube() {
    }
}
