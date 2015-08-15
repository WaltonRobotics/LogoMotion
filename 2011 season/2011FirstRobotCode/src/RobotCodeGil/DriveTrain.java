/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

/**
 *
 * @author Gil
 */
public abstract class DriveTrain {

    //maximum acceleration of a motor in one setting
    public static final double maxChange = .2;

    //the previous speeds
    public SpeedSet speedsPrevious;

    //whether the robot is in demo mode
    public boolean demoMode;

    //stops all movement
    public abstract void stop();

    //turns at given speed
    public abstract void turn(double speed);

    //goes forward at given speed
    public abstract void goForward(double speed);

    //gets a set of speeds from movement instructions
    public abstract SpeedSet getSpeeds(double[] instructions);

     //divides the pair by 2 if we are at a demo or overDrive is not engaged
    public SpeedSet handleOverDrive(SpeedSet speeds, boolean overDrive){

         //still goes only 3/4 speed with overDrive at a demo
         if(demoMode && overDrive){
             speeds.divideBy(4/3);
             return speeds;
         }

         //goes full speed if overDrive and not at a demo
         if(!demoMode && overDrive){
             return speeds;
         }

         //if overDrive is not engaged, goes at half speed regardless of demoMode
         else{
             speeds.divideBy(2);
             return speeds;
         }
     }



    //limits the acceleration appropriately based on demoMode
     public SpeedSet accelerationLimit(SpeedSet speeds){
         if(demoMode){
             speeds.limitIncreaseTo(speedsPrevious);
             return speeds;
         }
         else {
             speeds.limitIncreaseTo(speedsPrevious);
             return speeds;
         }
     }

    //sets speeds to the motors
    public abstract void setSpeeds(SpeedSet speeds);

    public abstract void setInvertedSide(boolean right);
}
