package edu.wpi.first.wpilibj.templates;

/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */



/**
 *
 * @author Gil
 */
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Jaguar;

//this class exists to express all of the similarities between the two arcade drives
//and the tank drive.  It contains the code for processing the motorSpeeds after they are
//calculated from joystick inputs, then using the speeds to set the controllers
// The only parts that are overwritten in subclasses are the constructors
//and the setSpeedPair() methods that convert the joystick input into raw motor
//speeds.

import edu.wpi.first.wpilibj.Encoder;
public class TwoMotorDriveTest extends DriveTrain{


    SpeedController leftController; //the left jaguar
    SpeedController rightController; //the right jaguar
    int rightOrientation;
    int leftOrientation;
    Encoder leftWheel;
    Encoder rightWheel;
    SpeedPair goal;

    //constructs a WaltTwoMotor drive with appropriate controllers and demo mode
    public TwoMotorDriveTest(int channelLeft, int channelRight, boolean demo){
        leftController = new Jaguar(channelLeft);
        rightController = new Jaguar(channelRight);
        leftOrientation=1;
        rightOrientation=1;
        speedsPrevious = new SpeedPair(0,0);
        demoMode = demo;
        int lChan1 = 2;
        int lChan2 = 3;
        int rChan1 = 4;
        int rChan2 = 5;
        leftWheel  = new Encoder(lChan1, lChan2);
        rightWheel = new Encoder(rChan1, rChan2);
        leftWheel.setDistancePerPulse(8*3.14/360);
        rightWheel.setDistancePerPulse(8*3.14/360);
    }

    public void handleAutonomous(){

    }

    //stops all movement
    public void stop(){
        SpeedPair stop = new SpeedPair(0,0);
        setSpeeds(stop);
    }

    //turns the robot at the given speed
    //positive is right, negative is left
    public void turn(double speed){
        SpeedPair turning = new SpeedPair(speed,-speed);
        turning.reduce();
        setSpeeds(turning);
    }

   //goes forward at a given speed- positive is forward, negative is backward.
    //Should be between -1 and 1, will interpret all others as full speed.
    public void goForward(double speed){
        SpeedPair forward = new SpeedPair(speed,speed);
        forward.reduce();
        setSpeeds(forward);
    }

    //generates a raw SpeedPair based on movement instructions
    public SpeedSet getSpeeds(double[] instructions){
        double leftSpeed;
        double rightSpeed;
        double forward = instructions[0];
        double turn = instructions[2];

        leftSpeed = forward + turn;
        rightSpeed = forward - turn;

        SpeedPair speeds = new SpeedPair(leftSpeed,rightSpeed);
        return speeds;
    }

    //chain limits the speeds appropriately based on demoMode
    public SpeedSet accelerationLimit(SpeedSet speeds){
         if(demoMode){
             speeds.limitIncreaseTo(speedsPrevious);
             return speeds;
         }
         else {
             speeds.limitTo(speedsPrevious);
             return speeds;
         }
     }

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

     //sets the speeds of the motors and updates leftPrev and rightPrev to reflect
     //the new speeds
     public void setSpeeds(SpeedSet spdsNew){
        spdsNew = (SpeedPair)spdsNew;
       SpeedPair speedsNew = (SpeedPair)spdsNew;
       goal.left = speedsNew.left;
       goal.right = speedsNew.right;
    }

     public void setInvertedSide(boolean right)
     {
         if(right)
        rightOrientation=-1;
         else
        leftOrientation=-1;
    }

    public class goalChaser extends Thread{
        SpeedPair current;
        double divider = 8 * 3.14;
        public void run(){
            while(true){
                current.left = leftWheel.getRate() / divider ;
                current.right = rightWheel.getRate() / divider;
                if(current.left < goal.left){
                    current.left = current.left + .06;
                }
                else{
                    current.left = current.left - .06;
                }
                if(current.right < goal.right){
                    current.right = current.right + .06;
                }
                else{
                    current.right = current.right - .06;
                }
                if(current.right>1){
                    current.right = 1;
                }
                if(current.right < -11){
                    current.right = -1;
                }
                if(current.left > 1){
                    current.left = 1;
                }
                if(current.left < -1){
                    current.left = -1;
                }
                leftController.set(current.left * leftOrientation);
                rightController.set(current.right * rightOrientation);
            }
        }
    }
}

