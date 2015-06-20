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
//a drive system for a mecanum wheel drive train that uses one joystick.
//it controls forward and backward motion via the main axis of the stick
//side to side motion via the sideways axis of the stick, and turning via
//the joystick's twist
public class FourMotorDrive extends DriveTrain{


    //the four motorcontrollers
    SpeedController frontLeft;
    SpeedController frontRight;
    SpeedController backLeft;
    SpeedController backRight;
    
    public FourMotorDrive(SpeedController frontL, SpeedController frontR, SpeedController backL, SpeedController backR, boolean demo){

        //sets the four motorControllers
        frontLeft = frontL;
        frontRight = frontR;
        backLeft = backL;
        backRight = backR;
        demoMode = demo;

        //sets the initial speeds to 0;
        speedsPrevious = new SpeedQuadruple(0,0,0,0);
    }

    public void handleAutonomous(){

    }

    //stops all motors
    public void stop(){
        SpeedQuadruple stop = new SpeedQuadruple(0,0,0,0);
        setSpeeds(stop);
    }


    //turns the robot at a given speed- positive is right, negative is left. On a
    //scale from -1 to 1, will interpret all others as full speed.
    public void turn(double speed){
        SpeedQuadruple turning= new SpeedQuadruple(speed,-speed,speed,-speed);
        turning.reduce();
        setSpeeds(turning);
    }

    //goes forward at a given speed- positive is forward, negative is backward.
    //Should be between -1 and 1, will interpret all others as full speed.
    public void goForward(double speed){
        SpeedQuadruple forward = new SpeedQuadruple(speed, speed, speed, speed);
        forward.reduce();
        setSpeeds(forward);
    }

    //generates an initial SpeedQuadruple based on joystick values
    public SpeedSet getSpeeds(double[] instructions){
        double forward = instructions[0];
        double turn = instructions[2];

        double leftSpeed = forward + turn;
        double rightSpeed = forward - turn;

        SpeedQuadruple speeds = new SpeedQuadruple(leftSpeed,rightSpeed,leftSpeed,rightSpeed);
        return speeds;
    }

    /**
     * sets the motor controllers based on a given quadruple and
     * updates speedsPrevious to reflect the change
     * @param speedsNew the speeds that the motors will be set to, must be a quadruple
     */
    public void setSpeeds(SpeedSet spdsNew){
        if(frontLeft == null || frontRight == null || backLeft == null || backRight == null) {
            throw new NullPointerException("Null motor provided");
        }

        //casts into a quadruple
        SpeedQuadruple speedsNew = (SpeedQuadruple)spdsNew;

        backLeft.set(speedsNew.back.left);
        frontLeft.set(speedsNew.front.left);
        backRight.set(speedsNew.back.right);
        frontRight.set(speedsNew.front.right);
        speedsPrevious = speedsNew;
    }

    public void setInvertedSide(boolean right){};
}

