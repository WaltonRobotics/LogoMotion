/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

/**
 *
 * @author Gil
 */
import edu.wpi.first.wpilibj.SpeedController;

//a drive system for an omniWheel drive system where the motors are oriented
//so that setting them to a positive speed makes them all go clockwise around the robot.
public class OmniWheelDrive extends FourMotorDrive{

    //constructs an omniWheel drive
    public OmniWheelDrive(SpeedController frontL, SpeedController frontR, SpeedController backL, SpeedController backR, boolean demo){
        super(frontL,frontR,backL,backR,demo);
    }

     //generates an initial SpeedQuadruple based on joystick values
    public SpeedSet getSpeeds(double[] instructions){

        //translates array into meaningful variables
        double forward = instructions[0];
        double sideways = instructions[1];
        double turn = instructions[2];

        //translates movement variables into motor speeds.
        //motors run same direction to go forward.
        //left motors and right motors run opposite directions to turn.
        //front motors and back motors run opposite directions to go sideways.
        double frontL = forward + turn + sideways;
        double frontR = forward - turn + sideways;
        double backL = forward + turn - sideways;
        double backR = forward - turn - sideways;

        SpeedQuadruple speeds = new SpeedQuadruple(frontL, frontR, backL, backR);
        return speeds;
    }
}