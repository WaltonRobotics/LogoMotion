/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

/**
 *
 * @author Gil
 */

import edu.wpi.first.wpilibj.Joystick;

//a control system for the robot with one joystick
public class ArcadeControls {
    //the joystick
    private Joystick joystick;

    //creates a new one joystick control system with a joystick connected to a given port
    public ArcadeControls(int Port){
        joystick=new Joystick(Port);
    }

    //uses the joystick axes to get a set of movement instructions for the robot.
    //these instructions are an array of double values telling the robot where it
    //should move and how fast.
    public double[] getMovementInstructions(){
        //sets 3 movement type values to correspond with the current axes
        double forward = getY();
        double sideways = 0;
        double turn = getX();

        //creates the array that will be returned
        double[] instructions = new double[3];

        //sets the array values
        instructions[0] = forward;
        instructions[1] = sideways;
        instructions[2] = turn;

        return instructions;
    }

    //various methods for getting the joystick axes and buttons
    public double getX(){
       return joystick.getAxis(Joystick.AxisType.kX);
    }

   public double getY(){
       return joystick.getAxis(Joystick.AxisType.kY);
    }

    public double getTwist(){
           return joystick.getAxis(Joystick.AxisType.kTwist);
    }

    public boolean getTrigger(){
       return joystick.getButton(Joystick.ButtonType.kTrigger);
    }

    public boolean getButton(int button){
       return joystick.getRawButton(button);
    }

    //returns an array of all button values that are numbered
    public boolean[] getNumButtons(){
         boolean[] buttons=new boolean[12];
         for(int n=1;n<=12;n++){
             buttons[n-1]=joystick.getRawButton(n);
         }
         return buttons;
    }

    //returns an array of all button values.
    public boolean[] getButtons(){
         boolean[] buttons=new boolean[14];
         buttons[0]=getTrigger();
         buttons[1]=getTop();
         for(int n=1;n<=12;n++){
             buttons[n+1] = joystick.getRawButton(n);
         }
         return buttons;
    }

    public boolean getTop(){
        return joystick.getButton(Joystick.ButtonType.kTop);
    }

}
