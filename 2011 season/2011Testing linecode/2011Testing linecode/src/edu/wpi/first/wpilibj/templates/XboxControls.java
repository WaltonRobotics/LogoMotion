/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

/**
 *
 * @author PAMELA
 */

//a control system for the robot with one joystick
import edu.wpi.first.wpilibj.Joystick;

//the Xbox Controls
public class XboxControls {

    //the joystick
    private Joystick joystick;

    //creates a new one joystick control system with a joystick connected to a given port
    public XboxControls(int Port){
        joystick=new Joystick(Port);
    }

    //uses the joystick axes to get a set of movement instructions for the robot.
    //these instructions are an array of double values telling the robot where it
    //should move and how fast.
    public double[] getMovementInstructions(){
        //sets 3 movement type values to correspond with the current axes
        double forward = getLeftY();
        double sideways = getLeftX();
        double turn = getRightX();

        //creates the array that will be returned
        double[] instructions = new double[3];

        //sets the array values
        instructions[0] = forward;
        instructions[1] = sideways;
        instructions[2] = turn;

        return instructions;
    }

    //various methods for getting the joystick axes and buttons
    public double getLeftX(){
        System.out.println("LeftX: " +((32767.5-joystick.getRawAxis(1))/32767.5) );
       return -((32767.5-joystick.getRawAxis(1))/32767.5);
    }

   public double getLeftY(){
       System.out.println("LeftY: " +((32767.5-joystick.getRawAxis(2))/32767.5));
       return ((32767.5-joystick.getRawAxis(2))/32767.5);
    }

   public double getRightX(){
       System.out.println("RightX: " +((32767.5-joystick.getRawAxis(4))/32767.5));
       return -((32767.5-joystick.getRawAxis(4))/32767.5);
    }

   public double getRightY(){
       System.out.println("RightY: " +((32767.5-joystick.getRawAxis(5))/32767.5));
       return ((32767.5-joystick.getRawAxis(5))/32767.5);
    }


   //Triggers (right trigger subtracts and left trigger adds to value)
   public double getZ(){
       System.out.println("Z(triggers): " +joystick.getRawAxis(3));
       return joystick.getRawAxis(3);
    }


    public boolean getButton(int button){
       return joystick.getRawButton(button);
    }

    //returns an array of all button values that are numbered
    public boolean[] getButtons(){
         boolean[] buttons=new boolean[10];
         for(int n=1;n<=10;n++){
             System.out.println("Button("+n+"): "+joystick.getRawButton(n));
             buttons[n-1]=joystick.getRawButton(n);
         }
         return buttons;
    }




}



//Axis indexes:
//1 - LeftX
//2 - LeftY
//3 - Triggers (Each trigger = 0 to 1, axis value = right - left)
//4 - RightX
//5 - RightY
//6 - DPad Left/Right
//
//Button mapping matches Windows Control Panel>Game Pads display




//Button numbers defualt chart
//
//     A:1
//     B:2
//     X:3
//     Y:4
//     LB:5
//     RB:6
//     Back:7
//     Start:8
//     LeftJoystickPush:9
//     RightJoystickPush:10

