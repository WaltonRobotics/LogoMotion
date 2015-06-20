package edu.wpi.first.wpilibj.templates;
import edu.wpi.first.wpilibj.Joystick;

/**
 * @author andrey
 * JoystickControls stores and enables access to the Joystick objects used for controlling the robot.
 */
public class TankControls
{
    //the two joysticks
    private Joystick joystickLeft, joystickRight;

            /**
     * The constructor used to initialize a system with two joysticks.
     * @param leftPort the port used for the initialization for the left joystick.
     * @param rightPort the port used for the initialization for the right joystick.
     */
    public TankControls(int leftPort,int rightPort){
        joystickLeft=new Joystick(leftPort);
        joystickRight=new Joystick(rightPort);
    }

    public double[] getMovementInstructions(){
        double forward = (joystickLeft.getY() + joystickRight.getY()) / 2;
        double turn = (joystickLeft.getY() - joystickRight.getY()) / 2;
        double sideways = (joystickLeft.getX() + joystickRight.getX()) / 2;

        double[] instructions = new double[3];

        instructions[0] = forward;
        instructions[1] = sideways;
        instructions[2] = turn;

        return instructions;
    }

    /**
     * The method returns all axis input
     * @return an array with X,Y, and Twist values for each joystick
     */
    public double[] getAllAxis(){
        double[] axes=new double[6];

        axes[0] = getLeftY();
        axes[1] = getLeftX();
        axes[2] = getLeftTwist();
        axes[3] = getRightY();
        axes[4] = getRightX();
        axes[5] = getRightTwist();

        return axes;
    }

    //All of the following methods do what their name implies in straightforward ways.
    //There are two versions of each method to allow access to both the left and right joysticks
    //X and Y are the joystick axis, Twist is the actual twist of the joystick, and the buttons are the same as on the joystick.
    //TODO: write actual javaDoc for every single method..

    public double getRightX(){
        return joystickRight.getAxis(Joystick.AxisType.kX);
    }

    public double getLeftX(){
        return joystickLeft.getAxis(Joystick.AxisType.kX);
    }

    public double getRightY(){
        return joystickRight.getAxis(Joystick.AxisType.kY);
    }

    public double getLeftY(){
        return joystickLeft.getAxis(Joystick.AxisType.kY);
    }

    public double getRightTwist(){
        return joystickRight.getAxis(Joystick.AxisType.kTwist);
    }

    public double getLeftTwist(){
        return joystickLeft.getAxis(Joystick.AxisType.kTwist);
    }

    public boolean getRightTop(){
        return joystickRight.getButton(Joystick.ButtonType.kTop);
    }

    public boolean getLeftTop(){
        return joystickLeft.getButton(Joystick.ButtonType.kTop);
    }

    public boolean getRightTrigger(){
       return joystickRight.getButton(Joystick.ButtonType.kTrigger);
    }

    public boolean getLeftTrigger(){
       return joystickLeft.getButton(Joystick.ButtonType.kTrigger);
    }

     public boolean getRightButton(int button){
        return joystickRight.getRawButton(button);
    }

     public boolean getLeftButton(int button) {
        return joystickLeft.getRawButton(button);
    }


    public boolean[] getRightButtons(){
        boolean[] buttons=new boolean[12];
        for(int n=1;n<=12;n++)
             buttons[n-1]=joystickRight.getRawButton(n);
        return buttons;
    }

     public boolean[] getLeftButtons(){
        boolean[] buttons=new boolean[12];
        for(int n=1;n<=12;n++)
             buttons[n-1]=joystickLeft.getRawButton(n);
        return buttons;
    }

   
}

