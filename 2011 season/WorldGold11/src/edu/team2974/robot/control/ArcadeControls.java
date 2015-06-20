package edu.team2974.robot.control;

import edu.wpi.first.wpilibj.Joystick;

/**
 * A control system for the robot with one joystick
 *
 * @author Gil
 */
public class ArcadeControls
{
    //the joystick

    private Joystick joystick;

    //creates a new one joystick control system with a joystick connected to a given port
    /**
     *
     * @param Port
     */
    public ArcadeControls(int Port) {
        joystick = new Joystick(Port);
    }

    //uses the joystick axes to get a set of movement instructions for the robot.
    //these instructions are an array of double values telling the robot where it
    //should move and how fast.
    /**
     *
     * @return
     */
    public double[] getMovementInstructions() {
        //sets 3 movement type values to correspond with the current axes
        double forward = getY();
        double sideways = 0;
        double turn = -1 * getX();

        //creates the array that will be returned
        double[] instructions = new double[3];

        //sets the array values
        instructions[0] = forward;
        instructions[1] = sideways;
        instructions[2] = turn;

        return instructions;
    }

    /**
     *
     * @return
     */
    public double[] getInvertedMoveInstructions() {
        //sets 3 movement type values to correspond with the current axes
        double forward = -getY();
        double sideways = 0;
        double turn = -1 * getX();

        //creates the array that will be returned
        double[] instructions = new double[3];

        //sets the array values
        instructions[0] = forward;
        instructions[1] = sideways;
        instructions[2] = turn;

        return instructions;
    }

    //various methods for getting the joystick axes and buttons
    /**
     *
     * @return
     */
    public double getX() {
        return joystick.getAxis(Joystick.AxisType.kX);
    }

    /**
     *
     * @return
     */
    public double getY() {
        return joystick.getAxis(Joystick.AxisType.kY);
    }

    /**
     *
     * @return
     */
    public double getTwist() {
        return joystick.getAxis(Joystick.AxisType.kTwist);
    }

    /**
     *
     * @return
     */
    public boolean getTrigger() {
        return joystick.getButton(Joystick.ButtonType.kTrigger);
    }

    /**
     *
     * @param button
     * @return
     */
    public boolean getButton(int button) {
        return joystick.getRawButton(button);
    }

    //returns an array of all button values that are numbered
    /**
     *
     * @return
     */
    public boolean[] getNumButtons() {
        boolean[] buttons = new boolean[12];
        for (int n = 1; n <= 12; n++) {
            buttons[n - 1] = joystick.getRawButton(n);
        }
        return buttons;
    }

    //returns an array of all button values.
    /**
     *
     * @return
     */
    public boolean[] getButtons() {
        boolean[] buttons = new boolean[14];
        buttons[0] = getTrigger();
        buttons[1] = getTop();
        for (int n = 1; n <= 12; n++) {
            buttons[n + 1] = joystick.getRawButton(n);
        }
        return buttons;
    }

    /**
     *
     * @return
     */
    public boolean getTop() {
        return joystick.getButton(Joystick.ButtonType.kTop);
    }
}
