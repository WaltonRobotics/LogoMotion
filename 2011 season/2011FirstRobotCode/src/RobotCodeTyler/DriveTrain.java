package RobotCodeTyler;

import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.RobotDrive;


/**
 *
 * @author Tyler
 */

/* note: I would have extended robotDrive, then called superclass constructor except
 * it would have to be the first statement of these constructors.
 * instead it mimics this by creating a drivetrain object that it accesses
 * for many of its methods
 */

/*
 * Note: This drivetrain requires that the left motor = motorChannels[0]
 * and right motor = motorChannels[1] for 2-motor driving.
 * For 4 motor driving, the order is as follows: frontLeft, rearLeft, frontRight, rearRight.
 * This is the same order as in RobotDrive.
 */

/*
 * TODO: create another constructor that also takes the slot of the motor controllers as a parameter
 * TODO: I don't like that I used an array, instead of similar constructors to RobotDrive
 * I would prefer to have variables for frontLeft, frontRight, rearLeft, and rearRight
 */

public class DriveTrain //extends RobotDrive
{
    private RobotDrive FRCDrive;
    private int numMotors;
    private Jaguar[] motorControllers;
    
    public DriveTrain(int[] motorChannels)
    {
        switch(motorChannels.length)
        {
            //must start with case 4, or array of length 2 will throw arrayIndexOutOfBounds
            case 4:
            {
                numMotors = 4;

                //this accomplishes the same as case 2 - verifies each motor channel is > 0
                boolean motorChannelsValid = true;
                for(int index = 0; index < motorChannels.length; index++)
                {
                    if(motorChannels[index] < 0)
                    {
                        motorChannelsValid = false;
                    }
                }
                if(motorChannelsValid)
                {
                    for(int index = 0; index < motorChannels.length; index++)
                    {
                        motorControllers[index] = new Jaguar(motorChannels[index]);
                    }
                    FRCDrive = new RobotDrive(motorControllers[0], motorControllers[1], motorControllers[2], motorControllers[3]);
                }
            }
            case 2:
            {
                numMotors = 2;
                if(motorChannels[0] > 0 && motorChannels[1] > 0)
                {
                    motorControllers[0] = new Jaguar(motorChannels[0]);
                    motorControllers[1] = new Jaguar(motorChannels[1]);
                    
                    FRCDrive = new RobotDrive(motorControllers[0], motorControllers[1]);
                    break;
                }
            }
            default: throw new IllegalArgumentException("Must have 2 or 4 motors!");
        }
    }

    //all other drive methods will be similar to this
    //but I will only include those that take values as parameters, not joysticks
    public void arcadeDrive(double moveValue, double rotateValue)
    {
        FRCDrive.arcadeDrive(moveValue, rotateValue);
    }

    /*
     * see documentation in <code>RobotDrive</code>. I like the FRC code.
     */
    public void setLeftRightMotorSpeeds(double leftSpeed, double rightSpeed)
    {
        FRCDrive.setLeftRightMotorOutputs(leftSpeed, rightSpeed);
    }

    //sends the values that all drivetrain jaguars are currently set to.
    public double[] getCurrentPWMValues()
    {
        double[] motorValues = new double[numMotors];
        for(int index = 0; index < motorValues.length; index++)
        {
            motorValues[index] = motorControllers[index].get();
        }
        return motorValues;
    }

    //returns number of motors used in the drivetrain
    public int getNumMotors()
    {
        return numMotors;
    }
}
