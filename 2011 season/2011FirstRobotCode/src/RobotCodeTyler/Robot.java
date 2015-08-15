package RobotCodeTyler;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

//question: should we use continous or periodic ("slow loop") methods?
//for more info, check out out the comment at the top of IterativeRobot.java
/**
 *
 * @author Tyler
 * This represents the robot and is the top-level class.
 */
public class Robot extends IterativeRobot
{
    //demo mode, direct drive mode (no chain limiters) (require password?), competition mode, testing mode(?)

    public static final int COMPETITION_MODE = 0, DEMO_MODE = 1, DIRECT_DRIVE_MODE = 2, TESTING_MODE = -1;
    public static final int ARCADE_CONFIGURATION = 0, TANK_CONFIGURATION = 2;
    //possibly later we can dynamically change the drivetrain type based on how many joysticks are plugged in
    //private JoystickList driveStationJoysticks;
    //see note at bottom
    private Joystick[] Joysticks;
    //private double[] driveMotorPreviousValues; - replaced by the Jaguar get() method
    private final double maxDriveTrainAcceleration = 0.1; //an arbitrary #. Must be between 0 & 1.
    private int driveMotors; //number of motors used in driveTrain
    private final DriverStation robotDriverStation = DriverStation.getInstance();
    private final int[] driveMotorChannels = {1, 2}; //reset to actual channels
    private int mode;
    private DriveTrain robotDriveTrain;

    /*
     * called to start up the robot
     * @override
     */
    public void robotInit()
    {
        robotDriveTrain = new DriveTrain(driveMotorChannels);
        //or DriveTrain robotDriveTrain = new MecanumDriveTrain(); // this would subclass DriveTrain
        driveMotors = robotDriveTrain.getNumMotors();
    }

    /* sets the mode (competition, demo, direct drive, testing)
     * to be used from the driver station
     * @param mode use the mode constants in <code>Robot</code>
     */
    public void setMode(int mode)
    {
        //hardcode a password later on to exit demo mode?
        this.mode = mode;
    }

    public void disabledInit()
    {
    }

    public void disabledContinous()
    {
    }

    public void autonomousInit()
    {
    }

    public void autonomousContinuous()
    {
    }

    public void teleopInit()
    {
    }

    public void teleopContinuous()
    {
        //call the proper updateDriveTrain method from here (after checking how many joysticks are plugged in?)
    }

    //if we want to save data to variables instead of accesing it every time
    public void saveJoystickData()
    {
    }

    /*
     * note on updateDriveTrain:
     * updateDriveTrain() is overloaded to provide various updating options based on joystick configuration.
     * I felt that the number of joysticks should be totally independent of the driveTrain system on the robot.
     * That is why we deal with Joysticks in Robot, then pass values for the motor controllers in DriveTrain.
     * Possible configurations that I considered:
     * arcade (1 joystick, 2 axes)
     * tank (2 joysticks, 2 axes)
     * mecanum/holonomic (1 joystick 3 axes)
     * however, more methods can be added as needed
     * When 2 configurations use the same number & type of parameters, I added a joystick configuration parameter
     * that uses constants (like the robot mode) to determine which configuration to use.
     * Also, this will make it easy to dynamically change the code based on current joystick configuration
     * Also, modifying the axes (e.g. adjustedX, adjustedY below) based on sensor data will be
     * really simple this way. We simply access the current sensor data in the updateDriveTrain method.
     */
    public void updateDriveTrain(double joystick1X, double joystick1Y, double joystick1Twist)//pass all values needed
    {
        //limit the total power available in demo mode
        if (mode == DEMO_MODE)
        {
        }
    }

    /*
     * Tank: axis1 = joystick1Y, axis2 = joystick2Y
     * arcade: axis1 = joystick1X, axis2 = joystick1Y
     * @param joystickConfig use the constants available in <code>Robot</code>
     */
    public void updateDriveTrain(double axis1, double axis2, int joystickConfig)
    {
        double leftMotorSpeed = -2, rightMotorSpeed = -2; //motor speed goes from -1 to 1

            //limits acceleration if not in direct drive mode (where joystick input gets sent directly to the motor controllers)
            if (mode != DIRECT_DRIVE_MODE)
            {
                if (joystickConfig == ARCADE_CONFIGURATION) //convert arcade values to left/right motor controller values
                {
                //copy-pasted from RobotDrive
                if (axis1 > 0.0)
                {
                    if (axis2 > 0.0)
                    {
                        leftMotorSpeed = axis1 - axis2;
                        rightMotorSpeed = Math.max(axis1, axis2);
                    }
                    else
                    {
                        leftMotorSpeed = Math.max(axis1, -axis2);
                        rightMotorSpeed = axis1 + axis2;
                    }
                }
                else
                {
                    if (axis2 > 0.0)
                    {
                        leftMotorSpeed = -Math.max(-axis1, axis2);
                        rightMotorSpeed = axis1 + axis2;
                    }
                    else
                    {
                        leftMotorSpeed = axis1 - axis2;
                        rightMotorSpeed = -Math.max(-axis1, -axis2);
                    }
                }
                }
                else if(mode == TANK_CONFIGURATION)
                {
                    leftMotorSpeed = axis1;
                    rightMotorSpeed = axis2;
                }
                double[] currentJagValues = robotDriveTrain.getCurrentPWMValues();
                double currentLeftMotorSpeed = -2, currentRightMotorSpeed = -2;
                if(currentJagValues.length == 2)
                {
                    currentLeftMotorSpeed = currentJagValues[0];
                    currentRightMotorSpeed = currentJagValues[1];
                }
                else if(currentJagValues.length == 4)
                {
                    currentLeftMotorSpeed = currentJagValues[1];
                    currentRightMotorSpeed = currentJagValues[3]; //indexes are as follows: frontLeft, rearLeft, frontRight, rearRight
                }
                else
                {
                    throw new IllegalArgumentException("currentJagValues has an incorrect length.");
                }

                leftMotorSpeed = limitAcceleration(currentLeftMotorSpeed, leftMotorSpeed);
                rightMotorSpeed = limitAcceleration(currentRightMotorSpeed, rightMotorSpeed);

                robotDriveTrain.setLeftRightMotorSpeeds(leftMotorSpeed, rightMotorSpeed);
            }
            else
            {
                robotDriveTrain.arcadeDrive(axis1, axis2);
                /*
                 * note that the arcadeDrive method doesn't actually require an arcade joystick configuration
                 * it simply makes it easier to set the motor values by taking the x and y values as parameters
                 * then converting those to motor controller values for the 2 side motors.
                 * However, we will be dealing almost entirely with tank drive
                 * since we will be limiting the acceleration OF THE INDIVIDUAL MOTORS
                 */
            }
        
    }

    /*
     * Uses maxDriveTrainAcceleration to limit the change in motor speed
     * @param currentSpeed the current motor speed
     * @param newSpeed the new motor speed
     */
    public double limitAcceleration(double currentSpeed, double newSpeed)
    {
        if(Math.abs(newSpeed - currentSpeed) > maxDriveTrainAcceleration)
                {
                    if(newSpeed > currentSpeed)
                    {
                        newSpeed = currentSpeed + maxDriveTrainAcceleration;
                    }
                    else
                    {
                        newSpeed = currentSpeed - maxDriveTrainAcceleration;
                    }
                }
        return newSpeed;
    }
}

    /* TODO:check out the following:
     * in wpilibj.DriverStation:
     *          kJoystickPorts      - is this the total # of usb ports?
     *          getStickAxis()      - could check if a joystick exists?
     *          kJoystickAxes       - a constant or a variable?
     *          getStickButtons()   - same as above
     *          setDigitalOut()     - could be useful later on?
     *          getEnhancedIO()     - see below
     * in DriverStationEnhancedIO
     *          getDigital()        - maybe joysticks can be accessed this way
     *          getTouchSlider()    - is this the laptop touchpad slider?
     *          resetEncoder()      - also contains other encoder methods
     *          setLED()            - what is the IO Board?
     */
