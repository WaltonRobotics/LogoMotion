package edu.wpi.first.wpilibj.templates;
import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationEnhancedIO;
import edu.wpi.first.wpilibj.DriverStationEnhancedIO.EnhancedIOException;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.DigitalInput;
/**
 * @author andrey
 * The RobotMain class instantiates and coordinates the rest of robot's classes.
 */
public class RobotMain extends IterativeRobot
{
    private DriveTrain robotDrive;//a two motor drive
    private TankControls joysticks;//a tank drive control system for the robot
    private pneumaticSystem pneumatics;
    public SensorSet sensors;//Sensors currently set to public because there is not much that can be changed and access could be needed in other classes.
    private boolean demoMode;
    private DriverStationLCD driverScreen;//Object used to display information to the driver as needed.
    private DriverStationEnhancedIO enhancedIO = DriverStation.getInstance().getEnhancedIO();
    private DigitalInput testLightSensor;

    /**
     * In accordance with the superClass, robotInit method is used for initialization rather than a constructor.
     */
    public void robotInit()
    {

     demoMode = false;
     Watchdog.getInstance().setEnabled(true);
     Watchdog.getInstance().setExpiration(2);
     joysticks=new TankControls(1,2);//All channel values arbitrary at this point and may need to be changed.

     //initializes pneumatics
     int[] solenoidChannels={3,4};
     int spikeChannel = 1;
     int pressureSwitch = 2;
     //pneumatics=new pneumaticSystem(solenoidChannels,spikeChannel,pressureSwitch, 120);

     //channel for gyro
     int gyroChannel = 1;

     //channels for camera initiation
     int baseCamChannel = -1;
     int panMotorCamChannel = 9;
     int servoCamChannel = 8;

     //channels for accelerators- may want multiple for multiple directions
     int accSlot=-1;

     //initiates sensors
    // sensors=new SensorSet(gyroChannel,accSlot,baseCamChannel,panMotorCamChannel,servoCamChannel);

     //initiates drive train
     robotDrive = new TwoMotorDrive(1,2, demoMode);
     robotDrive.setInvertedSide(true);//boolean is true to invert right, false for left

     //initiates the screen
     driverScreen=DriverStationLCD.getInstance();
     //testLightSensor=new DigitalInput(1);
     System.out.println("test is running");
    }

    /**
     * The method that runs and controls the Robot's autonomic behavior. Most, if not all, of it will be written after the
     * game is announced.
     */

    public void autonomousPeriodic()
    {
    Watchdog.getInstance().feed();
    SpeedPair forwardSpeedPair = new SpeedPair(-0.7,0.7);
    robotDrive.setSpeeds(forwardSpeedPair);
    }

    /**
     * The method that allows control of the robot during the teleoperational period and displays information from the sensors on the screen.
     * This runs in an infinite loop that updates every .02 seconds as per last year for the entirety of the teleoperational period.
     */

    public void teleopContinuous(){

        while(true)
        {
            Watchdog.getInstance().feed();
            try {
                handleInput();
            } catch (EnhancedIOException ex) {
                ex.printStackTrace();
            }
            driverScreen.println(DriverStationLCD.Line.kUser2, 1, "Angle: " + sensors.getGyroAngle());
            driverScreen.println(DriverStationLCD.Line.kUser3, 1, "Light: " + testLightSensor.get());

            System.out.print(sensors.getGyroAngle());
            Timer.delay(.02);

        }
}

    public void disabledInit() {
        System.out.println("Disabling robot and all threads.");
    }

    public void disabledPeriodic() {
    }

    /**
     * This method checks the input from the controllers and sends the appropriate input to the driver and pneumatics systems.
     * It also initiates safeStop() in case the proper buttons are pressed.
     */
    private void handleInput() throws EnhancedIOException
    {

        if(joysticks.getRightTop())//Buttons 1 on one of the two or the single controller calls safeStop()
            safeStop();
        else
        {
            if(joysticks.getRightTrigger())
            {
               //System.out.println("light sensor:"+testLightSensor.get()+", and dark is "+testLightSensor2.get()+", and powered is"+testLightSensor3.get());
               /* System.out.println(sensors.getAcceleration(ADXL345_I2C.Axes.kZ));
                System.out.println(sensors.getGyroAngle());
                driverScreen.println(DriverStationLCD.Line.kUser2, 1, "Angle: " + sensors.getGyroAngle());
                driverScreen.println(DriverStationLCD.Line.kUser3, 1, "X-Force: " + enhancedIO.getAcceleration(DriverStationEnhancedIO.tAccelChannel.kAccelX));
                driverScreen.updateLCD();
                *
                */
             turn(70);
            }
        boolean[] driveButtons=joysticks.getLeftButtons();
        double[] instructions=joysticks.getMovementInstructions();
        teleopDrive(instructions,driveButtons);
        boolean[] pneumaticsButtons=joysticks.getRightButtons();
        teleopPneumatics(pneumaticsButtons);
              driverScreen.updateLCD();
        }

    }

    /**
     * Stops all operation in the driverTrain and pneumaticSystem.
     * TODO: Decide whether to writer safeStart() or make safeStop() timed for a specific amount of seconds.
     */
    private void safeStop()
    {
        robotDrive.stop();
        //pneumatics.stop();
    }

    //handles the driveTrain during teleop mode given the buttons relevant to driving
    //and the movement instructions
    private void teleopDrive(double[] instructions, boolean[] buttons){
        //sets overDrive to correspond to the trigger

        boolean overDrive = buttons[0];

        //gets raw speeds
        SpeedSet speedsNew = robotDrive.getSpeeds(instructions);

        //reduces speeds to correct range;
        speedsNew.reduce();

        //handles overDrive
        speedsNew.square();
       driverScreen.println(DriverStationLCD.Line.kUser4, 1,  speedsNew.toString());

        //limits acceleration
        speedsNew = robotDrive.accelerationLimit(speedsNew);
        driverScreen.println(DriverStationLCD.Line.kUser5, 1,  speedsNew.toString());


        //sets the processed motor speeds, casted to be a pair
        robotDrive.setSpeeds((SpeedPair)speedsNew);
    }

    public void teleopPneumatics(boolean[] buttons){
        //pneumatics.process(buttons);
    }

    //turns the robot based on parameter degrees.  Used for things like autoTarget
    //and autonomous.  Degrees is how much to the right the robot should turn, if
    //its faster to turn left then it turns left.
    public void turn(double degrees){
        //reduces degrees to its lowest equivalent measured angle
        degrees = degrees % 360;

        double startAngle = sensors.getGyroAngle();

        //will keep track of whether the turn is over and the robot should stop
        boolean done;
        boolean turningLeft;
        boolean turningRight;

        //if being asked to make more than a half turn right, turn left instead
        if (degrees > 180)
        {
            robotDrive.turn(-.5);
            turningRight = false;
            turningLeft = true;
            done=false;
        }

        //if being asked to make less than a half turn right but more than 0 degrees, turn right
        if(degrees <= 180 && degrees != 0)
        {
            robotDrive.turn(.5);
            turningRight = true;
            turningLeft = false;
            done = false;
        }

        //if being asked to turn 0 degrees, don't turn and set done to true.
        else{
            turningLeft = false;
            turningRight = false;
            done = true;
        }

        //loop that monitors the robot's angle until it is done turning
        while(!done){
             Watchdog.getInstance().feed();
            //if the robot is turning left and is farther left than its goal, set done to true
            if(turningLeft && ((sensors.getGyroAngle() - startAngle) % 360) <= degrees){
                done  = true;
            }

            //if the robot is turning right and is farther right than its goal, set done to true
            if(turningRight && ((sensors.getGyroAngle() - startAngle) % 360) >= degrees){
                done  = true;
            }
        }

        //at this point, done must be true, so stop turning
        robotDrive.stop();
    }
}



