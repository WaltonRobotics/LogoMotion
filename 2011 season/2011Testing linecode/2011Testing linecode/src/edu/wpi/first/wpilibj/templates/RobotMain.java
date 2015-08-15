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
import edu.wpi.first.wpilibj.Encoder;
import java.io.*;

/**
 * @author andrey
 * The RobotMain class instantiates and coordinates the rest of robot's classes.
 */
public class RobotMain extends IterativeRobot {

    private DriveTrain robotDrive;//a two motor drive
    private TankControls joysticks;//a tank drive control system for the robot
    private pneumaticSystem pneumatics;
    public SensorSet sensors;//Sensors currently set to public because there is not much that can be changed and access could be needed in other classes.
    private boolean demoMode;
    private DriverStationLCD driverScreen;//Object used to display information to the driver as needed.
    private DriverStationEnhancedIO enhancedIO = DriverStation.getInstance().getEnhancedIO();
    private LineTrackThread linetracking;
    private boolean armState, driveState;
    private boolean AUTO = true, TELEOP = false;
    private AutoFunction armTask, driveTask;
    private XboxControls xboxController;
    private Encoder encoder;
    private Arm arm;
    DigitalInput left, right, middle;
    Watchdog myDog;
    int displayIndex;
    OutputStream fileOutput;
    DataOutputStream dataOutput;



    /**
     * In accordance with the superClass, robotInit method is used for initialization rather than a constructor.
     */
    public void robotInit() {
        //File file = new File ("c://");
        displayIndex = 1;
        driverScreen = DriverStationLCD.getInstance();

        left = new DigitalInput(12);
        if (left == null) {
            printToScreen("LEFT SENSOR [DigitalInput(12)] IS NULL!!");
        }
        else{
             printToScreen("LEFT SENSOR [DigitalInput(12)] is initialized");
        }

        middle = new DigitalInput(13);
        if (middle == null) {
            printToScreen("MIDDLE SENSOR [DigitalInput(13)] IS NULL!!");
        }
        else {
             printToScreen("MIDDLE SENSOR [DigitalInput(13)] is initialized");
        }

        right = new DigitalInput(14);
        if (right == null){
          printToScreen("RIGHT SENSOR [DigitalInput(14)] IS NULL!!");
        }
        else{
            printToScreen("RIGHT SENSOR [DigitalInput(14)] INITIALIZED");
        }

        demoMode = false;
        myDog = Watchdog.getInstance();
        myDog.setEnabled(true);
        myDog.setExpiration(1);
        joysticks = new TankControls(1, 2);//All channel values arbitrary at this point and may need to be changed.
        // xboxController = new XboxControls(3);//channel value
        //initializes pneumatics
        //int[] solenoidChannels=(4,5);
        int spikeChannel = 1;
        int pressureSwitch = 2;
        //pneumatics=new pneumaticSystem(solenoidChannels,spikeChannel,pressureSwitch, 120);

        //Arm constructor
        arm= new Arm(3,4);
        //channel for gyro
        int gyroChannel = 1;

        //channels for camera initiation
        boolean cameraUsed = false;
        int baseCamChannel = -1;
        int panMotorCamChannel = 9;
        int servoCamChannel = 8;

        //channels for accelerators- may want multiple for multiple directions
        int accSlot = -1;

        encoder = new Encoder(5, 3, false);
        encoder.start();
        encoder.setDistancePerPulse(8 * 3.14 / 360);
        //initiates sensors
        sensors = new SensorSet(gyroChannel, accSlot, baseCamChannel, panMotorCamChannel, servoCamChannel);

        //initiates drive train
        robotDrive = new TwoMotorDrive(1, 2, demoMode);
        robotDrive.setInvertedSide(true);//boolean is true to invert right, false for left

        //so that it doesn't return nulls- should not be started before re-creating
        driveTask = new WaltLineTrack();
        armTask = new WaltLineTrack();
    }

    /**
     * The method that runs and controls the Robot's autonomic behavior. Most, if not all, of it will be written after the
     * game is announced.  it is called repeatedly, though not continuously, throughout the autonomous period
     */
    public void autonomousPeriodic() {
        myDog.feed();

    }

    //this method is called a single time, at the beginning of the autonomous period.
    public void autonomousInit() {

        //the channels for the light sensors- we will have to , or at least want to rewrite the
        //line tracker code so that it doesn't have to take the channels as paramters- we should
        //initiate the sensors at the beginnning of a match then never have to worry about them again.
        //SpeedPair forwardSpeedPair = new SpeedPair(-0.7,0.7);
        //robotDrive.setSpeeds(forwardSpeedPair);
        //must change this to make the run method of LineTracker take no parameters

    }

    /**
     * The method that allows control of the robot during the teleoperational period and displays information from the sensors on the screen.
     * This runs in an infinite loop that updates every .02 seconds as per last year for the entirety of the teleoperational period.
     */
    public void teleopInit() {
        driveState = TELEOP;
        armState = TELEOP;

    }

    public void teleopContinuous() {
        myDog.feed();
        //checks safestop button
        if (joysticks.getRightButton(10))//Buttons on top of right stick stops robot
        {
            safeStop();
            //delay to allow driver to release button
            Timer.delay(.1);
            myDog.feed();
            //at this point driver should have released the butto
            while (!joysticks.getRightTop()) {
                myDog.feed();
            }
            //now the button is pressed again, indicating a restart- the program can continue

            //slight delay to allow driver to release the button before the next loop
            Timer.delay(.1);
        }

        //if arm is in teleop mode, handles input accordingly
        if (armState == TELEOP) {
            handleArmInput();
        }

        //if arm is in auto mode, checks whether user is pressing button 1 to regain
        //control and stops the auto task if so
        if (armState == AUTO) {
            if (joysticks.getRightButton(4) || !armTask.isRunning()) {
                armTask.interrupt();
                armTask.stop();
                armState = TELEOP;
            }
        }

        //if drive is in teleop mode, handles input accordingly
        if (driveState == TELEOP) {
            try {
                handleDriveInput();
            } catch (EnhancedIOException e) {
            }

        }

        //if drive is in auto mode, checks whether user is trying to regain control
        if (driveState == AUTO) {
            if (joysticks.getRightButton(2) || !driveTask.isRunning()) {
                driveTask.interrupt();
                driveTask.stop();
                driveState = TELEOP;
            }
        }
    }

    public void teleopPeriodic() {
        driverScreen.updateLCD();
    }

    public void disabledInit() {
        System.out.println("Disabling robot and all threads.");
    }

    public void disabledPeriodic() {
    }

    /**
     * This method checks the input from the controllers and sends the appropriate input to the driver and pneumatics systems.
     */
    private void handleDriveInput() throws EnhancedIOException {
        {

            boolean[] driveButtons = joysticks.getLeftButtons();

            //double[] instructions=joysticks.getMovementInstructions();

            //comment this out if not using xbox and uncomment above inorder to get correct movement instructions
            //sets overDrive to correspond to the trigger

            //changed from using xBox to test encoders and line code
            double[] instructions = joysticks.getMovementInstructions();


            //Comment one or the other out to set how to control overdrive
            boolean overDrive = driveButtons[0];
            //boolean overDrive = xboxController.getButton(6);

            //testing linetracking and encoder
        /*if(driveButtons[2] || xboxController.getButton(7));
            {
            int channel2= 12;
            int channel3 = 13;
            int channel4 = 14;
            driveTask = new LineTrackThread(channel2, channel3, channel4,true,false);
            driveTask.start();
            }
             */

            //gets raw speeds
            SpeedSet speedsNew = robotDrive.getSpeeds(instructions);

            //reduces speeds to correct range;
            speedsNew.reduce();

            //handles overDrive
            speedsNew.square();

            speedsNew = robotDrive.handleOverDrive(speedsNew, overDrive);

            //limits acceleration
            speedsNew = robotDrive.accelerationLimit(speedsNew);

            //sets the processed motor speeds, casted to be a pair
            robotDrive.setSpeeds((SpeedPair) speedsNew);

            if (driveButtons[4]) {
                driveTask = new WaltLineTrack();
                driveTask.start();
                driveState = AUTO;

            }
        }
    }

    private void handleArmInput() {
        boolean[] armButtons = joysticks.getRightButtons();

        if(armButtons[2])
        {
            arm.forkliftUp(true);
        }
        else
        {
            arm.forkliftUp(false);
        }

        if(armButtons[1])
        {
            arm.forkliftDown(true);
        }
        else
        {
            arm.forkliftDown(false);
        }

        if(armButtons[3])
        {
            arm.armUp(true);
        }
        else
        {
            arm.armUp(false);
        }

        if(armButtons[4])
        {
            arm.armDown(true);
        }
        else
        {
            arm.armDown(false);
        }

    }

    /**
     * Stops all operation in the driverTrain and pneumaticSystem.
     * TODO: Decide whether to writer safeStart() or make safeStop() timed for a specific amount of seconds.
     */
    private void safeStop() {
        robotDrive.stop();
        //pneumatics.stop();
    }

    public void printToScreen(String output)
    {
        if(displayIndex==1)
        {
            driverScreen.println(DriverStationLCD.Line.kMain6,1,output);
        }
        else if(displayIndex == 2)
        {
            driverScreen.println(DriverStationLCD.Line.kUser2,1,output);
        }
        else if(displayIndex == 3)
        {
            driverScreen.println(DriverStationLCD.Line.kUser3,1,output);
        }
        else if(displayIndex == 4)
        {
            driverScreen.println(DriverStationLCD.Line.kUser4,1,output);
        }
        else if(displayIndex == 5)
        {
            driverScreen.println(DriverStationLCD.Line.kUser5,1,output);
        }
        else
        {
            driverScreen.println(DriverStationLCD.Line.kUser6,1,output);
        }


        if(displayIndex==6)
        {
            displayIndex=1;
        }
        else
        {
            displayIndex++;
        }
    }

    /*an abstract class to represent an autonomous function of our robot
     *Any class extending this one should use running as a condition for all of its loops
     *or potential "stuck" points.
     * For example, say the robot is trying to follow a line, and some boolean variable
     * done is used to check whether the robot is at the end of the line.
     *  Then, the basic loop should have "while(!done && running) so that it will terminate in the
     * case that EITHER it reaches the end of the line OR that the robot is telling it to stop.
     */
    //additional note: We need to discuss whether we really want to nest all this in robotMain
    // or whether it would be better to have them as separate classes taking robotMain or at least
    //systems of robotMain as paramters- i know we don't want to couple, but it is kind of awkward to
    //nest EVERY autonomous thread we want within the RobotMain class, and i don't honestly think it would be an issue
    public abstract class AutoFunction extends Thread {

        protected boolean running;

        public void stop() {
            running = false;
        }

        public boolean isRunning(){
            return running;
        }
    }

    //turns the robot based on parameter degrees.  Used for things like autoTarget
    //and autonomous.  Degrees is how much to the right the robot should turn, if
    //its faster to turn left then it turns left.
    private class TurnThread extends AutoFunction {

        private int degrees;

        //initiates a turn thread that will be used to turn deg (parameter) degrees to the right
        public TurnThread(int deg) {
            //reduces degrees to its lowest equivalent measured angle
            degrees = degrees % 360;
            running = false;
        }

        public void run() {
            double startAngle = sensors.getGyroAngle();
            running = true;

            //will keep track of whether the turn is over and the robot should stop
            boolean done;
            boolean turningLeft;
            boolean turningRight;

            //if being asked to make more than a half turn right, turn left instead
            if (degrees > 180) {
                robotDrive.turn(-.5);
                turningRight = false;
                turningLeft = true;
                done = false;
            }

            //if being asked to make less than a half turn right but more than 0 degrees, turn right
            if (degrees <= 180 && degrees != 0) {
                robotDrive.turn(.5);
                turningRight = true;
                turningLeft = false;
                done = false;
            } //if being asked to turn 0 degrees, don't turn and set done to true.
            else {
                turningLeft = false;
                turningRight = false;
                done = true;
            }

            //loop that monitors the robot's angle until it is done turning
            while (!done && running) {
                //if the robot is turning left and is farther left than its goal, set done to true
                if (turningLeft && ((sensors.getGyroAngle() - startAngle) % 360) <= degrees) {
                    done = true;
                }

                //if the robot is turning right and is farther right than its goal, set done to true
                if (turningRight && ((sensors.getGyroAngle() - startAngle) % 360) >= degrees) {
                    done = true;
                }
            }

            //at this point, done must be true, so stop turning
            robotDrive.stop();
            running = false;
        }
    }

    private class WaltLineTrack extends AutoFunction {
        boolean straight;

        WaltLineTrack()//(changed for consistency with other constructors)
        {
            running = false;
        }

        public void run()
        {
            running = true;
            int last=0;
            int leftValue=1;
            int midValue=2;
            int rightValue=3;
            int allOnCount=0;
            double speed=0.0;
            double turn=0.0;
            double[] instructions = {0.0,0.0,0.0};
            boolean done = false;
            printToScreen("Line Tracking now");
            SpeedSet speeds;

            while (!done && running)
            {
               myDog.feed();
               if(left.get())
               {
                   last=leftValue;
                   allOnCount++;
               }
               if(middle.get())
               {
                   last=midValue;
                   allOnCount++;
               }
               if(right.get())
               {
                   last=rightValue;
                   allOnCount++;
               }
               printToScreen("Successfully got input from sensors");

              if(allOnCount==3)
              {
                robotDrive.stop();
                printToScreen("Robot stopping");
                running=false;
                done=true;
                break;
              }

              if(allOnCount==0)
              {
                if(last==leftValue)
                {
                    speed=.4;
                    turn=.1;
                    instructions[0] = speed;
                    instructions[1]=0.0;
                    instructions[2]=turn;
                    speeds = robotDrive.getSpeeds(instructions);
                    speeds.multiplyBy(-1);
                    robotDrive.setSpeeds(speeds);
                }
                if(last==midValue)
                {
                    speed=.5;
                    turn=0.0;
                    instructions[0] = speed;
                    instructions[1]=0.0;
                    instructions[2]=turn;
                    speeds = robotDrive.getSpeeds(instructions);
                    speeds.multiplyBy(-1);
                    robotDrive.setSpeeds(speeds);
                }
                if(last==rightValue)
                {
                    speed=-.4;
                    turn=.1;
                    instructions[0] = speed;
                    instructions[1]=0.0;
                    instructions[2]=turn;
                    speeds = robotDrive.getSpeeds(instructions);
                    speeds.multiplyBy(-1);
                    robotDrive.setSpeeds(speeds);
                }
              }

                if(last==midValue)
                {
                    speed=.5;
                    turn=0.0;
                    instructions[0] = speed;
                    instructions[1]=0.0;
                    instructions[2]=turn;
                    speeds = robotDrive.getSpeeds(instructions);
                    speeds.multiplyBy(-1);
                    robotDrive.setSpeeds(speeds);
                }
               Timer.delay(.5);
            }

        }
    }

    private class LineTrackThread extends AutoFunction {
        double defaultSteeringGain = 0.65;
        boolean straight;
        boolean turnLeft;

        LineTrackThread( boolean s, boolean l)//(changed for consistency with other constructors)
        {
            straight = s;
            turnLeft = l;
            running = false;
        }

        public void run() {
            printToScreen("Start of line tracking.");

            int binaryValue; // a single binary value of the three line tracking
            // sensors
            int previousValue = 0; // the binary value from the previous loop
            double steeringGain; // the amount of steering correction to apply
            running = true;
            // the power profiles for the straight and forked robot path. They are
            // different to let the robot drive more slowly as the robot approaches
            // the fork on the forked line case.
          

            //These are the original speed values. Mine are test values for build site
            //double forkProfile[] = {0.70, 0.70, 0.55, 0.60, 0.60, 0.50, 0.40, 0.00};
            //double straightProfile[] = {0.7, 0.7, 0.6, 0.6, 0.35, 0.35, 0.35, 0.0};
            //the ratio for changes is 0.4804
            double forkProfile[] = {0.70, 0.70, 0.55, 0.00};
            double straightProfile[] = {0.7, 0.7, 0.6, 0.00};
            double powerProfile[];   // the selected power profile

            // set the straightLine and left-right variables depending on chosen path
            //boolean straight set int autonomous
            powerProfile = (straight) ? straightProfile : forkProfile;
            double stopTime = (straight) ? .8766 : 1.7532; // when the robot should look for end
            boolean goLeft = turnLeft && !straight;


            boolean atCross = false; // if robot has arrived at end

            // time the path over the line
            Timer timer = new Timer();
            timer.start();
            timer.reset();

            int oldTimeInSeconds = -1;
            double time;
            double speed, turn;

            // loop until robot reaches "T" at end or 8 seconds has past
            //while ((time = timer.get()) < 8.0 && !atCross) {
            
            //Same as above only test times and lenghts are different
            while (!atCross && running) {
                myDog.feed();
                time = timer.get();
                int timeInSeconds = (int) time;
                // read the sensors
                int leftValue = left.get() ? 1 : 0;
                int middleValue = middle.get() ? 1 : 0;
                int rightValue = right.get() ? 1 : 0;
                // compute the single value from the 3 sensors. Notice that the bits
                // for the outside sensors are flipped depending on left or right
                // fork. Also the sign of the steering direction is different for left/right.
                if (goLeft) {
                    binaryValue = leftValue * 4 + middleValue * 2 + rightValue;
                    steeringGain = -defaultSteeringGain;
                } else {
                    binaryValue = rightValue * 4 + middleValue * 2 + leftValue;
                    steeringGain = defaultSteeringGain;
                }

                //testing bad
                steeringGain*=-1;

                // Change speed to encorder algorithim
                speed = powerProfile[timeInSeconds];
                turn = 0;

                // different cases for different line tracking sensor readings
                switch (binaryValue) {
                    case 1:  // on line edge
                        turn = 0;
                        break;
                    case 7:  // all sensors on (maybe at cross)
                        if (time > stopTime) {
                            atCross = true;
                            speed = 0;
                        }
                        break;
                    case 0:  // all sensors off
                        if (previousValue == 0 || previousValue == 1) {
                            turn = steeringGain;
                        } else {
                            turn = -steeringGain;
                        }
                        //break;
                    default:  // all other cases
                        turn = -steeringGain;
                }
                // print current status for debugging
                if (binaryValue != previousValue) {
                    printToScreen("Time: " + time + " Sensor: " + binaryValue + " speed: " + speed + " turn: " + turn + " atCross: " + atCross);
                }
                
                // set the robot speed and direction
                //drive.arcadeDrive(speed, turn);
                double[] instructions = {speed, 0, turn};
                SpeedSet speeds = robotDrive.getSpeeds(instructions);
                speeds.multiplyBy(-1);
                robotDrive.setSpeeds(speeds);

                if (binaryValue != 0) {
                    previousValue = binaryValue;
                }
                oldTimeInSeconds = timeInSeconds;

                Timer.delay(0.01);

            }
            running=false;
            robotDrive.stop();
            // Done with loop - stop the robot. Robot ought to be at the end of the line
            printToScreen("End of line tracking.");
        }
    }
}
