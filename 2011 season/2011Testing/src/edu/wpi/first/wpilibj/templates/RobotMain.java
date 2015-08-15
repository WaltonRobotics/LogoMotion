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
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import java.io.*;

/**
 * @author andrey
 * The RobotMain class instantiates and coordinates the rest of robot's classes.
 */
public class RobotMain extends IterativeRobot {

    Compressor compressor;
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
    private AutoFunction driveTask;
    private Arm.AutoFunction armTask;
    private XboxControls xboxController;
    private Encoder encoder;
    private Arm arm;
    private CameraMount cameraMount;
    DigitalInput left, right, middle;
    Watchdog myDog;
    int displayIndex;
    OutputStream fileOutput;
    DataOutputStream dataOutput;
    Joystick armJoystick, liftJoystick;

    /**
     * In accordance with the superClass, robotInit method is used for initialization rather than a constructor.
     */
    public void robotInit() {
        System.out.println("robot_init");
        liftJoystick = new Joystick(2);
        armJoystick = new Joystick(4);
        encoder = new Encoder(2, 3);
        encoder.setDistancePerPulse(8.0 * 3.14 / 360.0 / 2);
        compressor = new Compressor(1, 1);
        compressor.start();
        cameraMount = new CameraMount(10,9);
        //File file = new File ("c://");
        displayIndex = 1;
        driverScreen = DriverStationLCD.getInstance();

        left = new DigitalInput(12);
        if (left == null) {
            printToScreen(1, "LEFT SENSOR [DigitalInput(12)] IS NULL!!");
        } else {
            printToScreen(2, "LEFT SENSOR [DigitalInput(12)] is initialized");
        }

        middle = new DigitalInput(13);
        if (middle == null) {
            printToScreen(3, "MIDDLE SENSOR [DigitalInput(13)] IS NULL!!");
        } else {
            printToScreen(4, "MIDDLE SENSOR [DigitalInput(13)] is initialized");
        }

        right = new DigitalInput(14);
        if (right == null) {
            printToScreen(5, "RIGHT SENSOR [DigitalInput(14)] IS NULL!!");
        } else {
            printToScreen(6, "RIGHT SENSOR [DigitalInput(14)] INITIALIZED");
        }

        demoMode = false;
        myDog = Watchdog.getInstance();
        myDog.setEnabled(true);
        myDog.setExpiration(1);
        joysticks = new TankControls(1, 3);//All channel values arbitrary at this point and may need to be changed.
        // xboxController = new XboxControls(3);//channel value
        //initializes pneumatics
        //int[] solenoidChannels=(4,5);
        int spikeChannel = 1;
        int pressureSwitch = 2;
        //pneumatics=new pneumaticSystem(solenoidChannels,spikeChannel,pressureSwitch, 120);

        //Arm constructor
        //currently the arm is controlling the drive motors- arm channels are 3,4
        int liftChannel = 3;
        int armChannel = 4;
        int solenoidChannelWrist = 1;
        int clawSolenoid = 2;
        int armGyroChannel = 2;
        arm = new Arm(liftChannel, armChannel, solenoidChannelWrist, clawSolenoid, armGyroChannel);
        //channel for gyro
        int gyroChannel = 1;

        //channels for camera initiation
        boolean usingCamera = true;
        int panMotorCamChannel = 9;
        int tiltCamChannel = 8;

        //channels for accelerators- may want multiple for multiple directions
        int accSlot = -1;

        sensors = new SensorSet(gyroChannel, accSlot, usingCamera, tiltCamChannel, panMotorCamChannel);

        //initiates drive train. Currently the drive is controlling the arm jags. Drive jags are 1,2
        robotDrive = new TwoMotorDrive(1, 2, demoMode);
        robotDrive.setInvertedSide(true);//boolean is true to invert right, false for left

        //so that it doesn't return nulls- should not be started before re-creating
        driveTask = new WaltLineTrack(false, false);
        armTask = arm.new GoToHeight(4);
    }

    /**
     * The method that runs and controls the Robot's autonomic behavior. Most, if not all, of it will be written after the
     * game is announced.  it is called repeatedly, though not continuously, throughout the autonomous period
     */
    public void autonomousPeriodic() {
        myDog.feed();
    }

    //this method is called a single time, at the beginning of the autonomous period.
    public void autoScoreBottomCenterPeg(){
        robotDrive.goForward(-.5); //Speed at which to back up
        Timer.delay(1); //Time to continue backing up
        robotDrive.stop(); //Stop backing up
        arm.armJag.set(.4); //Speed to lower arm *may need to be inverted*
        Timer.delay(.5); //Time to lower arm
        arm.grabTube(true); //Opens the claw to release Ub3r Tube
        robotDrive.goForward(-.5); //Speed at which to back up again
        arm.armJag.set(.3); //Lowers arm safely to ground.

    }
    public void autonomousInit() {

        //Hard Coded Testing


        boolean willTrack;//keeps track of whether the robot should do autonomous or not

        //if the robot is placed with the middle sensor on the line, go straight
        if(!middle.get()){
            driveTask = new WaltLineTrack(true,false);
            willTrack = true;
        }

        //if the robot is placed with the left sensor on the line, it will
        //follow the line to the left
        else if(!left.get()){
            driveTask = new WaltLineTrack(false,true);
            willTrack = true;
        }

        //if the robot is placed with the right sensor on the line, it will
        //follow the line to the right
        else if(!right.get()){
            driveTask = new WaltLineTrack(false,false);
            willTrack = true;
        }

        else{
            willTrack = false;
        }

        //only do autonomous if willTrack is true
        if(willTrack){
        //Method for scoring on bottom peg
        autoScoreBottomCenterPeg();
        }





         /*
         //A draft of autnomous code:

         boolean willTrack;//keeps track of whether the robot should do autonomous or not

        //if the robot is placed with the middle sensor on the line, go straight
        if(!middle.get()){
            driveTask = new WaltLineTrack(true,false);
            willTrack = true;
        }
        
        //if the robot is placed with the left sensor on the line, it will
        //follow the line to the left
        else if(!left.get()){
            driveTask = new WaltLineTrack(false,true);
            willTrack = true;
        }
        
        //if the robot is placed with the right sensor on the line, it will
        //follow the line to the right
        else if(!right.get()){
            driveTask = new WaltLineTrack(false,false);
            willTrack = true;
        }
        
        else{
            willTrack = false;
        }

        //only do autonomous if willTrack is true
        if(willTrack){

        //create and begin the drive task to follow the right fork
        driveTask = new WaltLineTrack(false,false);
        driveTask.begin();

        //wait for line tracking to finish
        while(driveTask.isRunning()){
            Timer.delay(.1);
        }

        //make arm go to height 3
        armTask = arm.new GoToHeight(3);
        armTask.begin();

        //waits for forklift to be at the correct height
        while(!armTask.atCorrectHeight()){
            Timer.delay(.1);
        }

        //drops wrist
         arm.wristDown();
    
        //lowers arm to place tube on peg
         arm.lowerArm();
        }
        */
    }

    /**
     * The method that allows control of the robot during the teleoperational period and displays information from the sensors on the screen.
     * This runs in an infinite loop that updates every .02 seconds as per last year for the entirety of the teleoperational period.
     */
    public void teleopInit() {
        driveState = TELEOP;
        armState = TELEOP;

    }

    //this method is called repeatedly during the teleop period, as fast as it can be called
    public void teleopContinuous() {
        //feed the watchdog
        myDog.feed();

        //check safestop button
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
            //if the driver is attempting to cancel the task OR the task is done, return control
            //to the driver
            if (armJoystick.getRawButton(7) || !armTask.isRunning()) {
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
            //if the driver is attempting to cancel the task OR the task is done, return control
            //to the driver
            if (joysticks.getLeftButton(1) || !driveTask.isRunning()) {
                driveTask.interrupt();
                driveTask.stop();
                driveState = TELEOP;
            }
        }

        printToScreen(3, "Gyro Angle: " + arm.gyro.getAngle());
        printToScreen(4, "Arm Angle: " + arm.getArmAngle());
    }

    //this method is called frequently during the teleop period, though with delay
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

        //if button 5 is pressed on the joystick, start line tracking
        if (driveButtons[4]) {
            //first create the thread
            driveTask = new WaltLineTrack(false, false);

            //then start it WITH THE BEGIN method so that it is immediately set to running = true
            //(begin is IMPORTANT!)
            driveTask.begin();

            //tell the driveTrain it is under in automatic control
            driveState = AUTO;
        }
    }

    private void handleArmInput() {

        //the claw always stays closed unless the trigger is held on the arm joystick
        if (armJoystick.getRawButton(1)) {
            arm.grabTube(true); 
        } else {
            arm.grabTube(false);
        }


        //button 5 raises the wrist, button 4 lowers it
        if (armJoystick.getRawButton(5)) {
            arm.wristUp();
        } else if (armJoystick.getRawButton(4)) {
            arm.wristDown();
        }

        //attempts to correct for predictable gyro drift
        arm.gyroCorrect();

       //button two resets the gyro.  This only works if the arm is vertical
        if(liftJoystick.getRawButton(2)){;
            arm.resetArmAngle();
        }

//        if (liftJoystick.getRawButton(6)) {
//
//            armTask = arm.new GoToHeight(2);
//            armTask.begin();
//            armState = AUTO;
//        }

        //checks if the wrist should auto flip- it flips if it passes through the "critical" zone
        arm.wristFlipCheck();

        printToScreen(5, "Forklift moved " + arm.armEncoder.getDistance());

        /* if(armJoystick.getRawButton(8))
        {
        arm.forkliftControl(-armJoystick.getAxis(Joystick.AxisType.kZ));
        }
        else  if (armJoystick.getRawButton(9))
        {
         *
         */
        arm.forkliftControl(liftJoystick.getAxis(Joystick.AxisType.kY));
        arm.armControl(-armJoystick.getAxis(Joystick.AxisType.kY));

//        if (liftJoystick.getRawButton(3)) {
//            arm.heightenArm();
//        } else if (liftJoystick.getRawButton(2)) {
//            arm.lowerArm(); //0.4 seemed like a good number
//        } else {
//            arm.oscillate(); //set somethign to avoid gravy
//            //arm.armJag.set(0);
//        }
    }

    /**
     * Stops all operation in the driverTrain and pneumaticSystem.
     * TODO: Decide whether to writer safeStart() or make safeStop() timed for a specific amount of seconds.
     */
    private void safeStop() {
        robotDrive.stop();
        //pneumatics.stop();
    }

    public void printToScreen(String output) {
        if (displayIndex == 1) {
            driverScreen.println(DriverStationLCD.Line.kMain6, 1, output);
        } else if (displayIndex == 2) {
            driverScreen.println(DriverStationLCD.Line.kUser2, 1, output);
        } else if (displayIndex == 3) {
            driverScreen.println(DriverStationLCD.Line.kUser3, 1, output);
        } else if (displayIndex == 4) {
            driverScreen.println(DriverStationLCD.Line.kUser4, 1, output);
        } else if (displayIndex == 5) {
            driverScreen.println(DriverStationLCD.Line.kUser5, 1, output);
        } else {
            driverScreen.println(DriverStationLCD.Line.kUser6, 1, output);
        }


        if (displayIndex == 6) {
            displayIndex = 1;
        } else {
            displayIndex++;
        }
    }

    public void printToScreen(int line, String output) {
        if (line == 1) {
            driverScreen.println(DriverStationLCD.Line.kMain6, 1, output);
        } else if (line == 2) {
            driverScreen.println(DriverStationLCD.Line.kUser2, 1, output);
        } else if (line == 3) {
            driverScreen.println(DriverStationLCD.Line.kUser3, 1, output);
        } else if (line == 4) {
            driverScreen.println(DriverStationLCD.Line.kUser4, 1, output);
        } else if (line == 5) {
            driverScreen.println(DriverStationLCD.Line.kUser5, 1, output);
        } else {
            driverScreen.println(DriverStationLCD.Line.kUser6, 1, output);
        }

    }

    /*an abstract class to represent an autonomous function of our robot
     *Any class extending this one should use running as a condition for all of its loops
     *or potential "stuck" points.
     * For example, say the robot is trying to follow a line, and some boolean variable
     * done is used to check whether the robot is at the end of the line.
     *  Then, the basic loop should have "while(!done && running) so that it will terminate in the
     * case that EITHER it reaches the end of the line OR that the robot is telling it to stop.
     * Additionally, all such functions should set running = false; at the end of the run() method
     * and should ONLY be called with the begin() method, not the start() method.
     */
    /*additional note: We need to discuss whether we really want to nest all this in robotMain
    systems of robotMain as paramters- i know we don't want to couple, but it is kind of awkward to
    nest EVERY autonomous thread we want within the RobotMain class
     */
    public abstract class AutoFunction extends Thread {

        public boolean running;

        //sets running to be false, which for a validly written AutoFUnction will stop the thread
        public void stop() {
            running = false;
        }

        //checks whether the thread is currently running
        public boolean isRunning() {
            return running;
        }

        //sets running = true and starts the thread if not already started
        public void begin() {
            try {
                running = true;
                this.start();
            } catch (RuntimeException rte) {
            }
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
            printToScreen(4, "Angle: " + startAngle);
            //will keep track of whether the turn is over and the robot should stop
            boolean done=false;
            boolean turningLeft=false;
            boolean turningRight=false;

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

        boolean straightPath;
        boolean leftPath;

        WaltLineTrack(boolean s, boolean l)//(changed for consistency with other constructors)
        {
            //Couldn't this take 1 parameter, and then set leftPath=!straightPath ? ~Paul
            straightPath = s;
            leftPath = l; //meaning fork
            running = false;
        }

        public void run() {
            //intializes the last value which is used to take the last known sensor that was on the line
            int last = 0;
            //the values that will be passed to last if the corresponding snesor is triggered
            int leftValue = 1;
            int midValue = 2;
            int rightValue = 3;
            //counts how many sensors are triggered at one time
            int allOnCount = 0;

            //stops the encoders if they were activated before hand 
            encoder.stop();
            //resets the distance traveled
            encoder.reset();
            //starts them
            encoder.start();

            //distance from start of line to fork IN INCHES
            final double DISTANCE_TO_FORK = 162.4;
            //distance from beginning of the fork to when it straightens back out IN INCHES
            final double DISTANCE_LEG = 64.0;
            //final stretch to T IN INCHES
            final double FINAL_DISTANCE = 12.0;
            //forward speed IN JAG INPUT (-1 to 1)
            double speed = 0.0;
            //turn speed IN JAG INPUT (-1 to 1)
            double turn = 0.0;
            //speed values sent to robotDrive
            //Param: Speed amt forward, speed sideways (not used), speed Turn
            double[] instructions = {0.0, 0.0, 0.0};
            //if line tracking has finished or not
            boolean done = false;

            printToScreen(6, "Line Tracking now");
            SpeedSet speeds;

            //booleans that track whether a the different distances have been reached yet
            boolean forkNotReached = true;
            boolean secondTurnNotReached = true;
            boolean finalNotReached = true;

            //sets turns based on which path in the fork the robot is supposed to travel
            TurnThread turnFork;
            TurnThread secondTurn;
            if (leftPath) {
                turnFork = new TurnThread(-30);
                secondTurn = new TurnThread(30);
            } else {
                turnFork = new TurnThread(30);
                secondTurn = new TurnThread(-30);
            }
            //start of loop that handles line tracking
            while (!done && running) {
                myDog.feed();
                printToScreen("Moved " + encoder.getDistance() + " inches");
                //Determines which path the robot is following
                if (straightPath) {
                    //need to code straight path code, shouldn't take long
                } else {
                    //checks sensors to see if they are on lines. Will need to get rid of "!"
                    // at competition if tape is darker than floor. allOnCount is added to if a sensor is on
                    // and last is assigned the value of the last sensor triggered.
                    if (!middle.get()) {
                        last = midValue;
                        allOnCount++;
                    }
                    if (!left.get()) {
                        last = leftValue;
                        allOnCount++;
                    }
                    if (!right.get()) {
                        last = rightValue;
                        allOnCount++;
                    }

                    //sets the needed speeds depending on which sensor is triggered
                    if (last == midValue) {
                        speed = .25;
                        turn = 0.0;

                    } else if (last == rightValue) {
                        speed = 0.25;
                        turn = .1;
                    } else if (last == leftValue){
                        speed = 0.25;
                        turn = -.1;
                    }
                    else
                    {

                    }

                    //if the fork hasn't been reached yet, robot does a check whether
                    //the encoders have picked enough distance to turn
                    if (forkNotReached) {
                        //checks whether the distance traveled by the robot is bigger
                        // or equal to the distance needed to reach the fork
                        if (DISTANCE_TO_FORK <= encoder.getDistance()) {
                            //if the fork is reached turns the boolean to false
                            forkNotReached = false;
                            
                            //stops and waits 1 second- need to use sleep rather than delay so that
                            //other threads can keep going, like feeding watchdog in teleop continuous
                            robotDrive.stop();
                            try{
                                sleep(1000);
                            }
                            catch(InterruptedException ie){

                            }
                            //the turn at the fork is executed and line track code is
                            //delayed while the turn is executed
                            turnFork.begin();
                            while (turnFork.isRunning()) {
                                Timer.delay(.1);
                            }

                            //stops and waits 1 second- need to use sleep rather than delay so that
                            //other threads can keep going, like feeding watchdog in teleop continuous
                            robotDrive.stop();
                            try{
                                sleep(1000);
                            }
                            catch(InterruptedException ie){

                            }
                            //the distance recorded by the encoder is reset so the
                            //robot can begin measuring the distance to the next turn
                            encoder.reset();
                            //Robot forced to move straight after turn regardless
                            //of what sensor was triggered before the turn
                            speed = .25;
                            turn = 0.0;
                        }
                    } //checks whether the distance traveled by the robot is bigger
                    // or equal to the distance needed to reach the second turn
                    else if (secondTurnNotReached) {
                        //checks whether the distance traveled by the robot is bigger
                        // or equal to the distance needed to reach the second turn
                        if (DISTANCE_LEG <= encoder.getDistance()) {
                            //if the next turn is reached turns the boolean to false
                            secondTurnNotReached = false;
                            //the last turn is executed and line track code is
                            //delayed while the turn is executed

                           //stops and waits 1 second- need to use sleep rather than delay so that
                            //other threads can keep going, like feeding watchdog in teleop continuous
                            robotDrive.stop();
                            try{
                                sleep(1000);
                            }
                            catch(InterruptedException ie){

                            }

                            secondTurn.begin();
                            //the turn at the fork is executed and line track code is
                            //delayed while the turn is executed

                            while (secondTurn.isRunning()) {
                                Timer.delay(.1);
                            }

                           //stops and waits 1 second- need to use sleep rather than delay so that
                            //other threads can keep going, like feeding watchdog in teleop continuous
                            robotDrive.stop();
                            try{
                                sleep(1000);
                            }
                            catch(InterruptedException ie){

                            }
                            
                            //the distance recorded by the encoder is reset so the
                            //robot can begin measuring the distance to the next turn
                            encoder.reset();
                            //Robot forced to move straight after turn regardless
                            //of what sensor was triggered before the turn
                            speed = .25;
                            turn = 0.0;
                        }
                    } //checks whther the T has been reached
                    else if (finalNotReached) {
                        if (allOnCount == 3) {
                            robotDrive.stop();
                            printToScreen(6, "Robot stopping");
                            running = false;
                            done = true;
                            return;
                        }
                    }
                    printToScreen(6, "speed: " + speed + " Turn:" + turn);
                    instructions[0] = -speed;
                    instructions[1] = 0.0;
                    instructions[2] = -turn;
                    speeds = robotDrive.getSpeeds(instructions);
                    speeds = robotDrive.accelerationLimit(speeds);
                    robotDrive.setSpeeds(speeds);
                }
            }
            /*int last = 2;
            int leftValue = 1;
            int midValue = 2;
            int rightValue = 3;
            int allOnCount = 0;
            double speed = 0.0;
            double turn = 0.0;
            double[] instructions = {0.0, 0.0, 0.0};
            boolean done = false;
            printToScreen(6,"Line Tracking now");
            SpeedSet speeds;
             *
             */

            //while (!done && running) {
                /*myDog.feed();
            //gets input from sensors and assigns the last triggered to last
            //allOnCount keeps track of number of sensors triggered
            if (!left.get()) {
            last = leftValue;
            allOnCount++;
            }
            if (!middle.get()) {
            last = midValue;
            allOnCount++;
            }
            if (!right.get()) {
            last = rightValue;
            allOnCount++;
            }
            printToScreen(5,"Successfully got input from sensors");
            printToScreen(4,"-->SENSORS TRIGGERED = " + allOnCount
            + "\n-->last was " + last);
            //if all are on T robot is stopped
            if (allOnCount == 3) {
            robotDrive.stop();
            printToScreen(6,"Robot stopping");
            running = false;
            done = true;
            break;
            }
            else if(allOnCount==2)
            {
            if(straightPath)
            {
            robotDrive.stop();
            printToScreen(6,"Robot stopping");
            running = false;
            done = true;
            break;
            }
            else if(leftPath)
            {
            speed = 0.0;
            turn = -.15;
            }
            else
            {
            speed = 0.0;
            turn = .15;
            }
            }
            else {
            //if all are not on T then it takes the last sensor triggered and moves so robot becomes centered
            if (last == leftValue) {
            speed = 0.25;
            turn = -.1;
            }
            if (last == midValue) {
            speed = .25;
            turn = 0.0;
            }
            if (last == rightValue) {
            speed = 0.25;
            turn = .1;
            }

            }
            instructions[0] = -speed;
            instructions[1] = 0.0;
            instructions[2] = turn;
            speeds = robotDrive.getSpeeds(instructions);
            speeds = robotDrive.accelerationLimit(speeds);
            robotDrive.setSpeeds(speeds);
            printToScreen(3,"Motors set to " + (-speed - turn) + ", " + (-speed + turn));
            allOnCount = 0;
            //Timer.delay(.5);
             *
             */
        }
    }

    private class LineTrackThread extends AutoFunction {

        double defaultSteeringGain = 0.65;
        boolean straight;
        boolean turnLeft;

        LineTrackThread(boolean s, boolean l)//(changed for consistency with other constructors)
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
                steeringGain *= -1;

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
                robotDrive.accelerationLimit(speeds);
                robotDrive.setSpeeds(speeds);

                if (binaryValue != 0) {
                    previousValue = binaryValue;
                }
                oldTimeInSeconds = timeInSeconds;

                Timer.delay(0.01);
            }
            running = false;
            robotDrive.stop();
            // Done with loop - stop the robot. Robot ought to be at the end of the line
            printToScreen("End of line tracking.");
        }
    }
}
