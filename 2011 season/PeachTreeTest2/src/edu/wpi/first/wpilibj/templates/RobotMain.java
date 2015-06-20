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
import edu.wpi.first.wpilibj.templates.Arm.*;
import java.io.*;

/**
 * @author andrey
 * The RobotMain class instantiates and coordinates the rest of robot's classes.
 */
public class RobotMain extends IterativeRobot
{

    Compressor compressor;
    private DriveTrain robotDrive;//a two motor drive
    private TankControls joysticks;//a tank drive control system for the robot
    private pneumaticSystem pneumatics;
    private SensorSet sensors;//Sensors currently set to public because there is not much that can be changed and access could be needed in other classes.
    private boolean demoMode;
    private DriverStationLCD driverScreen;//Object used to display information to the driver as needed.
    private DriverStationEnhancedIO enhancedIO = DriverStation.getInstance().getEnhancedIO();
    private boolean armState, driveState;
    private boolean AUTO = true, TELEOP = false;
    private AutoFunction driveTask;
    private AutoFunction armTask, armTask2;
    //private XboxControls xboxController;
    private Encoder encoder;
    private Arm arm;
    private CameraMount cameraMount;
    private DigitalInput left;
    private DigitalInput right;
    private DigitalInput middle;
    Watchdog myDog;
    int displayIndex;
    OutputStream fileOutput;
    DataOutputStream dataOutput;
    Joystick armJoystick, liftJoystick;

    /**
     * In accordance with the superClass, robotInit method is used for initialization rather than a constructor.
     */
    public void robotInit() {
        ;
        System.out.println("robot_init");
        liftJoystick = new Joystick(2);
        armJoystick = new Joystick(4);
        setEncoder(new Encoder(2, 3));
        getEncoder().setDistancePerPulse(8.0 * 3.14 / 360.0 / 2);
        compressor = new Compressor(1, 1);
        compressor.start();
        // cameraMount = new CameraMount(10,9);
        //File file = new File ("WALT_output.txt");
        displayIndex = 1;
        driverScreen = DriverStationLCD.getInstance();

        //sensor wiring was switche so I fixed it programming wise
        setLeft(new DigitalInput(12));
        if (getLeft() == null) {
            printToScreen("LEFT SENSOR [DigitalInput(12)] IS NULL!!");
        } else {
            printToScreen("LEFT SENSOR [DigitalInput(12)] is initialized");
        }

        middle = new DigitalInput(13);
        if (getMiddle() == null) {
            printToScreen("MIDDLE SENSOR [DigitalInput(13)] IS NULL!!");
        } else {
            printToScreen("MIDDLE SENSOR [DigitalInput(13)] is initialized");
        }

        right = new DigitalInput(14);
        if (getRight() == null) {
            printToScreen("RIGHT SENSOR [DigitalInput(14)] IS NULL!!");
        } else {
            printToScreen("RIGHT SENSOR [DigitalInput(14)] INITIALIZED");
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
        arm = new Arm(liftChannel, armChannel, solenoidChannelWrist,
                clawSolenoid, armGyroChannel);
        //channel for gyro
        int gyroChannel = 1;

        //channels for camera initiation
        boolean usingCamera = true;
        int panMotorCamChannel = 9;
        int tiltCamChannel = 8;

        //channels for accelerators- may want multiple for multiple directions
        int accSlot = -1;

        setSensors(new SensorSet(gyroChannel, accSlot, usingCamera,
                tiltCamChannel, panMotorCamChannel));

        setRobotDrive(new TwoMotorDrive(1, 2, demoMode));
        getRobotDrive().setInvertedSide(true);//boolean is true to invert right, false for left

        //so that it doesn't return nulls- should not be started before re-creating Why do we initialize it here then?
        driveTask = new WaltLineTrack(false, false, this);
        armTask = arm.getNewGoToHeightInstance(2);
        armTask2 = arm.getNewGoToAngleInstance(135);
    }

    /**
     * The method that runs and controls the Robot's autonomic behavior. Most, if not all, of it will be written after the
     * game is announced.  it is called repeatedly, though not continuously, throughout the autonomous period
     */
    public void autonomousPeriodic() {
        myDog.feed();
    }

    //this method is called a single time, at the beginning of the autonomous period.
    public void autoScoreBottomCenterPeg() {
        getRobotDrive().goForward(-.5); //Speed at which to back up
        Timer.delay(1); //Time to continue backing up
        getRobotDrive().stop(); //Stop backing up
        arm.armJag.set(.4); //Speed to lower arm *may need to be inverted*
        Timer.delay(.5); //Time to lower arm
        arm.grabTube(true); //Opens the claw to release Ub3r Tube
        getRobotDrive().goForward(-.5); //Speed at which to back up again
        arm.armJag.set(.3); //Lowers arm safely to ground.

    }

    public void autonomousInit() {

        //Hard Coded Testing


        boolean willTrack;//keeps track of whether the robot should do autonomous or not

        //if the robot is placed with the middle sensor on the line, go straight
        if (!middle.get()) {
            driveTask = new WaltLineTrack(true, false, this);
            willTrack = true;
        } //if the robot is placed with the left sensor on the line, it will
        //follow the line to the left
        else if (!left.get()) {
            driveTask = new WaltLineTrack(false, true, this);
            willTrack = true;
        } //if the robot is placed with the right sensor on the line, it will
        //follow the line to the right
        else if (!right.get()) {
            driveTask = new WaltLineTrack(false, false, this);
            willTrack = true;
        } else {
            willTrack = false;
        }

        //only do autonomous if willTrack is true
       /*if(willTrack){

        driveTask.begin();

        while(driveTask.isRunning()){
        Timer.delay(.1);
        }
         */
        armTask.begin();
        Timer.delay(1);

        getRobotDrive().goForward(.3);

        getEncoder().reset();

        getEncoder().start();

        while (getEncoder().getDistance() < 6) {
            Timer.delay(.1);
            myDog.feed();
        }
        getRobotDrive().stop();

        arm.lowerArmSlow();
        Timer.delay(.5);
        arm.releaseTube();
        arm.wristDown();

        Timer.delay(.2);

        getRobotDrive().goForward(.3);

        getEncoder().reset();

        getEncoder().start();

        while (getEncoder().getDistance() < 36) {
            Timer.delay(.1);
            myDog.feed();
        }
        getRobotDrive().stop();




        //armTask2.begin();
        //}





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
        armTask.stop();
        driveTask.stop();
        armTask2.stop();
        getRobotDrive().stop();
        driveState = TELEOP;
        armState = TELEOP;

    }

    //this method is called repeatedly during the teleop period, as fast as it can be called
    public void teleopContinuous() {
        //feed the watchdog
        myDog.feed();

        //check safestop button
       /*safestop commented out
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
         */

        //if arm is in teleop mode, handles input accordingly
        if (armState == TELEOP) {
            handleArmInput();
        }

        //if arm is in auto mode, checks whether user is pressing button 1 to regain
        //control and stops the auto task if so

        /*
        arm never goes to auto mode, so this is commented out

        (armState == AUTO) {
        //if the driver is attempting to cancel the task OR the task is done, return control
        //to the driver
        if (armJoystick.getRawButton(7) || !armTask.isRunning()) {
        armTask.interrupt();
        armTask.stop();
        armState = TELEOP;
        }
        }
         */

        //if drive is in teleop mode, handles input accordingly
        if (driveState == TELEOP) {
            try {
                handleDriveInput();
            } catch (EnhancedIOException e) {
            }

        }

        //if drive is in auto mode, checks whether user is trying to regain control
       /*not used
        if (driveState == AUTO) {
        //if the driver is attempting to cancel the task OR the task is done, return control
        //to the driver
        if (joysticks.getLeftButton(1) || !driveTask.isRunning()) {
        driveTask.interrupt();
        driveTask.stop();
        driveState = TELEOP;
        }
        }
         */


        printToScreen("Gyro Angle: " + arm.gyro.getAngle());
        printToScreen("Arm Angle: " + arm.getArmAngle());
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
        SpeedSet speedsNew = getRobotDrive().getSpeeds(instructions);

        //reduces speeds to correct range;
        speedsNew.reduce();

        //handles overDrive
        speedsNew.square();

        speedsNew = getRobotDrive().handleOverDrive(speedsNew, overDrive);

        //limits acceleration
        speedsNew = getRobotDrive().accelerationLimit(speedsNew);

        //sets the processed motor speeds, casted to be a pair
        getRobotDrive().setSpeeds((SpeedPair) speedsNew);

        //if button 5 is pressed on the joystick, start line tracking

        /*commented out, we don't want line track during teleop
        if (driveButtons[4]) {
        //first create the thread
        driveTask = new WaltLineTrack(false, false);

        //then start it WITH THE BEGIN method so that it is immediately set to running = true
        //(begin is IMPORTANT!)
        driveTask.begin();

        //tell the driveTrain it is under in automatic control
        driveState = AUTO;
        }
         */
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
        if (liftJoystick.getRawButton(2)) {
            ;
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



        /* if(armJoystick.getRawButton(8))
        {
        arm.forkliftControl(-armJoystick.getAxis(Joystick.AxisType.kZ));
        }
        else  if (armJoystick.getRawButton(9))
        {
         *
         */


        arm.forkliftControl(liftJoystick.getAxis(Joystick.AxisType.kY));



        //START GILS CHANGES TO IMPLEMENT OSCILLATION
        double armSpeed = -armJoystick.getAxis(Joystick.AxisType.kY);

        //controls manually if its reading a non-negligible joystick input
        if (Math.abs(armSpeed) > .03) {
            arm.armControl(armSpeed);
        } //oscillates otherwise.  This causes it to maintain its angle as long as no significant
        //driver commands are given (since the joystick at rest is probably not exactly 0 we must account
        //for a range of some size being "insignificant." .03 was chosen arbitrarily
        else {
            arm.armOscillate();
        }
        //END GILS CHANGES TO IMPLEMENT OSCILLATION



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
        getRobotDrive().stop();
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

    /**
     * @return the robotDrive
     */
    public DriveTrain getRobotDrive() {
        return robotDrive;
    }

    /**
     * @param robotDrive the robotDrive to set
     */
    public void setRobotDrive(DriveTrain robotDrive) {
        this.robotDrive = robotDrive;
    }

    /**
     * @return the sensors
     */
    public SensorSet getSensors() {
        return sensors;
    }

    /**
     * @param sensors the sensors to set
     */
    public void setSensors(SensorSet sensors) {
        this.sensors = sensors;
    }

    /**
     * @return the encoder
     */
    public Encoder getEncoder() {
        return encoder;
    }

    /**
     * @param encoder the encoder to set
     */
    public void setEncoder(Encoder encoder) {
        this.encoder = encoder;
    }

    /**
     * @return the left
     */
    public DigitalInput getLeft() {
        return left;
    }

    /**
     * @param left the left to set
     */
    public void setLeft(DigitalInput left) {
        this.left = left;
    }

    /**
     * @return the right
     */
    public DigitalInput getRight() {
        return right;
    }

    /**
     * @return the middle
     */
    public DigitalInput getMiddle() {
        return middle;
    }
}
