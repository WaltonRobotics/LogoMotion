package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.Dashboard;
import edu.wpi.first.wpilibj.AnalogModule;
import edu.wpi.first.wpilibj.DigitalModule;
import edu.wpi.first.wpilibj.Solenoid;
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
public class RobotMain extends IterativeRobot {

    private MinibotDeployer minibotDeployer;
    private Compressor compressor;
    private AutoFunction driveTask;
    private AutoFunction armTask;
    private AutoFunction armTask2;
    private DriveTrain robotDrive;//a two motor drive
    private TankControls joysticks;//a tank drive control system for the robot
    //private pneumaticSystem pneumatics;
    private SensorSet sensors;//Sensors currently set to public because there is not much that can be changed and access could be needed in other classes.
    private boolean demoMode;
    private DriverStationLCD driverScreen;//Object used to display information to the driver as needed.
    //private DriverStationEnhancedIO enhancedIO = DriverStation.getInstance().getEnhancedIO();
    private boolean armState, driveState;
    //private boolean AUTO = true;
    private boolean TELEOP = false;
    //private XboxControls xboxController;
    private Encoder leftEncoder, rightEncoder;
    private Arm arm;
    //private CameraMount cameraMount;
    private DigitalInput left;
    private DigitalInput right;
    private DigitalInput middle;
    private Servo camPan;
    private Servo camTilt;
    private double tempPan;
    private double tempTilt;
    Watchdog myDog;
    int displayIndex;
    OutputStream fileOutput;
    DataOutputStream dataOutput;
    Joystick armJoystick, liftJoystick;
    private boolean driveInvert;
    private boolean turnInvert;

    /**
     * In accordance with the superClass, robotInit method is used for initialization rather than a constructor.
     */
    public void robotInit() {
        minibotDeployer = new MinibotDeployer(5, 2);

        System.out.println("robot_init");
        liftJoystick = new Joystick(2);
        armJoystick = new Joystick(4);
        setLeftEncoder(new Encoder(2, 3));
        getLeftEncoder().setDistancePerPulse(8.0 * 3.14 / 360.0 / 2);
        setRightEncoder(new Encoder(4, 5));
        getRightEncoder().setDistancePerPulse(8.0 * 3.14 / 360.0 / 2);
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
        myDog.setExpiration(5);
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
        camPan = new Servo(10);
        camTilt = new Servo(9);
        tempPan = .68;
        tempTilt= .15;
        driveTask = new WaltLineTrack(false, false, this);
        driveInvert=false;
        turnInvert=false;
    }

    /**
     * The method that runs and controls the Robot's autonomic behavior. Most, if not all, of it will be written after the
     * game is announced.  it is called repeatedly, though not continuously, throughout the autonomous period
     */
    public void autonomousPeriodic() {
        myDog.feed();
    }

    public void autonomousInit() {
        myDog.feed();
        //autonomousTest();
        autonomousGold();
        //forkLiftTest();
    }

    public void autonomousGold() {
        getLeftEncoder().reset();
        getLeftEncoder().start();
        getRightEncoder().reset();
        getRightEncoder().start();
        //Hard Coded Testing
        printToScreen(6, "AUTO INIT!!!!!");

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
        if (willTrack) {
            driveTask.begin();

            while (driveTask.isRunning()) {
                Timer.delay(.1);
            }

            //armTask.begin();
            //Timer.delay(1);

            getRobotDrive().goForward(.3);

            getLeftEncoder().reset();

            getLeftEncoder().start();

            long stopTime = System.currentTimeMillis() + 1050; //was 1200

            while (-getDriveDistance() < 29) {
                myDog.feed();
                if (System.currentTimeMillis() >= stopTime) {
                    printToScreen("ROBOT MAIN TIMED OUT!!");
                    break;
                }
            }
            // 1/10th power forward
            getRobotDrive().goForward(-.03);
            //.1 second delay so that jags can power forwad
            Timer.delay(.1);
            //stops robot
            getRobotDrive().stop();
            //makes the arm fall


            printToScreen("Running test");
            arm.armControl(.6);
            Timer.delay(.4);
            printToScreen("Jag speed " + arm.armJag.get());
            arm.armControl(-.9);
            Timer.delay(.4);
            printToScreen("Jag speed " + arm.armJag.get());

            //test to see fork lift go to predetermined heights
            arm.forkliftControl(.9);
            Timer.delay(2.0);
            arm.forkliftControl(.2);

            //makes the arm fall
            arm.armControl(.6);
            //makes sure arm is clear of hardstop and falling
            Timer.delay(.5);
            //lets gravity do the rest
            arm.armControl(0);
            //waits until arm is at a position to let go of the tube successfully
            Timer.delay(.75);
            //releases tube
            arm.grabTube(true);

            robotDrive.goForward(.3);
            Timer.delay(1);
            robotDrive.stop();
        }
        //We're done. Shut off everything we turned on
        disabledInit();
    }

    public void autonomousTest() {
        getLeftEncoder().reset();
        getLeftEncoder().start();
        getRightEncoder().reset();
        getRightEncoder().start();
        //Hard Coded Testing
        printToScreen(6, "AUTO INIT!!!!!");

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
        if (willTrack) {
            driveTask.begin();

            while (driveTask.isRunning()) {
                Timer.delay(.1);
            }

            //armTask.begin();
            //Timer.delay(1);

            getRobotDrive().goForward(.3);

            getLeftEncoder().reset();

            getLeftEncoder().start();

            long stopTime = System.currentTimeMillis() + 1100;

            while (-getDriveDistance() < 29) {
                myDog.feed();
                if (System.currentTimeMillis() >= stopTime) {
                    printToScreen("ROBOT MAIN TIMED OUT!!");
                    break;
                }
            }
            // 1/10th power forward
            getRobotDrive().goForward(-.03);
            //.1 second delay so that jags can power forwad
            Timer.delay(.1);
            //stops robot
            getRobotDrive().stop();
            //makes the arm fall


            printToScreen("Running test");
            arm.armControl(.6);
            Timer.delay(.4);
            printToScreen("Jag speed " + arm.armJag.get());
            arm.armControl(-.9);
            Timer.delay(.4);
            printToScreen("Jag speed " + arm.armJag.get());
            
            //test to see fork lift go to predetermined heights
            arm.forkliftControl(.9);
            Timer.delay(2.0);
            arm.forkliftControl(.2);

            //makes the arm fall
            arm.armControl(.6);
            //makes sure arm is clear of hardstop and falling
            Timer.delay(.5);
            //lets gravity do the rest
            arm.armControl(0);
            //waits until arm is at a position to let go of the tube successfully
            Timer.delay(.75);
            //releases tube
            arm.grabTube(true);

            robotDrive.goForward(.3);
            Timer.delay(1);
            robotDrive.stop();
        }
        disabledInit();
    }

    public void forkLiftTest()
    {
            printToScreen("Running hardstop");
            arm.armControl(.6);
            Timer.delay(.4);
            printToScreen("Jag speed " + arm.armJag.get());
            arm.armControl(-.9);
            Timer.delay(.6);
            printToScreen("Jag speed " + arm.armJag.get());
            printToScreen("Cleared hardstop");

            //test to see fork lift go to predetermined heights
            printToScreen("Forklift Moving up");
            arm.forkliftControl(.9);
            Timer.delay(1.0);
            arm.forkliftControl(.2);
            printToScreen("Holding forklift up");
    }

    /*
     * The method that allows control of the robot during the teleoperational period and displays information from the sensors on the screen.
     * This runs in an infinite loop that updates every .02 seconds as per last year for the entirety of the teleoperational period.
     */
    public void teleopInit() {
        printToScreen(4, ">>>TELEOP INIT ENTERED!!");
        driveState = TELEOP;
        armState = TELEOP;
        disabledInit();
        printToScreen(4, ">>BACK FROM Di IN Ti!!");
        getLeftEncoder().reset();
        getLeftEncoder().start();
        getRobotDrive().stop();
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

        handleMinibotInput();

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
    }

    //this method is called frequently during the teleop period, though with delay
    public void teleopPeriodic() {
        driverScreen.updateLCD();
        updateDashboard();
    }

    void updateDashboard() {
        Dashboard lowDashData = DriverStation.getInstance().getDashboardPackerLow();
        lowDashData.addCluster();
        {
            lowDashData.addCluster();
            {     //analog modules
                lowDashData.addCluster();
                {
                    for (int i = 1; i <= 8; i++) {
                        lowDashData.addFloat((float) AnalogModule.getInstance(1).getAverageVoltage(i));
                    }
                }
                lowDashData.finalizeCluster();
                lowDashData.addCluster();
                {
                    for (int i = 1; i <= 8; i++) {
                        lowDashData.addFloat((float) AnalogModule.getInstance(2).getAverageVoltage(i));
                    }
                }
                lowDashData.finalizeCluster();
            }
            lowDashData.finalizeCluster();

            lowDashData.addCluster();
            { //digital modules
                lowDashData.addCluster();
                {
                    lowDashData.addCluster();
                    {
                        int module = 4;
                        lowDashData.addByte(DigitalModule.getInstance(module).getRelayForward());
                        lowDashData.addByte(DigitalModule.getInstance(module).getRelayForward());
                        lowDashData.addShort(DigitalModule.getInstance(module).getAllDIO());
                        lowDashData.addShort(DigitalModule.getInstance(module).getDIODirection());
                        lowDashData.addCluster();
                        {
                            for (int i = 1; i <= 10; i++) {
                                lowDashData.addByte((byte) DigitalModule.getInstance(module).getPWM(i));
                            }
                        }
                        lowDashData.finalizeCluster();
                    }
                    lowDashData.finalizeCluster();
                }
                lowDashData.finalizeCluster();

                lowDashData.addCluster();
                {
                    lowDashData.addCluster();
                    {
                        int module = 6;
                        lowDashData.addByte(DigitalModule.getInstance(module).getRelayForward());
                        lowDashData.addByte(DigitalModule.getInstance(module).getRelayReverse());
                        lowDashData.addShort(DigitalModule.getInstance(module).getAllDIO());
                        lowDashData.addShort(DigitalModule.getInstance(module).getDIODirection());
                        lowDashData.addCluster();
                        {
                            for (int i = 1; i <= 10; i++) {
                                lowDashData.addByte((byte) DigitalModule.getInstance(module).getPWM(i));
                            }
                        }
                        lowDashData.finalizeCluster();
                    }
                    lowDashData.finalizeCluster();
                }
                lowDashData.finalizeCluster();

            }
            lowDashData.finalizeCluster();

            lowDashData.addByte(Solenoid.getAllFromDefaultModule());
        }
        lowDashData.finalizeCluster();
        lowDashData.commit();

    }

    public void disabledContinuous() {
        if (driveTask != null) {
            driveTask.stop();
            driveTask=null;
        }

        if (armTask != null) {
            armTask.stop();
            armTask=null;
        }

        if (armTask2 != null) {
            armTask2.stop();
            armTask2=null;
        }
        robotDrive.stop();
    }

    public void disabledInit() {
        if (driveTask != null) {
            driveTask.stop();
            driveTask = null;
        }

        if (armTask != null) {
            armTask.stop();
            armTask = null;
        }

        if (armTask2 != null) {
            armTask2.stop();
            armTask2 = null;
        }
        robotDrive.stop();
        printToScreen("Disabling robot and all threads.");
    }

    public void disabledPeriodic() {
        if (driveTask != null) {
            driveTask.stop();
            driveTask=null;
        }

        if (armTask != null) {
            armTask.stop();
            armTask=null;
        }

        if (armTask2 != null) {
            armTask2.stop();
            armTask2=null;
        }
        robotDrive.stop();
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
        //double[] instructions = joysticks.getMovementInstructions();
        if(joysticks.getRightButton(7))
        {
            driveInvert= true;
            printToScreen("Drive inverted");
        }
        else if(joysticks.getRightButton(6))
        {
            driveInvert=false;
            printToScreen("Drive normal");
        }

        if(joysticks.getRightButton(11))
        {
            turnInvert= true;
            printToScreen("Turn inverted");
        }
        else if(joysticks.getRightButton(10))
        {
            turnInvert=false;
            printToScreen("Turn normal");
        }
        
        double[] instructions;
        if(driveInvert)
        {
            instructions = joysticks.getInvertedMoveInstructions();
        }
        else
        {
           instructions= joysticks.getMovementInstructions();
        }

        


        //Comment one or the other out to set how to control overdrive
        boolean overDrive;
        if (joysticks.getLeftTrigger()||joysticks.getRightTrigger())
        {
            overDrive = true;
        }
        else
        {
            overDrive=false;
        }
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

       
    }

    private void handleArmInput() {
        //armjoy stick 8 drop 9 deploy
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

            arm.resetArmAngle();
        }

//        if (liftJoystick.getRawButton(6)) {
//
//            armTask = arm.new GoToHeight(2);
//            armTask.begin();
//            armState = AUTO;
//        }

        //checks if the wrist should auto flip- it flips if it passes through the "critical" zone
        //arm.wristFlipCheck(); NEVER HAVE UNCOMMENTED



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
        printToScreen("" + armSpeed);
        if(armJoystick.getRawButton(2)){
            arm.armControlRaw(armSpeed);
        }
        else{
            arm.armControl(armSpeed);
        }
                

        //oscillates otherwise.  This causes it to maintain its angle as long as no significant

        //END GILS CHANGES TO IMPLEMENT OSCILLATION

        if (liftJoystick.getRawButton(2)) {
            tempPan = tempPan + 0.01;
            if (tempPan > 1) {
                tempPan = 1;
            }
        }
        else if (liftJoystick.getRawButton(3)) {
            tempPan = tempPan - 0.01;
            if (tempPan < -1) {
                tempPan = -1;
            }
        }
        camPan.set(tempPan);
        printToScreen("Temp pan "+tempPan);
        if (liftJoystick.getRawButton(4)) {
            tempTilt = tempTilt + 0.01;
        } else if (liftJoystick.getRawButton(5)) {
            tempTilt = tempTilt - 0.01;
        }
        if(tempTilt > 1){
            tempTilt = 1;
        }
        if(tempTilt < -1){
            tempTilt = -1;
        }
        camTilt.set(tempTilt);
        printToScreen("Temp tilt "+tempTilt);
//        if (liftJoystick.getRawButton(3)) {
//            arm.heightenArm();
//        } else if (liftJoystick.getRawButton(2)) {
//            arm.lowerArm(); //0.4 seemed like a good number
//        } else {
//            arm.oscillate(); //set somethign to avoid gravy
//            //arm.armJag.set(0);
//        }
    }

    private void handleMinibotInput() {
        if (armJoystick.getRawButton(8)) {
            minibotDeployer.rotateDown();
        }
        else if(armJoystick.getRawButton(10)){
            minibotDeployer.rotateUp();
        }
        else {
            minibotDeployer.stopRotator();
        }

        if (armJoystick.getRawButton(9)) {
            minibotDeployer.extend();
        } else {
            minibotDeployer.stopExtender();
        }
    }

    /**
     * Stops all operation in the driverTrain and pneumaticSystem.
     * TODO: Decide whether to writer safeStart() or make safeStop() timed for a specific amount of seconds.
     */
    private void safeStop() {
        getRobotDrive().stop();
        //pneumatics.stop();
    }

    public void printToScreen(int line, String output) {
        System.out.println(output);
        printALineOfSpaces(line);
        switch (line) {
            case 1:
                driverScreen.println(DriverStationLCD.Line.kMain6, 1, output);
                break;
            case 2:
                driverScreen.println(DriverStationLCD.Line.kUser2, 1, output);
                break;
            case 3:
                driverScreen.println(DriverStationLCD.Line.kUser3, 1, output);
                break;
            case 4:
                driverScreen.println(DriverStationLCD.Line.kUser4, 1, output);
                break;
            case 5:
                driverScreen.println(DriverStationLCD.Line.kUser5, 1, output);
                break;
            case 6:
                driverScreen.println(DriverStationLCD.Line.kUser6, 1, output);
                break;
        }
    }

    public void printToScreen(String output) {
        System.out.println(output);
        printALineOfSpaces(displayIndex);
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
    private static final String _80spaces = "                                        "
            + "                                        ";

    private void printALineOfSpaces(int line) {
        switch (line) {
            case 1:
                driverScreen.println(DriverStationLCD.Line.kMain6, 1, _80spaces);
                break;
            case 2:
                driverScreen.println(DriverStationLCD.Line.kUser2, 1, _80spaces);
                break;
            case 3:
                driverScreen.println(DriverStationLCD.Line.kUser3, 1, _80spaces);
                break;
            case 4:
                driverScreen.println(DriverStationLCD.Line.kUser4, 1, _80spaces);
                break;
            case 5:
                driverScreen.println(DriverStationLCD.Line.kUser5, 1, _80spaces);
                break;
            case 6:
                driverScreen.println(DriverStationLCD.Line.kUser6, 1, _80spaces);
                break;
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
    public Encoder getLeftEncoder() {
        return leftEncoder;
    }

    /**
     * @param encoder the encoder to set
     */
    public void setLeftEncoder(Encoder encoder) {
        this.leftEncoder = encoder;
    }

    /**
     * @return the encoder
     */
    public Encoder getRightEncoder() {
        return rightEncoder;
    }

    /**
     * @param encoder the encoder to set
     */
    public void setRightEncoder(Encoder encoder) {
        this.rightEncoder = encoder;
    }

    public double getDriveDistance() {
        return -getLeftEncoder().getDistance();
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
