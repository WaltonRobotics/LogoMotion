
package edu.wpi.first.wpilibj.templates;
import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.Accelerometer;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Timer;

/**
 * @author andrey
 * SensorSet contains and enables access and control of the camera, accelerometer, and gyro sensors.
 */
public class SensorSet
{
 private Camera theCamera;

 //a single accelerometer only measures acceleration in one direction, multiple are necessary
 //to measure all movement (correct me if i'm wrong)
 private ADXL345_I2C accelerometer;
 private Gyro gyro;

 //an array holding the robot's speed in the directions parallel to the
 //accelerometers
 private double[] speed;

 //the thread that tracks the robots position based on accelerometer values
 private SensorThread positionTracker;

 //the coordinates of the robot, one coordinate for each acceleromter
 private double[] position;

 //the robot's angle
 private double angle;

 /**
  * A constructor that initializes all the sensors possible.
  * @param gyroChannel the channel used for the gyro initialization.
  *        If -1, there is no gyro.
  * @param accChannels the channels used for the accelerometer initialization.
  *        If the length of the array is 0, there are no accelerometers.
  * @param baseChannel the channel used for the initialization of the base PWM of the camera.
  *        If -1, there is no camera.
  * @param panMotor the channel used for the initialization of the pan PWM of the camera.
  * @param servoChannel the channel used for the initialization of the servo of the camera.
  */
    public SensorSet(int gyroChannel,int accSlot,boolean CameraInit,int baseChannel, int panMotor, int servoChannel){
        //sets robot initial angle to 0 - can change this based on in'itial set up in game
        angle = 0;
        position = new double[3];

        speed = new double[3];


        if(gyroChannel == -1)
        {
        }
        else{
            gyro = new Gyro(gyroChannel);
            gyro.reset();
        }
        //accelerometer = new ADXL345_I2C(accSlot,ADXL345_I2C.DataFormat_Range.k8G);

        if(baseChannel == -1 && CameraInit)
        {
            theCamera=new Camera();
        }
        else{
            theCamera=new Camera(baseChannel,panMotor,servoChannel);
        }
        positionTracker = new SensorThread(this);
 }

    //starts the sensor thread that tracks the robot's position
    public void initSensorTracking(){
        positionTracker.start();
    }

    /**
     * A method to allow access to the gyro angle method.
     * @return the current heading of the robot in degrees. This heading is based on integration
     * of the returned rate from the gyro. It is continuous and can be larger than 360 degrees.
     */
    public double getGyroAngle(){
        return gyro.getAngle();
    }

    /**
     * A method that resets the gyro to 0 angle.
     */
    public void gyroReset(){
        gyro.reset();
    }


    /**
     * A method that allows access to the accelerometer readout.
     * @return The current acceleration of the sensor in Gs.
     */
    public double getAcceleration(ADXL345_I2C.Axes axis)
    {
        return accelerometer.getAcceleration(axis);
    }

    public ADXL345_I2C.AllAxes getAccelerstions()
    {
        return accelerometer.getAccelerations();
    }

    public double getSpeed(int number){
        return speed[number];
    }

    public double[] getPosition(){
        return position;
    }

    public double getRobotAngle(){
        return angle;
    }

    public class SensorThread extends Thread{
        double[] lastCheckAcc;
        double[] lastAcc;
        SensorSet sensors;
        Timer timer;

        public SensorThread(SensorSet sens){
            sensors = sens;

        }

        //starts the thread, keeping track of the robots position
        public void run(){
        }
    }
}


