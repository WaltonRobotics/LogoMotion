package edu.team2974.robot.util;

import edu.team2974.robot.camera.Camera;
import edu.wpi.first.wpilibj.ADXL345_I2C;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Timer;

/**
 * Contains and enables access and control of camera, accelerometer and
 * gyro sensors. Currently. A very useful abstraction/bundle to have handy!
 * @author andrey
 */
public class SensorSet
{

    /**
     * If a camera is being controlled, this will be non-null.
     */
    private Camera theCamera = null;
    /**
     * A single accelerometer only measures acceleration in one direction,
     * multiples are necessary to measure all movement.
     * If used, will be non-null.
     */
    private ADXL345_I2C accelerometer = null;
    /**
     * Always created if passed a real channel during instantiation.
     */
    private Gyro gyro = null;
    /**
     * Holds the robot's speed in the directions parallel to the accelerometers.
     */
    private double[] speed;
    /**
     * This thread tracks the robot's position based on accelerometer values.
     */
    private SensorThread positionTracker;
    /**
     * These are the robot's current coordinates, one for each accelerometer.
     */
    private double[] position;
    /**
     * An attribute named "angle". Doesn't really do anything. Just sits here,
     * woefully unused, an undocumented remnant of a previous design idea or
     * partially implemented thought. Sad, really. It seems so very promising.
     */
    private double angle = 0.0;
    /**
     * Our measure of the gyro's drift per second.
     */
    private double gyroDriftPerSecond = 0.0;
    /**
     * Fudge factor applied to all gyro angle return values (ok, some) intended
     * to compensate for "drift" on the gyroscope.
     */
    private double gyroOffset;

    /**
     * A constructor that initializes all the sensors possible.
     *
     * @param gyroChannel the channel used for the gyro initialization. If -1,
     * there is no gyro.
     * @param accSlot currently unused
     * @param tiltChannel if non-zero we feed it to our created Camera
     * @param startCamera if <code>true</code> we create a Camera
     * @param panMotor currently unused.
     */
    public SensorSet(int gyroChannel, int accSlot, boolean startCamera,
            int tiltChannel, int panMotor) {
        position = new double[3];

        speed = new double[3];

        if (gyroChannel != -1) {
            gyro = new Gyro(gyroChannel);
            gyro.reset();
        }
        //accelerometer = new ADXL345_I2C(accSlot,ADXL345_I2C.DataFormat_Range.k8G);

        if (startCamera) {
            if (tiltChannel == 0) {
                theCamera = new Camera();
            } else {
                theCamera = new Camera(tiltChannel, panMotor);
            }

            positionTracker = new SensorThread(this);
        }
    }

    /**
     * Starts the sensor thread that tracks the robot's position
     */
    public void initSensorTracking() {
        positionTracker.start();
    }

    /**
     * A method to allow access to the gyro angle method.
     * @return the current heading of the robot in degrees. This heading is
     * based on integration of the returned rate from the gyro. It is continuous
     * and can be larger than 360 degrees. The current gyro offset is
     * subtracted from the measured value and the result is returned.
     */
    public double getGyroAngle() {
        return gyro == null ? 0.0 : gyro.getAngle() - gyroOffset;
    }

    /**
     * Obtains the raw value of the gyro's angle.
     * @return the result of calling <code>gyro.getAngle()</code>
     */
    public double getGyroAngleRaw() {
        return gyro == null ? 0.0 : gyro.getAngle();
    }

    /**
     * Performs gyro drift tuning. Recalculates the gyro drift per second.
     * Reads gyro values 1 full second apart, so calling this will freeze things
     * for that interval.
     */
    public void tuneGyro() {
        if (gyro != null) {
            //gyro drift tuning
            double start = gyro.getAngle();
            Timer.delay(1);
            double end = gyro.getAngle();
            gyroDriftPerSecond = end - start;
        }
    }

    /**
     * A method that resets the gyro to 0 angle.
     */
    public void gyroReset() {
        if (gyro != null) {
            gyro.reset();
        }
    }

    /**
     * A method that allows access to the accelerometer readout.
     * @param axis the axis whose acceleration you want
     * @return The current acceleration of the sensor in Gs.
     */
    public double getAcceleration(ADXL345_I2C.Axes axis) {
        return accelerometer == null
                ? 0.0
                : accelerometer.getAcceleration(axis);
    }

    /**
     * Returns the accelerations from the accelerometer.
     * @return the accelerometer's current acceleration values
     */
    public ADXL345_I2C.AllAxes getAccelerations() {
        return accelerometer == null ? null : accelerometer.getAccelerations();
    }

    /**
     * Gets the current stored speed value for the index/axis indicated
     * @param number needs to be between 0 - 3 to get an actual value
     * @return 0.0 or the value at the axis requested
     */
    public double getSpeed(int number) {
        if (number < 4 && number > -1) {
            return speed[number];
        }
        return 0.0;
    }

    /**
     * Gets the current position array.
     * @return the currently stored array of positional data.
     */
    public double[] getPosition() {
        return position;
    }

    /**
     * Returns the robot's angle. Currently unused.
     * @return 0.0
     */
    public double getRobotAngle() {
        return angle;
    }

    /**
     * Returns the camera fed to it at instantiation.
     * @return
     */
    public Camera getCamera() {
        return theCamera;
    }

    /**
     * Gets the stored gyro.
     * @return the stored/managed gyro, if any.
     */
    public Gyro getGyro() {
        return gyro;
    }

    /**
     * The currently calculated gyro drift per second. Presumes the existence
     * of a functioning gyro to be of much use.
     * @return
     */
    public double getGyroDriftPerSecond() {
        return gyroDriftPerSecond;
    }

    /**
     * Allows the gyro offset to be manually incremented by the current drift
     * per second multiplied by the passed in number of seconds.
     * @param seconds your favorite fudge/correction factor
     */
    public void correctDrift(double seconds) {
        gyroOffset += (gyroDriftPerSecond * seconds);
    }
}
