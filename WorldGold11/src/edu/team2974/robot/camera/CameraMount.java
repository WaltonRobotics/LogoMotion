package edu.team2974.robot.camera;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;

/**
 *
 * @author programming
 */
public class CameraMount
{

    Servo tiltServo;
    Servo panMotor;
    Joystick joystick;

    /**
     *
     * @param panChannel
     * @param tiltChannel
     */
    public CameraMount(int panChannel, int tiltChannel) {
        tiltServo = new Servo(tiltChannel);
        panMotor = new Servo(panChannel);
    }

    /**
     *
     * @param value
     */
    public void setTiltServo(double value) {
        tiltServo.set(-1 * (value / 2 + 0.5) + 1);
    }

    /**
     *
     * @param value
     */
    public void setPanServo(double value) {
        panMotor.set(-1 * (value * value * value / 2 + 0.5) + 1);
    }
}
