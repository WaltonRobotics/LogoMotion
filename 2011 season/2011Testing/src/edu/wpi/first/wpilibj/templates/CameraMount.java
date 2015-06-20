/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;

/**
 *
 * @author programming
 */
public class CameraMount {
Servo tiltServo;
Servo panMotor;
Joystick joystick;

public CameraMount(int panChannel,int tiltChannel)
    {
    tiltServo=new Servo(tiltChannel);
    panMotor = new Servo(panChannel);
}
public void setTiltServo(double value)
    {
    tiltServo.set(value/2+0.5);
}
public void setPanServo(double value)
    {
    panMotor.set(value*value*value/2+0.5);
}

}
