/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */


package edu.wpi.first.wpilibj.templates;
 
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Relay;
/**
 *
 * @author Developer
 */
public class MinibotDeployer {
    Relay extender;
    Victor rotator;
    private boolean deployState;

    public MinibotDeployer(int rotatorChannel, int extenderChannel){
        extender = new Relay(extenderChannel);
        rotator = new Victor(rotatorChannel);
        deployState = false;
    }

    public void extend(){
        extender.set(Relay.Value.kForward);
    }

    public void rotateDown(){
        rotator.set(.5);
        deployState=true;
    }
    public void rotateUp(){
        rotator.set(-.8);
        deployState=false;
    }
    public boolean isDown(){
        return deployState;
    }

    public void stopRotator(){
        rotator.set(0);
    }

    public void stopExtender(){
        extender.set(Relay.Value.kOff);
    }
    
    public void stop(){
        stopExtender();
        stopRotator();
    }
}
