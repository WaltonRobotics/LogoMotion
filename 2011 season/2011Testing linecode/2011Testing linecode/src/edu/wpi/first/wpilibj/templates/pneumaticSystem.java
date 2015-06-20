package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Relay;

/**
 * @author andrey(basic layout), Paul and The ever amazing CEO Chris
 * pneumaticSystem contains all the components of pneumatics and coordinates control of the entire system.
 */
public class pneumaticSystem
{
    private Compressor compressor;
    private boolean stop;
    private boolean ready;//Meant to be an enum object, but netbeans does not allow for them and so these are booleans for now.
    private boolean shoot;
    private Solenoid[] solenoids;//Consider changing from array to individual variables as needed; convert to cylinders at some point.
    private Relay compressorSpike;
    private double maxPressure;

    /**
     * A basic constructor that initializes all the needed components.
     * @param solenoidChannels the channels used for each solenoid constructor.
     * @param relayChannel the channel used for the relay constructor.
     * @param MaxPressure the maximum pressure that the pneumaticSystem can supply.
     */
    public pneumaticSystem(int[] solenoidChannels, int relayChannel, int switchChannel, double MaxPressure)
    {
        solenoids=new Solenoid[solenoidChannels.length];
        for(int c=0;c<solenoidChannels.length;c++)
            solenoids[c]=new Solenoid(solenoidChannels[c]);
        compressorSpike = new Relay(relayChannel);
        maxPressure=MaxPressure;
        ready=false;
        shoot=false;
        stop=false;
        compressorSpike.set(Relay.Value.kForward);
    }
    /**
     * This method takes input from either the singleController or the left controller and follows the directions.
     * TODO:Implement controls with true buttons calling methods such as shoot() and pressurize outside process() itself.
     * @param buttons the trigger, top, button 2, and button 3 of either the left or single Joystick.
     */
    public void process(boolean[] buttons)
    {
        boolean leftTrigger = buttons[0];
        boolean leftTop = buttons[1];
        boolean leftB4 = buttons[2];
        boolean leftB5 = buttons[3];

        if(leftTrigger)
        {

        }

        if(leftTop)
        {
            solenoids[0].set(true);

        }
        else{
            solenoids[0].set(false);
        }

        if(leftB4)
        {
           solenoids[1].set(true);
        }
        else{
           solenoids[1].set(false);
        }

        if(leftB5)
        {
           solenoids[1].set(true);
        }
        else{
           solenoids[1].set(false);
        }
    }

    public void pressurize()
    {
        compressor.start();
    }

    /**
     * This method releases the solenoids. Unfortunately since we do not know what each solenoid is supposed to do we can't really Do anything here yet XP
     * TODO:
     */


    /**
     * This method halts whatever the pneumatic system is doing.
     * TODO:Implement a start() if needed, implement the stop boolean in future written methods.
     * Make this method actually turn off all solenoids
     */
    public void stop()
    {
    compressor.stop();
    stop=true;
    shoot=false;
    ready=false;
    }


}

