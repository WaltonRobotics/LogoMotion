package edu.team2974.robot.minibot;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Relay;

/**
 * This class represents the deployment mechanism for a minibot. The mechanism
 * is expected to consist of a rotator (a Victor) and an extender (A Relay).
 * Basic control over these two components is provided. So this class basically
 * pulls those two together for convenience.
 */
public class MinibotDeployer
{

    /**
     * Our extender - provides back and forth "push" deployment.
     */
    private Relay extender = null;
    /**
     * Our rotator - provides vertical to horizontal movement of the deploy arm.
     */
    private Victor rotator = null;
    /**
     * Indicates whether or not we are currently in a "deployed" state.
     */
    private boolean deployState;

    /**
     * Get things started.
     * @param rotatorChannel going to create a Victor around this
     * @param extenderChannel going to create a Relay around this
     */
    public MinibotDeployer(int rotatorChannel, int extenderChannel) {
        extender = new Relay(extenderChannel);
        rotator = new Victor(rotatorChannel);
        deployState = false;
    }

    /**
     * Sets the relay's value to forward.
     */
    public void extend() {
        extender.set(Relay.Value.kForward);
    }

    /**
     * Sets the Victor down (.375) and toggles state to "deployed".
     */
    public void rotateDown() {
        rotator.set(.375);
        //rotator.set(.5);
        deployState = true;
    }

    /**
     * Sets the Victor up (-0.8) and toggles state to "undeployed".
     */
    public void rotateUp() {
        rotator.set(-.8);
        deployState = false;
    }

    /**
     * Indicates whether or not the Victor has been "deployed"
     * @return the current state of the Victor
     */
    public boolean isDeployed() {
        return deployState;
    }

    /**
     * Sets the Victor to 0.
     */
    public void stopRotator() {
        rotator.set(0);
    }

    /**
     * Turns off the relay.
     */
    public void stopExtender() {
        extender.set(Relay.Value.kOff);
    }

    /**
     * Calls both "stop" methods. Call this for convenience.
     */
    public void stop() {
        stopExtender();
        stopRotator();
    }
}
