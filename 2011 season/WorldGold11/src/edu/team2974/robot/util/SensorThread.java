package edu.team2974.robot.util;

import edu.wpi.first.wpilibj.Timer;

/**
 *
 */
public class SensorThread extends Thread
{

    private double[] lastCheckAcc;
    private double[] lastAcc;
    private SensorSet sensors;
    private Timer timer;

    /**
     * Not sure why this class exists. It runs forever but never stops. Allows
     * no control over the embedded sensor set. Unused attributes have been
     * left in place in hopes they will trigger further updates of more
     * substance, or result in this class going away.
     * @param sens
     */
    public SensorThread(SensorSet sens) {
        sensors = sens;
    }

    /**
     * Starts the thread, keeping track of the robots position
     */
    public void run() {
    }
}
