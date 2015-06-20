package edu.team2974.robot.util;

/**
 *
 */
public abstract class AutoFunction extends Thread
{

    private boolean running = false;

    /**
     *
     */
    public synchronized void start() {
        running = true;
        super.start();
    }

    /**
     *
     */
    public synchronized void stop() {
        running = false;
    }

    /**
     *
     * @return
     */
    public boolean isRunning() {
        return running;
    }

    /**
     *
     * @return
     */
    public abstract boolean atCorrectHeight();

    //sets running = true and starts the thread if not already started
    /**
     *
     */
    public void begin() {
        try {
            if (!running) {
                running = true;
                this.start();
            }
        } catch (RuntimeException rte) {
        }
    }
}
