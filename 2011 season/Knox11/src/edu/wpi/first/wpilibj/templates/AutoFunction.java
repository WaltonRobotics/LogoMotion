package edu.wpi.first.wpilibj.templates;

public abstract class AutoFunction extends Thread
{
    private boolean running = false;

    public synchronized void start() {
        running = true;
        super.start();
    }

    public synchronized void stop() {
        running = false;
    }

    public boolean isRunning() {
        return running;
    }

    public abstract boolean atCorrectHeight();

    //sets running = true and starts the thread if not already started
    public void begin() {
        try {
            running = true;
            this.start();
        } catch (RuntimeException rte) {
        }
    }
}
