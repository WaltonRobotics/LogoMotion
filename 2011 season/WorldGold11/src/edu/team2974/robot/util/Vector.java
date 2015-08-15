package edu.team2974.robot.util;

import com.sun.squawk.util.MathUtils;

/**
 * A simple class to store vectors.
 * @author andy
 */
public class Vector
{

    private double X;
    private double Y;

    /**
     *
     * @param x
     * @param y
     */
    public Vector(int x, int y) {
        X = x;
        Y = y;
    }

    /**
     *
     * @return
     */
    public double getX() {
        return X;
    }

    /**
     *
     * @return
     */
    public double getY() {
        return Y;
    }

    /**
     *
     * @return
     */
    public double calculateAngle() {
        return Math.toDegrees(MathUtils.atan(Y / X));
    }

    /**
     *
     * @return
     */
    public double calculateDistance() {
        return Math.sqrt(X * X + Y * Y);
    }
}
