

package edu.wpi.first.wpilibj.templates;
import com.sun.squawk.util.MathUtils;
/**
 * A simple class to store vectors.
 * @author andy
 */
public class Vector
{
    private double X;
    private double Y;
    public Vector(int x,int y)
    {
        X=x;
        Y=y;
    }
     public double getX()
    {
    return X;
     }

     public double getY()
    {
    return Y;
     }

     public double calculateAngle()
     {
         return Math.toDegrees(MathUtils.atan(Y/X));
     }

     public double calculateDistance()
    {
         return Math.sqrt(X*X+Y*Y);
     }

}
