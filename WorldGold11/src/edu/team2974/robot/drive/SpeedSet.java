package edu.team2974.robot.drive;

/**
 *
 * @author Gil
 */
public interface SpeedSet
{
    //reduces speedSet so that all values are between -1 and 1

    /**
     *
     */
    public void reduce();

    //divides the set by a given factor
    /**
     *
     * @param factor
     */
    public void divideBy(double factor);

    //limits how much the set changes in relation to a given pair
    /**
     *
     * @param previous
     */
    public void limitTo(SpeedSet previous);

    //limits how much the set increased its speed in relation to a given pair- to be used only for pairs
    /**
     *
     * @param previous
     */
    public void limitIncreaseTo(SpeedSet previous);

    /**
     *
     * @return
     */
    public String toString();

    /**
     *
     * @param multiplier
     */
    public void multiplyBy(double multiplier);

    /**
     *
     */
    public void square();
}
