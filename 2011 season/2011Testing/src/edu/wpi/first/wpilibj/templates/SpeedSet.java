/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

/**
 *
 * @author Gil
 */
public interface SpeedSet {
    //reduces speedSet so that all values are between -1 and 1
    public void reduce();

    //divides the set by a given factor
    public void divideBy(double factor);

    //limits how much the set changes in relation to a given pair
    public void limitTo(SpeedSet previous);

     //limits how much the set increased its speed in relation to a given pair- to be used only for pairs
    public void limitIncreaseTo(SpeedSet previous);

     public String toString();

     public void multiplyBy(double multiplier);

     public void square();
}
