/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;


/**
 *
 * @author Gil
 */
//this class stores a pair of double (decimal) values and contains methods for reduction
//to the correct range and for chain limiting.  It is used to store and process
//the values that will become the motors' speeds.
public class SpeedPair implements SpeedSet{
        static final double maxChange = .03;

        //left speed
        public double left;

        //right speed
        double right;

        //creates a new SpeedPair with values l and r;
        public SpeedPair(double l, double r){
            left = l;
            right = r;
        }

        //reduces the SpeedPair so that both left and right are in the range
        //[-1,1] while maintaining the ratio between the two
        public void reduce() {

           //sets double max equal to the larger magnitude of left and right
           double max = java.lang.Math.abs(left);
           if(java.lang.Math.abs(right) > max){
               max = java.lang.Math.abs(right);
           }

           //if max is greater than 1, divides the pair by max
           if (max > 1){
               this.divideBy(max);
           }
       }

        //changes the SpeedPair so that it does not differ by more than
        //the constant maxChange from another SpeedPair.
        //Used as a chain limiter by putting previous state as the parameter.
        public void limitTo(SpeedSet prev){

            SpeedPair previous = (SpeedPair)prev;

            // if the difference for the left speeds is too large, modify the speed accordingly
            if(java.lang.Math.abs(left - previous.left) > maxChange){
                if(left > previous.left){
                    left = previous.left + maxChange;
                }
                if(left < previous.left){
                    left = previous.left - maxChange;
                }
            }

            //if the difference for the right speeds is too large, modify the speed accordingly
            if(java.lang.Math.abs(right - previous.right) > maxChange){
                if(right > previous.right){
                    right = previous.right + maxChange;
                }
                if(right < previous.right){
                    right = previous.right - maxChange;
                }
            }
        }

        //changes the SpeedPair so that it is not greater than another SpeedPair
        //by more than the constant maxChange.  Does not affect deceleration.
        public void limitIncreaseTo(SpeedSet prev){

            SpeedPair previous = (SpeedPair)prev;

            //if the difference for the left speeds is too large, modify the speed accordingly
            if(java.lang.Math.abs(left - previous.left) > maxChange){

                //tests whether the new value of left has a greater magnitude than the old one.
                //If so, limits it.
                if(java.lang.Math.abs(left) > java.lang.Math.abs(previous.left)){
                    if(left > previous.left){
                       left = previous.left + maxChange;
                    }
                    if(left < previous.left){
                       left = previous.left - maxChange;
                    }
                }
            }

           //if the difference for the right speeds is too large, modify the speed accordingly
            if(java.lang.Math.abs(right - previous.right) > maxChange){

                //tests whether the new value of right has a greater magnitude than the old one.
                //If so, it limits it.
                if(java.lang.Math.abs(right) > java.lang.Math.abs(previous.right)){
                    if(right > previous.right){
                       right = previous.right + maxChange;
                    }
                    if(right < previous.right){
                       right = previous.right - maxChange;
                    }
                }
            }
        }

        //Divides the values left and right by the parameter.
        public void divideBy(double factor){
            left = left / factor;
            right = right / factor;
        }

        //multiplies the values left and right by the parameter
        public void multiplyBy(double multiplier){
            left = left * multiplier;
            right = right * multiplier;
        }

        public void square(){
            double max = Math.abs(left);
            if(Math.abs(right) > max){
                max = Math.abs(right);
            }
            this.multiplyBy(max);
        }

        public String toString()
        {
            return "R:"+right+" and L:"+left;
        }
    }
