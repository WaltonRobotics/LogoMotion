/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

/**
 *
 * @author Gil
 */
//this class stores a quadruple of double values and contains methods for reduction
//to the correct range and for chain limiting. This is used for a four wheel drive or
//mecanum/omni system.
public class SpeedQuadruple implements SpeedSet{

        //front two wheels
        SpeedPair front;

        //back two wheels
        SpeedPair back;

        //constructs a new quadruple with given values
        public SpeedQuadruple(double FL, double FR, double BL, double BR){
            front = new SpeedPair(FL, FR);
            back = new SpeedPair(BL, BR);
        }

        //for implementation only, not to be used (add throwing exception?)
        public void limitTo(SpeedPair previous){

        }

        //for implementation only, not to be used (add throwing exception?)
        public void limitIncreaseTo(SpeedPair previous){

        }

        //changes the SpeedPair so that it does not differ by more than
        //the constant maxChange from another SpeedQuadruple.
        //Used as a chain limiter by putting previous state as the parameter.
        //This will probably not need to be used considering that four motor systems
        //do not utilize chains, but there may still be some value to limiting acceleration/
        //deceleration.
        public void limitTo(SpeedSet prev){
            SpeedQuadruple previous = (SpeedQuadruple)prev;

            front.limitTo(previous.front);
            back.limitTo(previous.back);
        }

        //changes the SpeedPair so that it is not greater in magnitude than another SpeedPair
        //by more than the constant maxChange.  Does not affect deceleration.
        public void limitIncreaseTo(SpeedSet prev){
            SpeedQuadruple previous = (SpeedQuadruple)prev;

            front.limitIncreaseTo(previous.front);
            back.limitIncreaseTo(previous.back);
        }



        //reduces the SpeedQuadruple so that all four values are in the range
        //[-1,1] while maintaining the ratio between the values
        public void reduce(){
            //finds the value that is largest in magnitude, then divides the
            //entire quadruple by that magnitude if it is greater than 1
            //so that all values will have magnitudes at most 1.
            double max = this.getMaxMag();
            if(max > 1){
                this.divideBy(max);
            }
        }


        //divides all of the values in the quadruple by the parameter
        public void divideBy(double factor){
            front.divideBy(factor);
            back.divideBy(factor);
        }

        //multiplies all of the values in the quadruple by the parameter
        public void multiplyBy(double multiplier){
            front.multiplyBy(multiplier);
            back.multiplyBy(multiplier);
        }

        //multiplies the quadruple by its largest, maintaining the ratios.
        //Used to give driver finer control at slower speeds while still ramping up to maximum speed at full throttle.
        public void square(){
            double max = this.getMaxMag();
            this.multiplyBy(max);
        }

        //gets the value of the quadruple with the greatest magnitude, returning that magnitude
        public double  getMaxMag(){
            double max = java.lang.Math.abs(front.left);
            if(java.lang.Math.abs(front.right) > max){
                max = java.lang.Math.abs(front.right);
            }
            if(java.lang.Math.abs(back.left) > max){
                max = java.lang.Math.abs(back.left);
            }
            if(java.lang.Math.abs(back.right) > max){
                max = java.lang.Math.abs(back.right);
            }
            return max;
        }


         public String toString()
        {
         return "Front: "+front.toString()+" and back: "+back.toString();
         }
    }
