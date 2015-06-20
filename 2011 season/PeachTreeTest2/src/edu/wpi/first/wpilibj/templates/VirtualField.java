
package edu.wpi.first.wpilibj.templates;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;

/**
 * This is a class to keep track of robot position and robot Distance to the three target pins.
 * The 0,0 current used is the left corner of the red side of the field.
 * Although we could theoretically only keep one set of distance measurement, we need to test and see whether the camera
 * or the sensors operate better and which to weigh more.
 * @author andy
 */
public class VirtualField
{
   Vector robotPos;
   Vector distanceToLeftPins;
   Vector distanceToRightPins;
   Vector distanceToMiddlePins;
   Vector cameradistanceToLeftPins;
   Vector cameradistanceToRightPins;
   Vector cameradistanceToMiddlePins;
   DigitalInput Red;
   DigitalInput Left;
   DigitalInput Right;
   double currentAngle;
   /**
    *A constructor that initializes the positions based on three switchs.
    * The distances are currently not accurate and need to be made accurate.
    *
    * @param RedSwitchChannel the Channel of the switch which indicates whether the robot is on the red team.
    * @param LeftSwitchChannel the Channel of the switch which indicates whether the robot is initially in front of the left pins.
    * @param RightSwitchChannel the Channel of the switch which indicates whether the robot is initially in front of the right pins.
    */
   public VirtualField(int RedSwitchChannel,int LeftSwitchChannel,int RightSwitchChannel)
    {
       Red=new DigitalInput(RedSwitchChannel);
       Left=new DigitalInput(LeftSwitchChannel);
       Right=new DigitalInput(RightSwitchChannel);
       if(Red.get())
       {
           currentAngle=0;
       if(Left.get())
       {
           robotPos=new Vector(25,50);
           distanceToLeftPins =new Vector(0,10);
           cameradistanceToLeftPins=new Vector(0,10);
           distanceToRightPins=new Vector(50,10);
           cameradistanceToRightPins=new Vector(50,10);
           distanceToMiddlePins=new Vector(25,10);
           cameradistanceToMiddlePins=new Vector(25,10);
           }
       else if(Right.get())
       {
           robotPos=new Vector(75,50);
           distanceToLeftPins=new Vector(-50,10);
           distanceToRightPins=new Vector(0,10);
           distanceToMiddlePins=new Vector(-25,10);
           cameradistanceToLeftPins=new Vector(-50,10);
           cameradistanceToRightPins=new Vector(0,10);
           cameradistanceToMiddlePins=new Vector(-25,10);
       }
       else
       {
           robotPos=new Vector(50,50);
           distanceToLeftPins=new Vector(-25,10);
           distanceToRightPins=new Vector(25,10);
           distanceToMiddlePins=new Vector(0,10);
           cameradistanceToLeftPins=new Vector(-25,10);
           cameradistanceToRightPins=new Vector(25,10);
           cameradistanceToMiddlePins=new Vector(0,10);
        }
        }
       else
       {
           currentAngle=180;
       if(Left.get())
       {
           robotPos=new Vector(25,10);
           distanceToLeftPins=new Vector(0,-10);
           distanceToRightPins=new Vector(50,-10);
           distanceToMiddlePins=new Vector(25,-10);
           cameradistanceToLeftPins=new Vector(0,-10);
           cameradistanceToRightPins=new Vector(50,-10);
           cameradistanceToMiddlePins=new Vector(25,-10);
           }
       else if(Right.get())
       {
           robotPos=new Vector(75,10);
           distanceToLeftPins=new Vector(-50,-10);
           distanceToRightPins=new Vector(0,-10);
           distanceToMiddlePins=new Vector(-25,-10);
           cameradistanceToLeftPins=new Vector(-50,-10);
           cameradistanceToRightPins=new Vector(0,-10);
           cameradistanceToMiddlePins=new Vector(-25,-10);
       }
       else
       {
           robotPos=new Vector(50,10);
        }
       }
    }

   public void setCameraDistances(ParticleAnalysisReport[] pins)
    {

   }

   public void updateDistances(SpeedPair currentSpeed)
   {
   }
   }


