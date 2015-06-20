package RobotCodeTyler;

import com.sun.squawk.util.Arrays;
import edu.wpi.first.wpilibj.Joystick;

/**
 *
 * @author Tyler
 * mimicks <code>ArrayList</code> objects for Joysticks (i.e. ArrayList<Joystick>)
 */
public class JoystickList 
{
    private Joystick[] joystickArray;

    public JoystickList()
    {
        joystickArray = new Joystick[0];
    }

    public JoystickList(Joystick[] joysticks)
    {
        joystickArray = joysticks;
    }

    public void add(Joystick newJoystick)
    {
        Joystick[] tempArray = new Joystick[joystickArray.length];

        //copy old joystick array into temporary joystick array
        for(int i = 0; i< tempArray.length; i++)
        {
            tempArray[i] = joystickArray[i];
        }
        joystickArray = new Joystick[joystickArray.length+1];

        //copy temporary array into new joystick array
        for(int i = 0; i< tempArray.length; i++)
        {
            joystickArray[i] = tempArray[i];
        }
        tempArray = null;
        joystickArray[joystickArray.length-1] = newJoystick;
    }

    public Joystick[] getJoysticks()
    {
        return joystickArray;
    }

    //uses zero-based index for accessing specific objects in the array
    public Joystick get(int index)
    {
        return joystickArray[index];
    }

    //returns the size of the array
    public int size()
    {
        return joystickArray.length;
    }
}
