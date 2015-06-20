package edu.team2974.robot.drive;

import edu.wpi.first.wpilibj.SpeedController;

/**
 * A drive system for a mecanum wheel drive train that uses one joystick.
 * It controls forward and backward motion via the main axis of the stick
 * side to side motion via the sideways axis of the stick, and turning via
 * the joystick's twist.
 *
 * @author Gil
 */
public class MecanumDrive extends FourMotorDrive
{

    //creates a mecanum drive train with appropriate motor controllers and demoMode
    /**
     *
     * @param frontL
     * @param frontR
     * @param backL
     * @param backR
     * @param demo
     */
    public MecanumDrive(SpeedController frontL, SpeedController frontR,
            SpeedController backL, SpeedController backR, boolean demo) {
        super(frontL, frontR, backL, backR, demo);
    }

    ///generates an initial SpeedQuadruple based on movement instructions
    /**
     *
     * @param instructions
     * @return
     */
    public SpeedSet getSpeeds(double[] instructions) {

        //translates array into meaningful movement variables
        double forward = instructions[0];
        double sideways = instructions[1];
        double turn = instructions[2];

        //translates movement variables into motor speeds.
        //motors run same direction to go forward.
        //left motors and right motors run opposite directions to turn.
        //front motors and back motors run opposite directions to go sideways.
        double frontL = forward + turn - sideways;
        double frontR = forward - turn - sideways;
        double backL = forward + turn + sideways;
        double backR = forward - turn + sideways;

        SpeedQuadruple speeds = new SpeedQuadruple(frontL, frontR, backL, backR);
        return speeds;
    }
}
