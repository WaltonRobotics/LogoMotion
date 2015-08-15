package edu.wpi.first.wpilibj.templates;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.camera.AxisCameraException;

/**
 * @author andrey
 * The Camera class contains all necessary components of the camera and handles
 * all the processing necessary to locate a target.(to be expanded)
 */
public class Camera
{
    /**
     * The  servos of the camera are simple motors that can change the orientation of the camera
     * (tilt and direction) and are programmed in the API.
     */
    private Servo cameraTiltServo;
    /**
     * PWMs are the superclass of servos and speed controllers. They are created here because they were last year
     * and are presumingly needed, although further testing and examination of hardware itself is needed.
     */
    private PWM cameraBase;
    private PWM cameraPanMotor;
    /**
     * The camera itself relies on the basic provided API and this class is used to have the few methods actually needed
     * that incorporate and use the provided methods.
     */
    private AxisCamera camera;

    /**
     * The constructor for the camera, which instantiates the camera components and sets the default setting for the camera.
     * @param baseChannel the channel for the base motor to be used in the constructor.
     * @param PanMotorChannel the channel for the pan motor to be used in the constructor.
     * @param servoChannel the servo channel to be used in the constructor.
     */
    public Camera(int baseChannel,int PanMotorChannel,int servoChannel)
    {
     cameraBase = new PWM(baseChannel);
     cameraPanMotor=new PWM(PanMotorChannel);
     cameraTiltServo= new Servo(servoChannel);
     camera=AxisCamera.getInstance();//Replacement for initialization for the camera.
     camera.writeResolution(AxisCamera.ResolutionT.k320x240);
     camera.writeBrightness(50);//Parameter is 0-100 brightness level, 100 being brightest, so 50 is assumed as medium.
    }

    public Camera()
    {
     camera=AxisCamera.getInstance();//Replacement for initialization for the camera.
     camera.writeResolution(AxisCamera.ResolutionT.k320x240);
     camera.writeBrightness(50);//Parameter is 0-100 brightness level, 100 being brightest, so 50 is assumed as medium.
    }


}


