package edu.wpi.first.wpilibj.templates;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;
import edu.wpi.first.wpilibj.image.NIVisionException;
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
    private BinaryImage lastPinDetection;

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
     camera = AxisCamera.getInstance();
     camera.writeResolution(AxisCamera.ResolutionT.k160x120);
     camera.writeBrightness(50);
     camera.writeMaxFPS(10);
     camera.writeCompression(0);

    }

      public Camera()
    {

     camera=AxisCamera.getInstance();//Replacement for initialization for the camera.
     camera = AxisCamera.getInstance();
     camera.writeResolution(AxisCamera.ResolutionT.k160x120);
     camera.writeBrightness(50);
     camera.writeMaxFPS(10);
     camera.writeCompression(0);

    }

    public void saveCurrentImage(String fileName)
    {
        ColorImage currentImage;
        try{
        currentImage=camera.getImage();
        currentImage.write("C:\\savedImages\\"+fileName+".jpg");
        }
        catch (AxisCameraException ex)
        {
            System.out.println("not getting image - axis camera exception");
        }
        catch (NIVisionException ex)
        {
           System.out.println("not getting image - NIVision exception");
        }

    }

    public void saveCurrentBinaryLuminance(String fileName)
    {
        ColorImage currentImage;
        try{
        currentImage=camera.getImage();
        currentImage.thresholdHSL(55, 85, 19, 91, 186, 255);
        currentImage.write("C:\\savedImages\\"+fileName+".jpg");
        }
        catch (AxisCameraException ex)
        {
            System.out.println("not getting image - axis camera exception");
        }
        catch (NIVisionException ex)
        {
           System.out.println("not getting image - NIVision exception");
        }

    }

    public ParticleAnalysisReport[] getPinLocations()
    {
        ParticleAnalysisReport[] orderedPinReports=new ParticleAnalysisReport[0];
        try{
        ColorImage currentImage=camera.getImage();
        BinaryImage pins=currentImage.thresholdHSL(55, 85, 19, 91, 186, 255);
        orderedPinReports=pins.getOrderedParticleAnalysisReports();
        }
        catch (AxisCameraException ex)
        {
            System.out.println("not getting image - axis camera exception");
        }
        catch (NIVisionException ex)
        {
           System.out.println("not getting image - NIVision exception");
        }
       return orderedPinReports;

    }


}
