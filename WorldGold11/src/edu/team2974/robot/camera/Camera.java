package edu.team2974.robot.camera;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;
import edu.wpi.first.wpilibj.image.NIVisionException;

/**
 * @author andrey
 * The Camera class contains all necessary components of the camera and handles
 * all the processing necessary to locate a target.(to be expanded)
 */
public class Camera
{

    /**
     * The  servos of the camera are simple motors that can change the 
     * orientation of the camera (tilt and direction) and are programmed 
     * in the API.
     */
    private Servo cameraTilt;
    private Servo cameraPan;
    /**
     * The camera itself relies on the basic provided API and this class is 
     * used to have the few methods actually needed that incorporate and use 
     * the provided methods.
     */
    public AxisCamera camera;
    private BinaryImage lastPinDetection;
    private static final String SAVE_LOCATION = "savedImages\\";//"C:\\savedImages\\"

    /**
     * The constructor for the camera, which instantiates the camera components 
     * and sets the default setting for the camera.
     * @param tiltChannel currently unused
     * @param panChannel currently unused
     */
    public Camera(int tiltChannel, int panChannel) {
        // cameraPan = new Servo(PanChannel);
        //cameraTilt= new Servo(TiltChannel);
        camera = AxisCamera.getInstance();//Replacement for initialization for the camera.
        camera.writeResolution(AxisCamera.ResolutionT.k160x120);
        camera.writeBrightness(50);
        //camera.writeMaxFPS(10);
        camera.writeCompression(0);
    }
    
    /**
     * Gets the current Axis camera instance and initializes it with some
     * reasonable defaults.
     */
    public Camera() {
        camera = AxisCamera.getInstance();//Replacement for initialization for the camera.
        camera.writeResolution(AxisCamera.ResolutionT.k160x120);
        camera.writeBrightness(50);
        //camera.writeMaxFPS(10);
        camera.writeCompression(0);
    }


    /**
     * Empty implementation.
     * @param tilt
     * @param pan
     */
    public void moveCamera(double tilt, double pan) {
        //cameraTilt.set(pan/2+0.5);
        //cameraPan.set(tilt*tilt*tilt/2+0.5);
    }

    /**
     * Writes the current image to a file with the given name in the 
     * savedImages folder
     * @param fileName
     */
    public void saveCurrentImage(String fileName) {
        ColorImage currentImage;
        try {
            currentImage = camera.getImage();
            currentImage.write(SAVE_LOCATION + fileName + ".jpg");
        } catch (AxisCameraException ex) {
            System.out.println("not getting image - axis camera exception");
        } catch (NIVisionException ex) {
            System.out.println("not getting image - NIVision exception");
        }

    }

    /**
     *
     * @param fileName
     */
    public void saveCurrentBinaryLuminance(String fileName) {
        ColorImage currentImage;
        try {
            currentImage = camera.getImage();
            currentImage.thresholdHSL(55, 85, 19, 91, 186, 255);
            currentImage.write(SAVE_LOCATION + fileName + ".jpg");
        } catch (AxisCameraException ex) {
            System.out.println("not getting image - axis camera exception");
        } catch (NIVisionException ex) {
            System.out.println("not getting image - NIVision exception");
        }

    }

    /**
     *
     * @return
     */
    public ParticleAnalysisReport[] getPinLocations() {
        ParticleAnalysisReport[] orderedPinReports = new ParticleAnalysisReport[0];
        try {
            ColorImage currentImage = camera.getImage();
            BinaryImage pins = currentImage.thresholdHSL(55, 85, 19, 91, 186,
                    255);
            orderedPinReports = pins.getOrderedParticleAnalysisReports();
        } catch (AxisCameraException ex) {
            System.out.println("not getting image - axis camera exception");
        } catch (NIVisionException ex) {
            System.out.println("not getting image - NIVision exception");
        }
        return orderedPinReports;

    }

    /**
     *
     * @return
     */
    public boolean blueSeen() {
        boolean seen = false;;
        try {
            ColorImage currentImage = camera.getImage();
            BinaryImage pins = currentImage.thresholdRGB(0, 0, 0, 0, 50, 200);
            ParticleAnalysisReport[] orderedPinReports = pins.getOrderedParticleAnalysisReports(
                    1);
            currentImage.free();
            if (orderedPinReports[0].particleToImagePercent > 10.0) {
                seen = true;
            }
        } catch (AxisCameraException ex) {
            System.out.println("not getting image - axis camera exception");
        } catch (NIVisionException ex) {
            System.out.println("not getting image - NIVision exception");
        }
        return seen;

    }
}
