package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTable;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {
    /* Limelight */
    private NetworkTableEntry mDeltaX;
    private NetworkTableEntry mDeltaY;
    private NetworkTableEntry mTargetArea;
    private NetworkTableEntry mTargetsPresent;

    /* Photonvision */
    private PhotonCamera cam;
    
    /*
     * TableEntry values:
     *  tx -> The horizontal offset from the crosshair to the vision target
     *  ty -> The vertical offset from the crosshair to the vision target
     *  ta -> The target area / percentage that the target takes up of the image
     *  tv -> A binary value representing if the Limelight has any valid targets
     */

    public VisionSubsystem() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-nova");
        mDeltaX = table.getEntry("tx");
        mDeltaY = table.getEntry("ty");
        mTargetArea = table.getEntry("ta");
        mTargetsPresent = table.getEntry("tv");

        cam = new PhotonCamera("Global_Shutter_Camera");
    }

    public double getXDegsFromTarget() {
        return mDeltaX.getDouble(0);
    }

    public double getYDegsFromTarget() {
        return mDeltaY.getDouble(0);
    }


    /*
     * Returns either true or false depending on the value of the tv entry
     * 0 -> No valid targets discovered
     * 1 -> Valid target is present
     */
    public boolean targetsPresent() {
        return mTargetsPresent.getDouble(0) > 0;
    }
    
    public double getTargetArea() {
        return mTargetArea.getDouble(0);
    }

    /*
     * Get the angle relative to the target
     * 
     *  @return the current drivetrain rotation plus the tx entry value
     */
    public double getAngleToTarget() {
        return mDeltaX.getDouble(0);
    }

    /* Photonvision methods */

    /*
     * Gets the lastest result from a Photonvision snapshot
     * 
     * @return The lastest image from Photonvision
     */
    public PhotonPipelineResult getLastestPhotonResult() {
        return cam.getLatestResult();
    }

    /*
     * Returns true or false depending on whether or not there is a target present in the latest image;
     * 
     * @return True or false for whether or not a target is present.
     */
    public boolean photonHasTargets() {
        return getLastestPhotonResult().hasTargets();
    }


    /*
     * Get a target from the latest Photovision pipeline result
     * 
     * @return PhotonTrackedTarget representing the target that best fits the given contour
     */
    public PhotonTrackedTarget getTrackedPhotonTarget() {
        return getLastestPhotonResult().getBestTarget();
    }


}