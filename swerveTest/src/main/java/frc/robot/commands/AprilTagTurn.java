package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.util.DynamicTuner;

public class AprilTagTurn  extends CommandBase {
    private boolean targetSeen;
    private final DynamicTuner tuner;

    private final ProfiledPIDController mController;
    
    private final SwerveDriveSubsystem drive;
    private final MedianFilter mFilter;
    private final double mTimeout;
    private final Timer mTimer;
    private final PhotonCamera mVision;
    private boolean mFacingTarget;

    public AprilTagTurn(SwerveDriveSubsystem drive, double timeout) {
        this.drive = drive;
        this.tuner = DynamicTuner.getInstance();
        mVision = new PhotonCamera("Global_Shutter_Camera");
        mFilter = new MedianFilter(5);
        mTimeout = timeout;
        mTimer = new Timer();
        this.mController = new ProfiledPIDController(tuner.getAutoAlignP(), 0, tuner.getAutoAlignD(), 
        new TrapezoidProfile.Constraints(
            0.5 * Constants.METERS_PER_SECONDS_TO_DEGREES_PER_SECONDS, 
            0.5 * Constants.METERS_PER_SECONDS_TO_DEGREES_PER_SECONDS));

        mController.enableContinuousInput(-180, 180);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        targetSeen = false;
        mFacingTarget = false;
        double angle = mVision.getLatestResult().hasTargets() ?
            mVision.getLatestResult().getBestTarget().getYaw() : 0;
        mController.reset(angle);
        mController.setP(tuner.getAutoAlignP());
        mController.setD(tuner.getAutoAlignD());
        mFilter.reset();
        mTimer.start();
    }

    @Override
    public void execute() {
        System.out.printf("P: %.2f || D: %.2f\n", mController.getP(), mController.getD());
        PhotonPipelineResult vision = mVision.getLatestResult();

        if (targetSeen && vision.hasTargets()) {
            PhotonTrackedTarget bestTarget = vision.getBestTarget();
            double angleToTarget = bestTarget.getYaw();
            double filteredAngleToTarget = mFilter.calculate(angleToTarget);
    
            SmartDashboard.putNumber("Angle To Target", angleToTarget);
            SmartDashboard.putNumber("Filtered Angle To Target", filteredAngleToTarget);
            double rotationalVelocity = mController.calculate(filteredAngleToTarget, 0);
            double error = Math.abs(filteredAngleToTarget);
            if(error < 1.8) {
                rotationalVelocity = 0;
                mFacingTarget = true;
            }
            SmartDashboard.putNumber("Auto Rot Error", error);
            SmartDashboard.putNumber("Auto Rot", rotationalVelocity);

            // Include default drive logic to enable strafing while aiming
            drive.driveFieldOriented(0, 0, rotationalVelocity, false);
        } else {
            targetSeen = vision.hasTargets();
        }
    }

    @Override
    public boolean isFinished() {
        return mFacingTarget || mTimeout < mTimer.get();
    }

    @Override
    public void end(boolean pInterrupted) {
        mTimer.stop();
        mTimer.reset();
    }
}
