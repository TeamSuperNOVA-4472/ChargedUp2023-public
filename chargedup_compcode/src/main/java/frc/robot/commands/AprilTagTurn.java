package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;

public class AprilTagTurn extends CommandBase {
    private boolean targetSeen;

    private final ProfiledPIDController controller =         
    new ProfiledPIDController(.1, 0, 0.002, 
        new TrapezoidProfile.Constraints(
            0.275 * Constants.AutoConstants.kMetersPerSecondToDegreesPerSecond, 
            0.275 * Constants.AutoConstants.kMetersPerSecondToDegreesPerSecond));
    
    private final Swerve drive;
    private final VisionSubsystem vision;
    private final MedianFilter mFilter;
    private final double mTimeout;
    private final Timer mTimer;
    private boolean mFacingTarget;
    private double mCurrentAngleAway = 0;

    public AprilTagTurn(Swerve drive, VisionSubsystem vision, double timeout) {
        this.drive = drive;
        this.vision = vision;
        mFilter = new MedianFilter(5);
        mTimeout = timeout;
        mTimer = new Timer();
        controller.enableContinuousInput(-180, 180);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        targetSeen = vision.photonHasTargets();
        mFacingTarget = false;
        mCurrentAngleAway = targetSeen ?
            vision.getTrackedPhotonTarget().getYaw() : 0;
        controller.reset(mCurrentAngleAway);
        mFilter.reset();
        mTimer.start();
    }

    @Override
    public void execute() {
        if (targetSeen) {
            PhotonTrackedTarget bestTarget = vision.getTrackedPhotonTarget();
            mCurrentAngleAway = (bestTarget != null) ? bestTarget.getYaw() : 0;
            double filteredAngleToTarget = mFilter.calculate(mCurrentAngleAway);
    
            SmartDashboard.putNumber("Angle To Target", mCurrentAngleAway);
            SmartDashboard.putNumber("Filtered Angle To Target", filteredAngleToTarget);
            double rotationalVelocity = controller.calculate(filteredAngleToTarget, 0 + Constants.VisionConstants.kAprilTagOffsetDegs);
            double error = Math.abs(filteredAngleToTarget);
            if(error < 1.8) {
                rotationalVelocity = 0;
                mFacingTarget = true;
            }
            SmartDashboard.putNumber("Auto Rot Error", error);
            SmartDashboard.putNumber("Auto Rot", rotationalVelocity);

            // Include default drive logic to enable strafing while aiming
            drive.drive(new Translation2d(), rotationalVelocity, true, false);
        } else {
            targetSeen = vision.photonHasTargets();
        }
    }

    @Override
    public boolean isFinished() {
        return mTimeout < mTimer.get();
    }

    @Override
    public void end(boolean pInterrupted) {
        mTimer.stop();
        mTimer.reset();
    }
}