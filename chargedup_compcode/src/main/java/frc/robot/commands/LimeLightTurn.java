package frc.robot.commands;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.filter.MedianFilter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class LimeLightTurn extends CommandBase {
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

    public LimeLightTurn(Swerve drive, VisionSubsystem vision, double timeout) {
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
        targetSeen = false;
        mFacingTarget = false;
        controller.reset(vision.getAngleToTarget());
        mFilter.reset();
        mTimer.start();
    }

    @Override
    public void execute() {
        double angleToTarget = targetSeen ? vision.getAngleToTarget() : 0;
        double filteredAngleToTarget = mFilter.calculate(angleToTarget);

        SmartDashboard.putNumber("Angle To Target", angleToTarget);
        SmartDashboard.putNumber("Filtered Angle To Target", filteredAngleToTarget);

        if (targetSeen) {
            double rotationalVelocity = controller.calculate(filteredAngleToTarget, 0 + Constants.VisionConstants.kRetroreflectiveOffsetDegs);
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
            targetSeen = vision.targetsPresent();
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