// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class GetInRangeOfAprilTag extends CommandBase {
  /** Creates a new GetInRangeOfAprilTag. */
  Swerve drive;
  VisionSubsystem vision;

  double range;
  double mTimeout;
  private final Timer mTimer;
  boolean targetsSeen;
  double translationalVelocity = 0;

  ProfiledPIDController controller = new ProfiledPIDController(Constants.Swerve.driveKP, Constants.Swerve.driveKP, Constants.Swerve.driveKP,
    new TrapezoidProfile.Constraints(
      0.5 * Constants.AutoConstants.kMetersPerSecondToDegreesPerSecond, 
      0.5 * Constants.AutoConstants.kMetersPerSecondToDegreesPerSecond)
  );


  public GetInRangeOfAprilTag(Swerve drive, VisionSubsystem vision, double timeout) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.vision = vision;

    mTimeout = timeout;
    
    mTimer = new Timer();
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetsSeen = vision.photonHasTargets();
    range = PhotonUtils.calculateDistanceToTargetMeters
    (
      Constants.VisionConstants.kCameraHeightMeters,
      Constants.VisionConstants.kTargetHeightMeters,
      Units.degreesToRadians(Constants.VisionConstants.kCameraPitchDegs),
      (vision.photonHasTargets()) ? vision.getTrackedPhotonTarget().getPitch() : 0
    );
  
    controller.setGoal(Constants.VisionConstants.kDesiredRangeMeters);
    mTimer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.photonHasTargets()) {
      range = PhotonUtils.calculateDistanceToTargetMeters
      (
        Constants.VisionConstants.kCameraHeightMeters,
        Constants.VisionConstants.kTargetHeightMeters,
        Units.degreesToRadians(Constants.VisionConstants.kCameraPitchDegs),
        (vision.photonHasTargets()) ? vision.getTrackedPhotonTarget().getPitch() : 0
      );

      translationalVelocity = -controller.calculate(range);

      drive.drive(new Translation2d(translationalVelocity, 0), 0, true, false);
    } else {
      translationalVelocity = 0;
      targetsSeen = vision.photonHasTargets();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mTimer.stop();
    mTimer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atGoal() || mTimer.get() < mTimeout;
  }
}
