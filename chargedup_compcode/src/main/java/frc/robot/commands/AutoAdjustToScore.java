// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Swerve;

public class AutoAdjustToScore extends SequentialCommandGroup  {
  
  public AutoAdjustToScore(Swerve drive, VisionSubsystem vision) {
    addCommands(
      new AprilTagTurn(drive, vision, 2),
      new GetInRangeOfAprilTag(drive, vision, 2)
    );
    // Use addRequirements() here to declare subsystem dependencies.
  }
}
