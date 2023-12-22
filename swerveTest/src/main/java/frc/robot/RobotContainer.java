// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.autos.ScoreAndDriveBackToChargingStation;
import frc.robot.autos.PathPlannerAuto;
import frc.robot.commands.SwerveDriveAutoCommand;
import frc.robot.commands.SwerveDriveDefaultCommand;
import frc.robot.commands.TestProfiledPidCommand;
import frc.robot.commands.AprilTagTurn;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.AutoScore;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.GoStraight;
import frc.robot.commands.PathPlannerCommand;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveDriveSubsystem mSwerveDrive = new SwerveDriveSubsystem();

  private final VisionSubsystem mVisionSubsystem = new VisionSubsystem();

  private final XboxController mController = new XboxController(0);

  private final SwerveDriveDefaultCommand mSwerveDriveDefault = new SwerveDriveDefaultCommand(mSwerveDrive, mController);

  private final SwerveDriveAutoCommand mSwerveDriveAutoCommand = new SwerveDriveAutoCommand(mSwerveDrive);

  private final AutoAlign mAlign = new AutoAlign(mSwerveDrive, mVisionSubsystem, mController, 0);

  private final AutoScore mAutoScore = new AutoScore(mSwerveDrive, mVisionSubsystem);

  private final TestProfiledPidCommand mTestProfiledPidCommand = new TestProfiledPidCommand(mSwerveDrive);

  private final GoStraight mForward = new GoStraight(mSwerveDrive, 1, 1);

  private final PathPlannerAuto autoCommand = new PathPlannerAuto(mSwerveDrive);

  private final ScoreAndDriveBackToChargingStation autoForward=new ScoreAndDriveBackToChargingStation(mSwerveDrive);

  private final AprilTagTurn aprilTagTurn = new AprilTagTurn(mSwerveDrive, Double.MAX_VALUE);
  private final PathPlannerCommand strafeCommand = new PathPlannerCommand("Strafe", new PathConstraints(2, 1), mSwerveDrive);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    mSwerveDrive.setDefaultCommand(mSwerveDriveDefault);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    JoystickButton xButton = new JoystickButton(mController, XboxController.Button.kX.value);
    JoystickButton yButton = new JoystickButton(mController, XboxController.Button.kY.value);
    yButton.whileTrue(mForward);
    xButton.whileTrue(strafeCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    return autoForward;
  }
}
