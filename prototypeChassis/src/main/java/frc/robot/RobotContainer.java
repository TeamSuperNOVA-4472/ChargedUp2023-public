package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.DriveTrainTeleoperated;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DrivetrainTeleop;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController mController = new XboxController(0);
  private final XboxController mPartner = new XboxController(1);
  
  private final DriveTrain mDriveTrain = new DriveTrain();
  private final DrivetrainTeleop mDriveTrainTeleoperated = new DrivetrainTeleop(mDriveTrain, mController);


    private final DrivetrainTeleop mDrivetrainTeleop;

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */
    public RobotContainer() {
        mDrivetrainTeleop = new DrivetrainTeleop(mDriveTrain, mController);
       
        mDriveTrain.setDefaultCommand(mDrivetrainTeleop);
        mDriveTrain.setDefaultCommand(mDriveTrainTeleoperated);
    }

  public Command getAutonomousCommand() {
    return null;
    // An ExampleCommand will run in autonomous
  }
}

