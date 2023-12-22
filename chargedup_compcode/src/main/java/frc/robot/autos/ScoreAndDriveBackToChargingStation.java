package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.GoStraight;
import frc.robot.commands.RollerIntakeAuton;
import frc.robot.commands.ArmSetPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class ScoreAndDriveBackToChargingStation extends SequentialCommandGroup{

    public ScoreAndDriveBackToChargingStation(Swerve s_Swerve, Arm pArm, RollerIntake pIntake) {
        addCommands(
            new ScorePreloadAuton(s_Swerve, pArm, pIntake, -4.25, 5.5),
            new GoStraight(s_Swerve, 2, Units.inchesToMeters(70), 3),
            new AutoBalance(s_Swerve)
        );
    }
}
