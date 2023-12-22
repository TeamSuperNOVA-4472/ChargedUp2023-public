package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.GoStraight;
import frc.robot.commands.RollerIntakeAuton;
import frc.robot.commands.ArmSetPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.GyroTurn;

public class ScoreDriveGrabBalance extends SequentialCommandGroup{

    public ScoreDriveGrabBalance(Swerve s_Swerve, Arm pArm, RollerIntake pIntake) {
        addCommands(
            new SequentialCommandGroup(
                new ScorePreloadAuton(s_Swerve, pArm, pIntake, -0.685, 2),
                //new GyroTurn(s_Swerve, 180, 8.55, 2),
                new ParallelCommandGroup(
                    new PathPlannerAuto("GrabDatPiece", s_Swerve, 2.5, 1.5),
                    new SequentialCommandGroup(
                        new WaitCommand(1.975),

                        new ParallelCommandGroup(
                            new ArmSetPosition(0, -100, pArm, .9),
                            new RollerIntakeAuton(pIntake, -0.95, 1.75)
                        ),
                        new ArmSetPosition(0, 0, pArm, .9)
                        
                    )
                ),
                new GoStraight(s_Swerve, 2, -Units.inchesToMeters(70), 3),
                new AutoBalance(s_Swerve)
            )
        );
    }
}
