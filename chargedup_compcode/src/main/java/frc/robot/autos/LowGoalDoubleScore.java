package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;

import frc.robot.Constants;
import frc.robot.commands.RollerIntakeAuton;
import frc.robot.commands.ArmSetPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathPlannerCommand;

public class LowGoalDoubleScore extends SequentialCommandGroup{
    public LowGoalDoubleScore(Swerve s_Swerve, Arm pArm, RollerIntake pIntake) {
        addCommands(
            new InstantCommand(()->pArm.reset(), pArm),
            new ParallelCommandGroup(
                new ArmSetPosition(Constants.Arm.kShoulderLowGoal, 0, pArm, 2.0),
                new PathPlannerCommand("Realism Part 1", new PathConstraints(2, 1), s_Swerve),
                new RollerIntakeAuton(pIntake, -0.75, 2.5)
            ),
            new PathPlannerCommand("Realism Part 2", new PathConstraints(2, 1), s_Swerve),
            new RollerIntakeAuton(pIntake, 0.75, 1),
            new ParallelCommandGroup(
                new PathPlannerCommand("Realism Part 3", new PathConstraints(2, 1), s_Swerve),
                new RollerIntakeAuton(pIntake, -0.75, 3)
            )
        );
    }
}
