package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.RollerIntakeAuton;
import frc.robot.commands.ArmSetPosition;
import frc.robot.commands.GoStraight;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Two extends SequentialCommandGroup{
    public Two(Swerve s_Swerve, Arm pArm, RollerIntake pIntake){
        addCommands(
            new ScorePreloadAuton(s_Swerve, pArm, pIntake, -1, 2),
            new ParallelCommandGroup(
                new PathPlannerAuto("Realisim High", s_Swerve),
                new SequentialCommandGroup(
                    new WaitCommand(2),
                    new ArmSetPosition(0, -100, pArm, .9),
                    new RollerIntakeAuton(pIntake, -0.75, 1.3),
                    new ArmSetPosition(0, 0, pArm, 1),
                    new WaitCommand(1.3),
                    new ArmSetPosition(Constants.Arm.kShoulderHighGoal, -190, pArm, 1.5)
                )
            ),
            new ParallelCommandGroup( //Experimental! Used to be that all of these commands would run in sequence.
                new GoStraight(s_Swerve, 1, -1.5, 2),
                new SequentialCommandGroup(
                    new RollerIntakeAuton(pIntake, 0.5, 0.8),
                    new ArmSetPosition(0, 0, pArm, 0.6)
                )
            )
        );
    }
}