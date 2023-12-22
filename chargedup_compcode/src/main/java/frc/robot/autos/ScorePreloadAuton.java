package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.GoStraight;
import frc.robot.commands.RollerIntakeAuton;
import frc.robot.commands.ArmSetPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.RollerIntake;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScorePreloadAuton extends SequentialCommandGroup{

    public ScorePreloadAuton(Swerve s_Swerve, Arm pArm, RollerIntake pIntake, double pGoBack, double pTime) {
        addCommands(
            new InstantCommand(()->{
                pArm.reset();
                s_Swerve.resetModulesToAbsolute();
                s_Swerve.setGyroOffset(180);
            }, pArm),
            new ParallelCommandGroup(
                new GoStraight(s_Swerve,1.5, 0.5, 1.6), //TODO: Experimental! The time was 2
                new ArmSetPosition(Constants.Arm.kShoulderHighGoal, -65, pArm, 0.75)
            ),
            new ArmSetPosition(Constants.Arm.kShoulderHighGoal, -183, pArm, 0.85), //TODO: Experimental! The time was 1.75
            new ParallelCommandGroup(
                new GoStraight(s_Swerve, 1.0, pGoBack, pTime), 
                new SequentialCommandGroup(
                    new RollerIntakeAuton(pIntake, 0.75, 0.75),
                    new ArmSetPosition(0, 0, pArm, 1)
                )
            )
        );
    }
}
