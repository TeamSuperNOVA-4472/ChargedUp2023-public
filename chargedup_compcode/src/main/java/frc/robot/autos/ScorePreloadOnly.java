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


public class ScorePreloadOnly extends SequentialCommandGroup {

    public ScorePreloadOnly(Swerve s_Swerve, Arm pArm, RollerIntake pIntake) {
        addCommands(
            new InstantCommand(()->{
                pArm.reset();
                s_Swerve.resetModulesToAbsolute();
                s_Swerve.setGyroOffset(180);
            }, pArm),
            new ParallelCommandGroup(
                new GoStraight(s_Swerve,1.5, 0.5, 2),
                new ArmSetPosition(Constants.Arm.kShoulderHighGoal, -65, pArm, 0.75)  //placeholder for auto score
            ),
            new ArmSetPosition(Constants.Arm.kShoulderHighGoal, -180, pArm, 1.75),
            new ParallelCommandGroup(
                new GoStraight(s_Swerve, 1.0, -0.6, 5), 
                new SequentialCommandGroup(                  
                    new RollerIntakeAuton(pIntake, 0.75, 0.75),
                    new ArmSetPosition(0, 0, pArm, 1)
                )
            )
        );
    }
    
}
