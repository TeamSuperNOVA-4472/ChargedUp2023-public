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

public class TwoScoreOther extends SequentialCommandGroup{
    public TwoScoreOther(Swerve s_Swerve, Arm pArm, RollerIntake pIntake){
        addCommands(
            new ScorePreloadAuton(s_Swerve, pArm, pIntake, -1, 2),
            new ArmSetPosition(0, -100, pArm, .9),
            new ParallelCommandGroup(
            //new ArmSetPosition(0, -100, pArm, .9),
            new PathPlannerAuto("twotwo", s_Swerve, 4, 3.5),
            new SequentialCommandGroup(
                new RollerIntakeAuton(pIntake, -0.95, 2.5),
                new ArmSetPosition(0, 0, pArm, 1),
                new WaitCommand(1),
                new ArmSetPosition(Constants.Arm.kShoulderHighGoal, -190, pArm, 1.5),
                new WaitCommand(0.3),
                new RollerIntakeAuton(pIntake, 0.5, 0.8),
                new ArmSetPosition(0, 0, pArm, 1)
            )
            ),
            new AutoBalance(s_Swerve)
            /* 
            new ParallelCommandGroup(
                //new PathPlannerAuto("twotwo", s_Swerve, 4, 4),
                new SequentialCommandGroup(
                    //new WaitCommand(0.2),
                   // new ArmSetPosition(0, -100, pArm, .9),
                    //new RollerIntakeAuton(pIntake, -0.95, 2),
                    //new ArmSetPosition(0, 0, pArm, 1),
                    //new WaitCommand(1.3),
                    //new ArmSetPosition(Constants.Arm.kShoulderHighGoal, -190, pArm, 1.5)
                )
            ),*/
        );
    }
}