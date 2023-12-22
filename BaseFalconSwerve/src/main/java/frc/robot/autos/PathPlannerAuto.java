package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathPlannerCommand;
import frc.robot.subsystems.Swerve;

public class PathPlannerAuto extends SequentialCommandGroup{
    public PathPlannerAuto(Swerve swerve)
    {
        addCommands(
            new PathPlannerCommand("New Path" ,new PathConstraints(2, 1), swerve)
        );
    }
}
