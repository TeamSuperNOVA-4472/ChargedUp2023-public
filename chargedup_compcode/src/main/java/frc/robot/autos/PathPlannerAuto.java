package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathPlannerCommand;
import frc.robot.subsystems.Swerve;

public class PathPlannerAuto extends SequentialCommandGroup{
    public PathPlannerAuto(String file, Swerve swerve){
        addCommands(
            new PathPlannerCommand(file, new PathConstraints(2, 1), swerve)
        );
    }

    public PathPlannerAuto(String file, Swerve swerve, double maxVel, double maxAccel) {
        addCommands(
            new PathPlannerCommand(file, new PathConstraints(maxVel, maxAccel), swerve)
        );
    }
}