package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PathPlannerCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

/**
 * The autonomous controller that uses the created PathPlanner paths.
 */
public class PathPlannerAuto extends SequentialCommandGroup {
    private final PathConstraints mPathConstraints;
    private final SwerveDriveSubsystem mSwerveDrive;

    /**
     * Set up the autonomous path. Call the method that sets up the desired
     * path and events.
     */
    public PathPlannerAuto(SwerveDriveSubsystem swerve) {
        mPathConstraints = new PathConstraints(2, 1);
        mSwerveDrive = swerve;
        setupTestAuto();
    }

    /**
     * Set up a test autonomous with basic events.
     */
    private void setupTestAuto() {
        PathPlannerCommand path = new PathPlannerCommand(
            "Test Autonomous", mPathConstraints, mSwerveDrive);
        

        addCommands(path);
    }
}
