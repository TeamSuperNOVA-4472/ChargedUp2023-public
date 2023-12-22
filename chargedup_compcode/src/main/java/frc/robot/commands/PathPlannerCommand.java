package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;

public class PathPlannerCommand extends PPSwerveControllerCommand {
    
    private static final PIDController xController = new PIDController(4, 0, 0);
    private static final PIDController yController = new PIDController(4, 0, 0);
    private static final PIDController angleController = new PIDController(2, 0, 0);

    private final Swerve drive;
    private final PathPlannerTrajectory trajectory;

    public PathPlannerCommand(String file, PathConstraints constraints, Swerve s) {
        this(getTrajectory(file, constraints), s);
    }

    public PathPlannerCommand(PathPlannerTrajectory traj, Swerve s) {
        super(
            traj,
            s::getPose,
            Constants.Swerve.swerveKinematics,
            xController,
            yController,
            angleController,
            s::setModuleStates,
            true,
            s
          );
          this.drive = s;
          trajectory = traj;
    }

    public static PathPlannerTrajectory getTrajectory(String file, PathConstraints constraints) {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(file, constraints);
        return trajectory;
    }

    @Override
    public void initialize()
    {
        PathPlannerTrajectory transformed =
            PathPlannerTrajectory
                .transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance());
        drive.resetOdometry(transformed.getInitialHolonomicPose());
        super.initialize();
    }

    @Override
    public void end(boolean interrupted){
        super.end(interrupted);
        drive.setModuleStates(new SwerveModuleState[] {new SwerveModuleState(0.0, new Rotation2d(0)), new SwerveModuleState(0.0, new Rotation2d(0)), new SwerveModuleState(0.0, new Rotation2d(0)), new SwerveModuleState(0.0, new Rotation2d(0)) });
    }

}
