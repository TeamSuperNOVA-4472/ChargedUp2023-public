package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;

public class PathPlannerCommand extends PPSwerveControllerCommand{
    
    Swerve drive;
    private static final PIDController leftController = new PIDController(20, 0, 0);
    private static final PIDController rightController = new PIDController(0, 0, 0);
    private static final PIDController angleController = new PIDController(0, 0, 0);
    static PathPlannerTrajectory trajectory;

    public PathPlannerCommand(String file, PathConstraints constraints, Swerve s) {
        this(getTrajectory(file, constraints), s);
    }

    public PathPlannerCommand(PathPlannerTrajectory traj, Swerve s) {
        super(
            traj,
            s::getPose,
            Constants.Swerve.swerveKinematics,
            leftController,
            rightController,
            angleController,
            s::setModuleStates,
            true,
            s
          );
          this.drive = s;
          trajectory = traj;
    }

    public static PathPlannerTrajectory getTrajectory(String file, PathConstraints constraints) {
        trajectory = PathPlanner.loadPath(file, constraints);
        return trajectory;
    }

    @Override
    public void initialize()
    {
        drive.resetOdometry(trajectory.getInitialPose());
        super.initialize();
    }

    @Override
    public void end(boolean interrupted){
        super.end(interrupted);
        drive.setModuleStates(new SwerveModuleState[] {new SwerveModuleState(0.0, new Rotation2d(0)), new SwerveModuleState(0.0, new Rotation2d(0)), new SwerveModuleState(0.0, new Rotation2d(0)), new SwerveModuleState(0.0, new Rotation2d(0)) });
    }

}
