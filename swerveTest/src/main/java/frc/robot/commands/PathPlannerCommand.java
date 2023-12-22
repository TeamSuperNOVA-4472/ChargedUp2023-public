package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class PathPlannerCommand extends PPSwerveControllerCommand{
    
    //Creting PID controllers and declaring trajectory and subsystem variables
    private static final PIDController mXController = new PIDController(1.0, 0, 0);
    private static final PIDController mYController = new PIDController(1.0, 0, 0);
    private static final PIDController mAngleController = new PIDController(8, 0, 0);

    private final PathPlannerTrajectory mTrajectory;
    private SwerveDriveSubsystem mDrive;

    //Generate a PathPlannerCommand given a file, constraints and subsystem
    //NOTE: file is the first part (ex. Line.wpilib.json would just be passed in as "Line")
    public PathPlannerCommand(String pFile, PathConstraints pConstraints, SwerveDriveSubsystem pS) {
        this(getTrajectory(pFile, pConstraints), pS);
    }

    //Generate a PathPlannerCommand given a trajectory and subsystem
    public PathPlannerCommand(PathPlannerTrajectory traj, SwerveDriveSubsystem pS) {
        super(
            traj,
            pS::getPose,
            pS.getSwerveKinematics(),
            mXController,
            mYController,
            mAngleController, 
            pS::setModuleStates,
            true,
            pS
        );
        mTrajectory = traj;
        mDrive = pS;
    }

    //Wrapper function to shorten the first constructor
    public static PathPlannerTrajectory getTrajectory(String pFile, PathConstraints pConstraints) {
        return PathPlanner.loadPath(pFile, pConstraints);
    }

    //Resets the odometry to the start of the path, then runs the PPSwerveControllerCommand initialize
    @Override
    public void initialize() {
        PathPlannerTrajectory transformed =
            PathPlannerTrajectory
                .transformTrajectoryForAlliance(mTrajectory, DriverStation.getAlliance());
        mDrive.resetOdometry(transformed.getInitialHolonomicPose());
        super.initialize();
    }

    //After running PPSwerveControllerCommand's end method, stop the swerve drive
    @Override
    public void end(boolean pInterrupted){
        super.end(pInterrupted);
        mDrive.setModuleStates(new SwerveModuleState[]{
            new SwerveModuleState(0, new Rotation2d(0)),
            new SwerveModuleState(0, new Rotation2d(0)),
            new SwerveModuleState(0, new Rotation2d(0)),
            new SwerveModuleState(0, new Rotation2d(0))
        });
    }

}
