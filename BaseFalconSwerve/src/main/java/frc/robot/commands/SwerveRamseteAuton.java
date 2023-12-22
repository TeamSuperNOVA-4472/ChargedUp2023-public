package frc.robot.commands;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class SwerveRamseteAuton extends CommandBase{
    private RamseteController mController;
    private Swerve mDrive;
    private Trajectory mTrajectory;
    private double prevTime;
    private Trajectory.State state;
    private SwerveDriveKinematics kinematics;
    private SwerveModuleState[] states;
    private Timer time;

    public SwerveRamseteAuton(Trajectory t, RamseteController c, Swerve s)
    {
        this.mController = c;
        this.mTrajectory = t;
        this.mDrive = s;
        addRequirements(s);
    }

    @Override 
    public void initialize()
    {
        prevTime = 0;
        state = mTrajectory.sample(0);
        double angularVelocity = state.velocityMetersPerSecond * state.curvatureRadPerMeter;
        double yCalc = state.velocityMetersPerSecond * Math.sin(angularVelocity);
        states = kinematics.toSwerveModuleStates(new ChassisSpeeds(state.velocityMetersPerSecond, yCalc, angularVelocity));
        time.reset();
        time.start();
    }

    @Override
    public void execute()
    {
        double currTime = time.get();

        var getNewSetpoint = mTrajectory.sample(currTime);
        var controllerOut = mController.calculate(mDrive.getPose(), getNewSetpoint);
        double y = controllerOut.vxMetersPerSecond * Math.sin(controllerOut.omegaRadiansPerSecond);
        ChassisSpeeds finalSpeeds = new ChassisSpeeds(controllerOut.vxMetersPerSecond, y, controllerOut.omegaRadiansPerSecond);
        states = kinematics.toSwerveModuleStates(finalSpeeds);

        mDrive.setModuleStates(states);
        this.prevTime = currTime;
    }

    @Override
    public void end(boolean interrupted)
    {
        time.stop();

        if(interrupted)
        {
            var zero = kinematics.toSwerveModuleStates(new ChassisSpeeds(0,0,0));
            mDrive.setModuleStates(zero);
        }
    }

    @Override
    public boolean isFinished()
    {
        return time.hasElapsed(mTrajectory.getTotalTimeSeconds());
    }
}
