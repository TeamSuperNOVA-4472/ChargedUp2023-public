package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Swerve;

public class GyroTurn extends CommandBase {

    private final Swerve mSwerveDriveSubsystem;

    private final ProfiledPIDController mProfiledPIDController;

    private double mTargetAngle = 0;
    private final double mDesiredAngle;
    private final Timer mTimer;
    private final double mTimeout;

    public GyroTurn(Swerve pSwerveDriveSubsystem, double pDesiredAngle, double pTimeout) {
        mSwerveDriveSubsystem = pSwerveDriveSubsystem;
        mProfiledPIDController = new ProfiledPIDController(.15, 0, 0.000, 
            new TrapezoidProfile.Constraints(
                0.5 * Constants.Swerve.maxAngularVelocity * 180 / Math.PI, 
                0.5 * Constants.Swerve.maxAngularVelocity * 180 / Math.PI));
        

        mProfiledPIDController.enableContinuousInput(0, 360);
        mDesiredAngle = pDesiredAngle;
        mTimer = new Timer();
        mTimeout = pTimeout;
        addRequirements(pSwerveDriveSubsystem);
    }

    public GyroTurn(Swerve pSwerveDriveSubsystem, double pDesiredAngle, double pSpeed, double pTimeout) {
        mSwerveDriveSubsystem = pSwerveDriveSubsystem;
        mProfiledPIDController =
        new ProfiledPIDController(.16, 0, 0.000, 
            new TrapezoidProfile.Constraints(
                0.5 * Units.radiansToDegrees(pSpeed), 
                0.5 * Units.radiansToDegrees(pSpeed)));
                
        mProfiledPIDController.enableContinuousInput(0, 360);
        mDesiredAngle = pDesiredAngle;
        mTimer = new Timer();
        mTimeout = pTimeout;
        addRequirements(pSwerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        double currentAngle = mSwerveDriveSubsystem.getPose().getRotation().getDegrees();
        mTargetAngle = (((currentAngle + mDesiredAngle) % 360) + 360) % 360;
        mProfiledPIDController.reset(currentAngle);
        mTimer.reset();
        mTimer.start();
    }

    @Override
    public void execute() {
        double currentAngle = ((mSwerveDriveSubsystem.getPose().getRotation().getDegrees() % 360) + 360) % 360;

        double output = mProfiledPIDController.calculate(currentAngle, mTargetAngle);
        double diff = Math.abs(mTargetAngle - currentAngle);
        double error = diff;

        if(diff > 180) {
            error = 360 - diff; 
        }

        SmartDashboard.putNumber("Profiled PID target", mTargetAngle);
        SmartDashboard.putNumber("Profiled PID current", currentAngle);
        SmartDashboard.putNumber("Profiled PID Error", error);
        SmartDashboard.putNumber("Profiled PID Output", output);

        if(error < Constants.AutoConstants.kGyroTurnToleranceDegs) {
            output = 0; 
        }

        mSwerveDriveSubsystem.drive(new Translation2d(0, 0),
            output, 
            false,
            false);
    }

    @Override
    public boolean isFinished() {
        return mProfiledPIDController.atGoal() || mTimer.get() >= mTimeout;
    }
}
