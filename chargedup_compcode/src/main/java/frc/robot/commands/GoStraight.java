package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GoStraight extends CommandBase {
    
    private final Swerve drive;
    private final ProfiledPIDController controller;
    private final PIDController mGyroController;

    private final double mDistance;
    private final double[] initialPositions;
    private final double mSign;
    private final double mTimeout;
    private final Timer mTimer = new Timer();

    public GoStraight(Swerve driveSubsystem, double speed, double distance, double pTimeout) {
        drive = driveSubsystem;
        controller = new ProfiledPIDController(2, 0, 0,
            new TrapezoidProfile.Constraints(speed, 1.0));
        mGyroController = new PIDController(0.15, 0, 0.00);
        mGyroController.enableContinuousInput(0, 360);
        mDistance = distance;
        initialPositions = new double[driveSubsystem.getModulePositions().length];
        mSign = distance < 0 ? -1 : 1;
        mTimeout = pTimeout;
        addRequirements(drive);
    }

    public double getError() {
        return mDistance - getDistanceTraveled();
    }

    public double getDistanceTraveled() {
        double total = 0;
        double avgDistance;
        SwerveModulePosition[] currentPositions = drive.getModulePositions();
        for (int i = 0; i < currentPositions.length; i ++) {
            double moduleDistTraveled = Math.abs(currentPositions[i].distanceMeters - initialPositions[i]);
            total += moduleDistTraveled;
        }
        avgDistance = total / (double) drive.getModulePositions().length;
        return mSign * avgDistance;
    }

    @Override
    public void initialize() {
        for (int i = 0; i < drive.getModulePositions().length; i++) {
            initialPositions[i] = drive.getModulePositions()[i].distanceMeters;
        }
        controller.reset(0);
        controller.setGoal(mDistance);
        mGyroController.setSetpoint(drive.getYaw().getDegrees());
        mTimer.reset();
        mTimer.start();
    }

    @Override
    public void execute() {
        double errorDist = getError() / mDistance;
        double turn = mGyroController.calculate(drive.getYaw().getDegrees());
        double speed = controller.calculate(getDistanceTraveled());
        drive.drive(new Translation2d(speed, 0), turn, false, false);
        SmartDashboard.putNumber("Drive forward error", errorDist);
        SmartDashboard.putNumber("current speed", speed);
    }

    @Override
    public boolean isFinished() {
        return mTimer.get() >= mTimeout;
    }

    @Override
    public void end(boolean pInterruped) {
        mTimer.stop();
        drive.drive(new Translation2d(0, 0), 0, false, false);
    }
}