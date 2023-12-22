package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class BalanceCommand extends CommandBase {
    SwerveDriveSubsystem mDriveSubsystem;

    
    public BalanceCommand (SwerveDriveSubsystem pSwerveDriveSubsystem) {
        mDriveSubsystem = pSwerveDriveSubsystem;
        addRequirements(pSwerveDriveSubsystem);
    }

    @Override
    public void execute() {
        if(Math.abs(mDriveSubsystem.getGyro().getYaw()) > 0)
            mDriveSubsystem.driveRobotOriented(mDriveSubsystem.getGyro().getYaw()/180, 0.0, 0.0, false);
    }

    @Override
    public void end(boolean interrupted) {
        mDriveSubsystem.driveRobotOriented(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        return mDriveSubsystem.getGyro().getPitch() == 0;
    }
}
