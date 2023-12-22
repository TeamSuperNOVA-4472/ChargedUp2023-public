package frc.robot.autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

public class AutoBalance extends CommandBase{
   
    private Swerve mSwerve;
    private PIDController control = new PIDController(0.021, 0.0, 0.00015);

    public AutoBalance(Swerve pSwerve)
    {
        mSwerve = pSwerve;
        control.setTolerance(2);
        addRequirements(mSwerve);
    
    }

    @Override
    public void initialize()
    {
        control.setSetpoint(0);
    }

    @Override
    public void execute()
    {
        SmartDashboard.putNumber("Gyro", mSwerve.getGyro().getRoll());
        if(Math.abs(mSwerve.getGyro().getRoll()) > 0)
        {
            double out = -control.calculate(mSwerve.getGyro().getRoll());
            double x = MathUtil.clamp(out, -2, 2);
            mSwerve.drive(new Translation2d(x, 0), 0, false, false);
            SmartDashboard.putNumber("power", mSwerve.getGyro().getRoll());
            SmartDashboard.putNumber("Autobalance PID Output", out);
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        mSwerve.drive(new Translation2d(0, 0), 0.0, false, false);

    }

    @Override
    public boolean isFinished()
    {
        
        return control.atSetpoint();
    }
}
