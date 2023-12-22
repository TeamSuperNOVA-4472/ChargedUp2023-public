package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.controller.PIDController;

public class AutoBalance extends CommandBase{
   
    ExampleSubsystem drive;
    private PIDController control = new PIDController(0.03, 0, 0.0009);

    public AutoBalance(ExampleSubsystem s)
    {
        drive = s;
        addRequirements(s);
    }

    @Override
    public void initialize()
    {
        control.setSetpoint(0);
    }

    @Override
    public void execute()
    {
        SmartDashboard.putNumber("Gyro", drive.getGyro().getPitch());
        if(Math.abs(drive.getGyro().getPitch()) > 0)
        {
            double out = control.calculate(drive.getGyro().getPitch());
            double calcLeft = MathUtil.clamp(out, -0.3, 0.3);
            double calcRight = MathUtil.clamp(out, -0.4, 0.4) ;
            //calcLeft += Math.signum(calcLeft) * 0.1 + out;
            //calcRight += Math.signum(calcRight) * 0.1 + out;
            drive.move(calcLeft, calcRight);
            SmartDashboard.putNumber("power", drive.getGyro().getPitch());
            SmartDashboard.putNumber("Left Speed ", calcLeft);
            SmartDashboard.putNumber("Right Speed", calcRight);
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        drive.move(0,0);
    }

    @Override
    public boolean isFinished()
    {
        
        return control.atSetpoint();//Math.abs(drive.getGyro().getPitch()) < 1;
    }
}
