package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.math.filter.SlewRateLimiter;


public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private final SlewRateLimiter mFwdLimiter = new SlewRateLimiter(1);
    private final SlewRateLimiter mSideLimiter = new SlewRateLimiter(1);
    private final SlewRateLimiter mTurnLimiter = new SlewRateLimiter(1);

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void initialize() {
        s_Swerve.resetModulesToAbsolute();
        s_Swerve.zeroGyro();
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        double fwdJoyVal = Math.signum(translationVal) * Constants.CONTROLLER_PROFILE_MAP.get(Math.abs(translationVal)); //Math.pow(Math.abs(joyLeftY), 2);
        double sideJoyVal = Math.signum(strafeVal) * Constants.CONTROLLER_PROFILE_MAP.get(Math.abs(strafeVal));//Math.pow(Math.abs(joyLeftX), 2);
        double turnJoyVal = Math.signum(rotationVal) * Constants.CONTROLLER_PROFILE_MAP.get(Math.abs(rotationVal));

        fwdJoyVal = mFwdLimiter.calculate(fwdJoyVal);
        sideJoyVal = mSideLimiter.calculate(sideJoyVal);
        turnJoyVal = mTurnLimiter.calculate(turnJoyVal);
        
        /* Drive */
        s_Swerve.drive(
            new Translation2d(fwdJoyVal, sideJoyVal).times(Constants.Swerve.maxSpeed), 
            turnJoyVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}