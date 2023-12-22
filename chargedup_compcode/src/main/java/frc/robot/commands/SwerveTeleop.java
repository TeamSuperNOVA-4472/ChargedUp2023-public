package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.math.filter.SlewRateLimiter;


public class SwerveTeleop extends CommandBase {    
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier zeroGyroSupplier;
    private double mHeading = 0;
    private final PIDController mGyroController = new PIDController(0.15, 0, 0.00);
    private final SlewRateLimiter mFwdLimiter = new SlewRateLimiter(1);
    private final SlewRateLimiter mSideLimiter = new SlewRateLimiter(1);
    private final SlewRateLimiter mTurnLimiter = new SlewRateLimiter(1);

    public SwerveTeleop(
        Swerve s_Swerve,
        DoubleSupplier translationSup,
        DoubleSupplier strafeSup,
        DoubleSupplier rotationSup,
        BooleanSupplier robotCentricSup,
        BooleanSupplier zeroGyroSupplier) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.zeroGyroSupplier = zeroGyroSupplier;
        mGyroController.enableContinuousInput(0, 360);
    }

    @Override
    public void initialize() {
        s_Swerve.resetModulesToAbsolute();
        mHeading = s_Swerve.getYaw().getDegrees();
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
        double turnVal = turnJoyVal * Constants.Swerve.maxAngularVelocity;
        if(zeroGyroSupplier.getAsBoolean()) {
            s_Swerve.zeroGyro();
            mHeading = s_Swerve.getYaw().getDegrees();
        }

        if(turnJoyVal == 0 && (fwdJoyVal != 0 || sideJoyVal !=0)) {
            turnVal = mGyroController.calculate(s_Swerve.getYaw().getDegrees(), mHeading);
        } else {
            mHeading = s_Swerve.getYaw().getDegrees();
        }
        
        /* Drive */
        s_Swerve.drive(
            new Translation2d(fwdJoyVal, sideJoyVal).times(Constants.Swerve.maxSpeed), 
            turnVal, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}