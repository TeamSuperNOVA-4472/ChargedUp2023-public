package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
    private static final CTREConfigs kCtreConfigs = new CTREConfigs();
    private static final SimpleMotorFeedforward kFeedForward =
        new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    private final int mModuleNumber;
    private final Rotation2d mAngleOffset;
    private final TalonFX mAngleMotor;
    private final TalonFX mDriveMotor;
    private final CANCoder mAngleEncoder;

    /**
     * Utility method that takes any angle and bounds it to a 0 to 360 range.
     * @param pUnboundedAngle The unbounded angle to convert.
     * @return The converted angle bounded to a 0 to 360 range.
     */
    private static double getBoundedAngle(double pUnboundedAngle) {
        return (pUnboundedAngle % 360 + 360) % 360;
    }

    public SwerveModule(int pModuleNumber, SwerveModuleConstants pModuleConstants){
        this.mModuleNumber = pModuleNumber;
        this.mAngleOffset = pModuleConstants.angleOffset;
        
        /* Angle Encoder Config */
        mAngleEncoder = new CANCoder(pModuleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(pModuleConstants.angleMotorID);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(pModuleConstants.driveMotorID);
        configDriveMotor();

    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, kFeedForward.calculate(desiredState.speedMetersPerSecond));
        } 
    }

    private void setAngle(SwerveModuleState desiredState){
        boolean isMoving = Math.abs(desiredState.speedMetersPerSecond) >= 0.1;
        if (isMoving) {
            // Get current and desired angles
            double currentRawAngle = getRawAngle();
            double currentAngle = getBoundedAngle(currentRawAngle);
            double targetAngle = getBoundedAngle(desiredState.angle.getDegrees());
            
            // Evaluate offset by taking difference from target and current angles along with
            // its opposite signed offset
            double offset = targetAngle - currentAngle;
            double oppOffset = offset > 0 ? offset - 360 : offset + 360;

            // Choose the offset with the smallest absolute value and add it to the current raw angle
            // to use as the new angle for the angle motor position setpoint
            double chosenOffset = Math.abs(offset) < Math.abs(oppOffset) ? offset : oppOffset;
            double newAngle = currentRawAngle + chosenOffset;
            mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(newAngle, Constants.Swerve.angleGearRatio));
        } else {
            mAngleMotor.set(ControlMode.PercentOutput, 0.0);
        }
    }

    private Rotation2d getAngle(){
        double angle = getBoundedAngle(getRawAngle());
        return Rotation2d.fromDegrees(angle);
    }

    private double getRawAngle() {
        return Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(mAngleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - mAngleOffset.getDegrees(), Constants.Swerve.angleGearRatio);
        mAngleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        mAngleEncoder.configFactoryDefault();
        mAngleEncoder.configAllSettings(kCtreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        mAngleMotor.configFactoryDefault();
        mAngleMotor.configAllSettings(kCtreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
    }

    private void configDriveMotor(){        
        mDriveMotor.configFactoryDefault();
        mDriveMotor.configAllSettings(kCtreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        mDriveMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState() {
        double swerveMPS =
            Conversions.falconToMPS(
                mDriveMotor.getSelectedSensorVelocity(),
                Constants.Swerve.wheelCircumference,
                Constants.Swerve.driveGearRatio);
    
        return new SwerveModuleState(swerveMPS, getAngle()); 
    }

    public SwerveModulePosition getPosition() {
        double swerveMeters =
            Conversions.falconToMeters(
                mDriveMotor.getSelectedSensorPosition(),
                Constants.Swerve.wheelCircumference,
                Constants.Swerve.driveGearRatio);

        return new SwerveModulePosition(swerveMeters, getAngle());
    }

    public int getModuleNumber() {
        return mModuleNumber;
    }
}
