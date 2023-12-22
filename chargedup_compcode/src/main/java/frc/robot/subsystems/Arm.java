package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

/**
 * This subsystem controls the rotation and speed of the arm.
 */
public class Arm extends SubsystemBase {
    
    private final CANSparkMax mShoulderMotorMain;
    private final CANSparkMax mShoulderFollower;
    private final DutyCycleEncoder mShoulderEncoder;

    private final CANSparkMax mWristMotorMain;
    private final CANSparkMax mWristFollower;
    private final DutyCycleEncoder mWristEncoder;

    private final ProfiledPIDController mShoulderPid;
    private final ProfiledPIDController mWristPid;
    private double mShoulderPosition = 0; 
    private double mWristPosition = -Constants.Arm.kWrist0ffsetMargin;


    public Arm() {
        // Assign subsystem components.
       
        // Set up the main shoulder motor and its follower.
        mShoulderMotorMain = new CANSparkMax(Constants.Arm.kShoulderMain, MotorType.kBrushless);
        mShoulderFollower = new CANSparkMax(Constants.Arm.kShoulderFollow, MotorType.kBrushless);
        mShoulderEncoder = new DutyCycleEncoder(Constants.Arm.kShoulderEncoderChannel);
        mShoulderMotorMain.setIdleMode(IdleMode.kBrake);
        mShoulderFollower.setIdleMode(IdleMode.kBrake);
        mShoulderMotorMain.enableVoltageCompensation(10);
        mShoulderMotorMain.setSmartCurrentLimit(30);
        mShoulderFollower.enableVoltageCompensation(10);
        mShoulderFollower.setSmartCurrentLimit(30);
        mShoulderEncoder.setPositionOffset(Constants.Arm.kShoulderOffsetDeg/Constants.Arm.kShoulderEncRotDeg);
        mShoulderEncoder.setDistancePerRotation(Constants.Arm.kShoulderEncRotDeg);
        mShoulderPid = new ProfiledPIDController(
            Constants.Arm.kShoulderP,
            Constants.Arm.kShoulderI,
            Constants.Arm.kShoulderD,
            new TrapezoidProfile.Constraints(
                Constants.Arm.kShoulderMaxSpeedDegSec,
                Constants.Arm.kShoulderMaxAccDegSec));
        mShoulderPid.enableContinuousInput(0, 360);

        // Now do the same idea for the wrist motor(s).
        mWristMotorMain = new CANSparkMax(Constants.Arm.kWristMain, MotorType.kBrushless);
        mWristFollower = new CANSparkMax(Constants.Arm.kWristFollow, MotorType.kBrushless);    
        mWristEncoder = new DutyCycleEncoder(Constants.Arm.kWristEncoderChannel);
        mWristMotorMain.setIdleMode(IdleMode.kBrake);
        mWristFollower.setIdleMode(IdleMode.kBrake);
        mWristMotorMain.enableVoltageCompensation(10);
        mWristMotorMain.setSmartCurrentLimit(20);
        mWristFollower.enableVoltageCompensation(10);
        mWristFollower.setSmartCurrentLimit(20);
        mWristEncoder.setPositionOffset(Constants.Arm.kWrist0ffsetDeg/Constants.Arm.kWristEncRotDeg);
        mWristEncoder.setDistancePerRotation(Constants.Arm.kWristEncRotDeg);
        mWristPid = new ProfiledPIDController(
            Constants.Arm.kWristP,
            Constants.Arm.kWristI,
            Constants.Arm.kWristD,
            new TrapezoidProfile.Constraints(
                Constants.Arm.kWristMaxSpeedDegSec,
                Constants.Arm.kWristMaxAccDegSec));
    }

    public void reset() {
        mShoulderPid.reset(getShoulderRotation());
        mWristPid.reset(getWristRotation());
    }

    /**
     * Get the current rotation of the Shoulder (measured by the duty
     * cycle encoder) in degrees.
     */
    public double getShoulderRotation() {
        return mShoulderEncoder.getDistance();
    }
    /**
     * Get the current rotation of the wrist (measured by the duty
     * cycle encoder) in degrees.
     */
    public double getWristRotation() {
        double wristAngle = ((mWristEncoder.getDistance() % 360) - 360) % 360;
        return wristAngle;
    }

    /**
     * Update the velocity of the arm to a new speed.
     */
    private void setShoulderSpeed(double speed) {
        mShoulderMotorMain.set(-speed);
        mShoulderFollower.set(speed);
    }
 
    /**
     * Update the velocity of the wrist to a new speed.
     */
    private void setWristSpeed(double speed) {
        mWristMotorMain.set(speed);
        mWristFollower.set(-speed);
    }
    
    public void setShoulderAngle(double pShoulderAngle){
        double angleToSet =
            MathUtil.clamp(pShoulderAngle,
                Constants.Arm.kShoulderMinimum,
                Constants.Arm.kShoulderMaximum);
    
        mShoulderPosition = angleToSet;
    }
    public void setWristAngle(double pWristAngle){
        double angleToSet  =
            MathUtil.clamp(pWristAngle - Constants.Arm.kWrist0ffsetMargin,
                Constants.Arm.kWristMinimum - Constants.Arm.kWrist0ffsetMargin - getShoulderRotation(),
                Constants.Arm.kWristMaximum - Constants.Arm.kWrist0ffsetMargin);
        mWristPosition = angleToSet;
    }

    public double getTargetShoulderRotation(){
        return mShoulderPosition;
    }

    public double getTargetWristRotation(){
       return mWristPosition + Constants.Arm.kWrist0ffsetMargin;
    }

    public void zeroShoulder() {
        mShoulderEncoder.reset();
    }


    @Override
    public void periodic() {
        // Display the current arm encoder rotation.
        
        SmartDashboard.putNumber("Shoulder Rotation", getShoulderRotation());
        SmartDashboard.putNumber("Shoulder Main Power", mShoulderMotorMain.get());
        SmartDashboard.putNumber("Shoulder Follower Power", mShoulderFollower.get());
        SmartDashboard.putNumber("Wrist Rotation", getWristRotation() + Constants.Arm.kWrist0ffsetMargin);
        SmartDashboard.putNumber("Wrist Raw Rotation", getWristRotation());
        SmartDashboard.putNumber("Wrist Main Power", mWristMotorMain.get());
        SmartDashboard.putNumber("Wrist Follower Power", mWristFollower.get());

        double shoulderOutput =  mShoulderPid.calculate(getShoulderRotation(), mShoulderPosition)
            + Constants.Arm.kShoulderFeedForwardPct * Math.sin(Math.toRadians(mShoulderPosition));    
        double wristOutput = mWristPid.calculate(getWristRotation(), mWristPosition)
            + Constants.Arm.kWristFeedForwardPct * Math.sin(Math.toRadians(mWristPosition));
        setShoulderSpeed(shoulderOutput);
        setWristSpeed(wristOutput);
    }
}