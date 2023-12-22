package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ARM_PORTS;
import static frc.robot.Constants.ARM_ENCODER_CHANNEL;
import static frc.robot.Constants.WRIST_PORTS;
import static frc.robot.Constants.WRIST_ENCODER_CHANNEL;

/**
 * This subsystem controls the rotation and speed of the arm.
 */
public class Arm extends SubsystemBase {
    public static final double kArmDeadband = 0.01;
    public static final double kControllerDeadband = 0.15;
    
    public static final double kArmSpeed = 0.1;
    public static final double kWristSpeed = 0.1;

    public static final double kArmLowGoal = 0.75;
    public static final double kArmMiddleGoal = 0.65;
    public static final double kArmHighGoal = 0.55;

    public static final double kWristLowGoal = 0.75; // TODO: these values are just guesses
    public static final double kWristMiddleGoal = 0.65;
    public static final double kWristHighGoal = 0.55;

    public static final double kArmMinimum = 0.5;
    public static final double kArmMaximum = 0.85;

    public static final double kWristMinimum = 0.25; // TODO: these values are just guesses
    public static final double kWristMaximum = 0.75;

    public static final double kArmP = 1.0;
    public static final double kArmI = 0.0;
    public static final double kArmD = 0.0;

    public static final double kWristP = 1.0;
    public static final double kWristI = 0.0;
    public static final double kWristD = 0.0;
    
    private final CANSparkMax mArmMotorMain;
    private final CANSparkMax[] mArmFollowers;
    private final DutyCycleEncoder mArmEncoder;

    private final CANSparkMax mWristMotorMain;
    private final CANSparkMax[] mWristFollowers;
    private final DutyCycleEncoder mWristEncoder;
    
    public Arm() {
        // Assign subsystem components.

        // The first port in the list of arm motor ports will be the main motor.
        // The rest will be followers and be put in the `mArmFollowers` array.
        if (ARM_PORTS.length == 0) {
            mArmMotorMain = null;
            mArmFollowers = null;
        }
        else {
            // Set up the main arm motor and its followers.
            mArmMotorMain = new CANSparkMax(ARM_PORTS[0], MotorType.kBrushless);
            mArmFollowers = new CANSparkMax[ARM_PORTS.length - 1];
            
            for (int i = 1; i < ARM_PORTS.length; i++) {
                // Create a follower motor with an additional arm port.
                CANSparkMax follower = new CANSparkMax(ARM_PORTS[i], MotorType.kBrushless);
                follower.follow(mArmMotorMain);
                mArmFollowers[i - 1] = follower;
            }
        }
        mArmEncoder = new DutyCycleEncoder(ARM_ENCODER_CHANNEL);

        // Now do the same idea for the wrist motor(s).
        if (WRIST_PORTS.length == 0) {
            mWristMotorMain = new CANSparkMax(WRIST_PORTS[0], MotorType.kBrushless);
            mWristFollowers = new CANSparkMax[WRIST_PORTS.length - 1];

            for (int i = 1; i < WRIST_PORTS.length; i++) {
                CANSparkMax follower = new CANSparkMax(WRIST_PORTS[i], MotorType.kBrushless);
                follower.follow(mWristMotorMain);
                mWristFollowers[i - 1] = follower;
            }
        }
        else {
            mWristMotorMain = null;
            mWristFollowers = null;
        }
        mWristEncoder = new DutyCycleEncoder(WRIST_ENCODER_CHANNEL);
    }

    /**
     * Get the current rotation of the arm (measured by the duty
     * cycle encoder), in whatever units it's currently in.
     */
    public double getArmRotation() {
        // TODO: maybe convert whatever this number is to degrees.
        return mArmEncoder.get();
    }
    /**
     * Get the current rotation of the wrist (measured by the duty
     * cycle encoder), in whatever units it's currently in.
     */
    public double getWristRotation() {
        // TODO: maybe convert whatever this number is to degrees.
        return mWristEncoder.get();
    }

    /**
     * Update the velocity of the arm to a new speed.
     */
    public void setArmSpeed(double speed) {
        mArmMotorMain.set(speed);
    }    
    /**
     * Update the velocity of the wrist to a new speed.
     */
    public void setWristSpeed(double speed) {
        mWristMotorMain.set(speed);
    }

    /**
     * Stops both the arm and wrist motor.
     */
    public void stopAll() {
        stopArm();
        stopWrist();
    }
    /**
     * Stops the arm motor.
     */
    public void stopArm() {
        mArmMotorMain.stopMotor();
    }
    /**
     * Stops the wrist motor.
     */
    public void stopWrist() {
        mWristMotorMain.stopMotor();
    }

    @Override
    public void periodic() {
        // Display the current arm encoder rotation.
        SmartDashboard.putNumber("Arm Rotation", mArmEncoder.getAbsolutePosition());
    }    
}
