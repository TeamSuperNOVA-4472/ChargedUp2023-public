package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This subsystem controls the robot drivetrain. This drivetrain
 * is set up to be a tank-drive system, with 2 motors on the left
 * and 2 motors on the right. The front motors are the encoders
 * and the rear motors are the followers.
 */
public class DriveTrain extends SubsystemBase {
    private WPI_TalonSRX mLeftEncoder;
    private WPI_TalonSRX mLeftFollow;
    private WPI_TalonSRX mRightEncoder;
    private WPI_VictorSPX mRightFollow;

    public DriveTrain() {
        // Assign subsystem components
        mLeftEncoder = new WPI_TalonSRX(2);
        mLeftFollow = new WPI_TalonSRX(3);
        mRightEncoder = new WPI_TalonSRX(4);
        mRightFollow = new WPI_VictorSPX(5);

        // Enable following on the follower motors.
        mLeftFollow.follow(mLeftEncoder);
        mRightFollow.follow(mRightEncoder);
    }

    /**
     * Set the velocities of the left and right side motors with
     * the given left and right percentage velocities. Each velocity
     * should range between -1.0 and 1.0, and will be clamped
     * accordingly.
     */
    public void driveTank(double pLeftPercentage, double pRightPercent) {
        // Clamp the velocities if they have not been already.
        pLeftPercentage = MathUtil.clamp(pLeftPercentage, -1, 1);
        pRightPercent = MathUtil.clamp(pRightPercent, -1, 1);

        // Update the motor velocities.
        mLeftEncoder.set(pLeftPercentage);
        mRightEncoder.set(pRightPercent);
    }

    /**
     * Set the velocities of the left right side motors via arcade-style
     * movement. The turn percentage will affect the two sides differently.
     * Each percentage should range between -1.0 and 1.0, and will be
     * clamped accordingly.
     */
    public void driveArcade(double pDrivePercent, double pTurnPercent) {
        // Clamp the velocities if they have not been already.
        double left = MathUtil.clamp(pDrivePercent - pTurnPercent, -1.0, 1.0);
        double right = MathUtil.clamp(pDrivePercent + pTurnPercent, -1.0, 1.0);

        driveTank(left, right);
    }
}
