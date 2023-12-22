package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A command that enables control of the drivetrain via the
 * Xbox controller's left joystick.
 */
public class DrivetrainTeleop extends CommandBase {
    private final double kDeadband = 0.1;
    private final double kMovementSpeed = 1;
    private final double kTurnSpeed = 1;

    private final DriveTrain mDrivetrain;
    private final XboxController mController;

    public DrivetrainTeleop(DriveTrain pDrivetrain, XboxController pController) {
        // Add command requirements.
        addRequirements(pDrivetrain);

        // Assign command components.
        mDrivetrain = pDrivetrain;
        mController = pController;
    }

    @Override
    public void execute() {
        // Use the left joystick to drive.
        // Use up/down to move forward/backwards, and use left/right to
        // rotate the robot left/right.
        // This is slightly different from classic arcade style because
        // currently, the right joystick's vertical axis controls the arm.
        double speed = mController.getLeftY() * kMovementSpeed,
               angle = mController.getLeftX() * kTurnSpeed;

        // Apply the deadband
        speed = MathUtil.applyDeadband(speed, kDeadband);
        angle = MathUtil.applyDeadband(angle, kDeadband);

        // Update the new speed/angle.
        mDrivetrain.driveArcade(speed, angle);
    }
}
