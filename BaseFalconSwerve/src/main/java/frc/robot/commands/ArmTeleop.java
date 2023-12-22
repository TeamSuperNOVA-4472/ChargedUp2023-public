package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

/**
 * A command that enables the control of the arm and wrist by the user controller.
 * The arm is controlled by the Y-axis of the right joystick and the wrist is
 * controlled by the X-axis of the right joystick.
 */
public class ArmTeleop extends CommandBase {
    private final Arm mArm;
    private final XboxController mController;
    private final PIDController mArmPid;
    private final PIDController mWristPid;

    private double mDesiredArmPos = Arm.kArmMaximum;
    private boolean mHasStoppedArm;
    private boolean mIsArmAnalog = false;

    private double mDesiredWristPos = Arm.kArmMaximum;
    private boolean mHasStoppedWrist;
    private boolean mIsWristAnalog = false;

    public ArmTeleop(XboxController pController, Arm pArm) {
        // Add requirements.
        addRequirements(pArm);

        // Assign subsystems and controllers.
        mController = pController;
        mArm = pArm;
        mArmPid = new PIDController(Arm.kArmP, Arm.kArmI, Arm.kArmD);
        mWristPid = new PIDController(Arm.kWristP, Arm.kWristI, Arm.kWristD);

        // Make sure the arm and wrist are stopped so we can assign the
        // `mHasStopped` variables.
        mArm.stopAll();
        mHasStoppedArm = true;
        mHasStoppedWrist = true;
    }

    @Override
    public void execute() {
        // Update the arm.
        getArmButtonControls();
        getArmAnalogControls();
        updateArmVelocity();

        // Update the wrist.
        getWristAnalogControls();
        getWristButtonControls();
        updateWristVelocity();
    }

    /**
     * Update the desired arm position based on the analog joystick inputs.
     */
    private void getArmAnalogControls() {
        // Get the amount to change the desired arm value by.
        // Use the controller value and the maximum speed and
        // apply a deadband.
        double updatePos = mController.getRightY();
        updatePos = MathUtil.applyDeadband(updatePos, Arm.kControllerDeadband);
        updatePos *= Arm.kArmSpeed;

        // Set the analog flag to true if the controller is using the analog function.
        if (updatePos != 0) mIsArmAnalog = true;

        // Set the position if we are in analog mode.
        if (mIsArmAnalog) mDesiredArmPos = mArm.getArmRotation() + updatePos;

        SmartDashboard.putNumber("Arm Update Position", updatePos);
    }
    /**
     * Update the desired arm position based on the controller buttons.
     */
    private void getArmButtonControls() {
        // The button X positions the arm for the low goal.
        // The button Y positions the arm for the middle goal.
        // The button B positions the arm for the high goal.
        boolean lowPressed = mController.getXButtonPressed(),
                middlePressed = mController.getYButtonPressed(),
                highPressed = mController.getBButtonPressed();

        // Set the desired position based on the button pressed.
        if (lowPressed) mDesiredArmPos = Arm.kArmLowGoal;
        if (middlePressed) mDesiredArmPos = Arm.kArmMiddleGoal;
        if (highPressed) mDesiredArmPos = Arm.kArmHighGoal;

        if (lowPressed || middlePressed || highPressed) mIsArmAnalog = false;
    }
    /**
     * Set the arm velocity based on the current and desired positions.
     */
    private void updateArmVelocity() {
        // Clamp the desired position between the minimum and
        // maximum and update the PID setpoint.
        mDesiredArmPos = MathUtil.clamp(mDesiredArmPos, Arm.kArmMinimum, Arm.kArmMaximum);
        mArmPid.setSetpoint(mDesiredArmPos);

        // Get our desired velocity for the arm via the PID controller.
        double currentPos = mArm.getArmRotation();
        double velocity = mArmPid.calculate(currentPos);

        // If the velocity is greater than the deadband, we can apply
        // the new speed. Otherwise, stop the robot if it isn't stopped
        // already.
        if (Math.abs(velocity) > Arm.kArmDeadband)
        {
            // Apply the new speed to the arm.
            mArm.setArmSpeed(velocity);
            mHasStoppedArm = false;
        }
        else
        {
            // Stop the arm if it is not stopped already.
            if (!mHasStoppedArm) mArm.stopArm();
            mHasStoppedArm = true;
        }

        SmartDashboard.putNumber("Arm Desired Position", mDesiredArmPos);
        SmartDashboard.putNumber("Arm Velocity", velocity);
    }

    /**
     * Update the desired wrist position based on the analog joystick inputs.
     */
    private void getWristAnalogControls() {
        // Get the amount to change the desired wrist value by.
        // Use the controller value and the maximum speed and
        // apply a deadband.
        double updatePos = mController.getRightX();
        updatePos = MathUtil.applyDeadband(updatePos, Arm.kControllerDeadband);
        updatePos *= Arm.kWristSpeed;

        // Set the analog flag to true if the controller is using the analog function.
        if (updatePos != 0) mIsWristAnalog = true;

        // Set the position if we are in analog mode.
        if (mIsWristAnalog) mDesiredWristPos = mArm.getWristRotation() + updatePos;

        SmartDashboard.putNumber("Wrist Update Position", updatePos);
    }
    /**
     * Update the desired wrist position based onn the controller buttons.
     */
    private void getWristButtonControls() {
        // The button X positions the arm for the low goal.
        // The button Y positions the arm for the middle goal.
        // The button B positions the arm for the high goal.
        boolean lowPressed = mController.getXButtonPressed(),
                middlePressed = mController.getYButtonPressed(),
                highPressed = mController.getBButtonPressed();

        // Set the desired position based on the button pressed.
        if (lowPressed) mDesiredWristPos = Arm.kWristLowGoal;
        if (middlePressed) mDesiredWristPos = Arm.kWristMiddleGoal;
        if (highPressed) mDesiredWristPos = Arm.kWristHighGoal;

        if (lowPressed || middlePressed || highPressed) mIsWristAnalog = false;
    }
    /**
     * Set the wrist velocity based on the current and desired positions.
     */
    private void updateWristVelocity() {
        // Clamp the desired position between the minimum and
        // maximum and update the PID setpoint.
        mDesiredWristPos = MathUtil.clamp(mDesiredWristPos, Arm.kWristMinimum, Arm.kWristMaximum);
        mWristPid.setSetpoint(mDesiredWristPos);

        // Get our desired velocity for the wrist via the PID controller.
        double currentPos = mArm.getWristRotation();
        double velocity = mWristPid.calculate(currentPos);

        // If the velocity is greater than the deadband, we can apply
        // the new speed. Otherwise, stop the robot if it isn't stopped
        // already.
        if (Math.abs(velocity) > Arm.kArmDeadband)
        {
            // Apply the new speed to the arm.
            mArm.setWristSpeed(velocity);
            mHasStoppedWrist = false;
        }
        else
        {
            // Stop the arm if it is not stopped already.
            if (!mHasStoppedWrist) mArm.stopWrist();
            mHasStoppedWrist = true;
        }

        SmartDashboard.putNumber("Wrist Desired Position", mDesiredWristPos);
        SmartDashboard.putNumber("Wrist Velocity", velocity);
    }
}
