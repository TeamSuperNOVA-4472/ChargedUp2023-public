package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmTeleop extends CommandBase {
    private final Arm mArm;
    private final XboxController mController;

    private double mShoulderRot = 0;
    private double mWristRot = 0;
    private boolean mLevelWrist = false;

    public ArmTeleop(XboxController pController, Arm pArm) {
        // Add requirements
        addRequirements(pArm);

        // Assign constants
        mArm = pArm;
        mController = pController;
    }

    @Override
    public void initialize() {
        mArm.reset();
        mShoulderRot = mArm.getTargetShoulderRotation();
        mWristRot = mArm.getTargetWristRotation();
        mLevelWrist = false;
    }

    @Override
    public void execute() {
        applyButtons();
        applyAnalog();
        updateRotations();
        updateDashboard();
    }

    /**
     * Apply the controller analog inputs to the shoulder and wrist rotations.
     */
    private void applyAnalog() {
        // The controller's opinions on how much the arm should move.
        double controllerShoulder = -MathUtil.applyDeadband(mController.getRightY(), Constants.stickDeadband),
               controllerWrist = -MathUtil.applyDeadband(mController.getLeftY(), Constants.stickDeadband);
        
        if (controllerWrist != 0) {
            mLevelWrist = false;
        }
        // Apply deadband and turn into change for the shoulder and wrist rotation.
        mShoulderRot += controllerShoulder * Constants.Arm.kShoulderAnalogFactor;
        mWristRot += controllerWrist * Constants.Arm.kWristAnalogFactor;
    
        mShoulderRot = MathUtil.clamp(mShoulderRot, Constants.Arm.kShoulderMinimum, Constants.Arm.kShoulderMaximum);
        mWristRot = MathUtil.clamp(mWristRot, Constants.Arm.kWristMinimum - mShoulderRot, Constants.Arm.kWristMaximum);
    }

    /**
     * Apply the controller button presses to the shoulder and wrist rotations
     * via presets.
     */
    private void applyButtons() {
        int pov = mController.getPOV();

        // The buttons control which preset position to set the arm to.
        // The down button is for the low goal,
        // the up button is for the driver pickup dock,
        // the right button is for the high goal, and
        // the down button resets the arm.

        if (pov > 315 || (pov <= 45 && pov >= 0)) // Between up-left and up-right
        {
            // High goal. This has been mapped and should work.
            mShoulderRot = Constants.Arm.kShoulderHighGoal;
        }
        if (pov > 45 && pov <= 135) // Between up-right and down-right
        {
            // Mid goal.
            mShoulderRot = Constants.Arm.kShoulderMiddleGoal;
        }
        if (pov > 225 && pov <= 315) // Between down-left and up-left
        {
            // Return to zero position. This should work.
            mShoulderRot = Constants.Arm.kShoulderMiddleGoal;
        }
        if (pov > 135 && pov <= 225) // Between down-right and down-left
        {
            // High goal. This has been mapped and should work.
            mShoulderRot = Constants.Arm.kShoulderLowGoal;
        }

        if (mController.getLeftBumperPressed()) {
            mLevelWrist = !mLevelWrist;
            if(!mLevelWrist) {
                mWristRot = Constants.Arm.kWristMaximum;
            }
        }

        if (mController.getRightTriggerAxis() != 0) {
            mLevelWrist = false;
            mWristRot = Constants.Arm.kWristMaximum;
        }

        if (mController.getRightBumperPressed()) {
            mLevelWrist = false;
            mWristRot = Constants.Arm.kWristSubstationAngle;
        }

        if (mController.getLeftTriggerAxis() != 0) {
            mLevelWrist = false;
            mWristRot = Constants.Arm.kWristScoringPreset;
        }

        if(mLevelWrist) {
            mWristRot = Constants.Arm.kWristMinimum - mArm.getShoulderRotation();
        }

    }

    /**
     * Give the arm subsystem the new desired shoulder and wrist rotations.
     */
    private void updateRotations() {
        mArm.setShoulderAngle(mShoulderRot);
        mArm.setWristAngle(mWristRot);
    }

    /**
     * Update the Smart Dashboard with some debug values.
     */
    private void updateDashboard() {
        SmartDashboard.putNumber("Shoulder Target", mShoulderRot);
        SmartDashboard.putNumber("Wrist Target", mWristRot);
    }
}