package frc.robot.commands;


import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Arm;

public class ArmSetPosition extends CommandBase{
    //private final double kArmLowGoal = 0.75;
    //private final double kArmMiddleGoal = 0.65;
    //private final double kArmHighGoal = 0.55;

    private final Arm mArm;
    private final PIDController mPid;

    private double mDesiredPos = 0;

    public ArmSetPosition(double pPos, Arm pArm)
    {
        // Add requirements.
        addRequirements(pArm);

        // Assign subsystems and controllers.
        mArm = pArm;
        mPid = new PIDController(Constants.Arm.kArmP, Constants.Arm.kArmI, Constants.Arm.kArmD);

        // Make sure the arm is stopped so we know its exact position.
        mArm.stopArm();
        //initializes the position we want the arm to go to.
        mDesiredPos = pPos; 
    }

    @Override
    public void execute(){
        armMoveTo(mDesiredPos);
    }

    private void armMoveTo(double pDesiredPos){
        if (mDesiredPos > Constants.Arm.kArmMaximum) mDesiredPos = Constants.Arm.kArmMaximum;
        if (mDesiredPos < Constants.Arm.kArmMinimum) mDesiredPos = Constants.Arm.kArmMinimum;
        mPid.setSetpoint(pDesiredPos);
        if (!mPid.atSetpoint()){
            mArm.setArmSpeed(mPid.calculate(mArm.getArmRotation()));
        }
        else{
            mArm.stopArm();
        }
    }
}

