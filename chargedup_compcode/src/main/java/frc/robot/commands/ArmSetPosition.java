package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
//import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ArmSetPosition extends CommandBase{
    //private final double kArmLowGoal = 0.75;
    //private final double kArmMiddleGoal = 0.65;
    //private final double kArmHighGoal = 0.55;

    private final Arm mArm;
    private final Timer mTimer = new Timer();
    private final double mTimeout;
    private double mArmPos = 0;
    private double mWristPos = 0;
   

    public ArmSetPosition(double pArmPos, double pWristPos, Arm pArm, double pTimeout)
    {
        // Add requirements.
        addRequirements(pArm);

        // Assign subsystems and controllers.
        mArm = pArm;
        mTimeout = pTimeout;

        //initializes the position we want the arm to go to.
        mArmPos = pArmPos; 
        mWristPos = pWristPos;
    }

    @Override
    public void execute(){
        mArm.setShoulderAngle(mArmPos);
        mArm.setWristAngle(mWristPos);

    }

    @Override
    public void initialize(){
        mTimer.reset();
        mTimer.start();
    }

    @Override
    public boolean isFinished(){
        return mTimer.get() >= mTimeout;
    }

    @Override
    public void end(boolean pInterruped){
        mTimer.stop();
    }

    
}
