package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RollerIntake;

public class RollerIntakeTeleop extends CommandBase{
    private final XboxController mController;
    private final RollerIntake mIntake;

    public RollerIntakeTeleop(XboxController pController, RollerIntake pIntake){
        mController = pController;
        mIntake = pIntake;
        addRequirements(mIntake);
    }

    @Override
    public void execute(){ // mayeb use a deadband
        boolean forIntake = mController.getBButton(); // true if b button is pressed
        boolean revIntake = mController.getAButton(); // true if a button is pressed

        if(forIntake){ // goes forward
            mIntake.moveSpeed(0.75); // speed is positive, will have to put in a constant later
        } // change to moveVoltage later?

        else if(revIntake){
            mIntake.moveSpeed(-1.0); // speed is negative, constant later
        }
        else{
            mIntake.moveSpeed(0); // or just doesnt move
        }
    }

    
}
