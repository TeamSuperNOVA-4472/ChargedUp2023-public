package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerIntake extends SubsystemBase{ //pid?
    private final CANSparkMax mRoller1 = new CANSparkMax(0, MotorType.kBrushless);
    //private final CANSparkMax mRoller2 = new CANSparkMax(1, MotorType.kBrushless);
    
    public RollerIntake(){

    }

    public void moveVoltage(double pVolts){
        mRoller1.setVoltage(pVolts);
        //mRoller2.setVoltage(-pVolts);
    }

    public void moveSpeed(double pSpeed){
        mRoller1.set(pSpeed); // sets the two motors to tunr opposite of each other
        //mRoller2.set(-pSpeed);
    }
}
