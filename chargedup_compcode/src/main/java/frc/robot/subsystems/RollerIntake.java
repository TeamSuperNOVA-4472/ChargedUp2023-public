package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RollerIntake extends SubsystemBase{
    private final CANSparkMax mRoller1;
    private double mTargetSpeed;
    
    public RollerIntake(){
        mRoller1 = new CANSparkMax(Constants.kRollerIntakePort, MotorType.kBrushless);
        mRoller1.setSmartCurrentLimit(20);
        mRoller1.enableVoltageCompensation(10);
        mRoller1.setIdleMode(IdleMode.kBrake);
        mTargetSpeed = 0;
    }

    public void moveSpeed(double pSpeed){
        mTargetSpeed = pSpeed;
    }

    @Override
    public void periodic() {
        if(mTargetSpeed != 0) {
            mRoller1.set(mTargetSpeed); 
        } else {
            mRoller1.set(-Constants.kRollerHoldPct);
        }  
    }
}
