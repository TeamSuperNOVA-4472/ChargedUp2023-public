// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  TalonSRX mLeftFront, mLeftBack, mRightFront, mRightBack;
  AHRS gyro;

  public ExampleSubsystem(TalonSRX lf, TalonSRX lb, TalonSRX rf, TalonSRX rb, AHRS g) {
    mLeftFront = lf;
    mLeftBack = lb;
    mRightFront = rf;
    mRightBack = rb;
    gyro = g;
  }

  public void move(double lSpeed, double rSpeed) {
    mLeftBack.set(TalonSRXControlMode.PercentOutput, lSpeed);
    mLeftFront.set(TalonSRXControlMode.PercentOutput, lSpeed);
    mRightBack.set(TalonSRXControlMode.PercentOutput, rSpeed);
    mRightFront.set(TalonSRXControlMode.PercentOutput, rSpeed);
  }

  public AHRS getGyro()
  {
    return gyro;
  }

}
