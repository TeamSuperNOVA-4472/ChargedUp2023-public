package frc.lib.util;

import frc.robot.Constants;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;

public final class DynamicTuner {
    private static ShuffleboardTab tuningTab;
        
    private static ShuffleboardLayout drivetrainTuningList;
    private static GenericEntry drive_kP;
    private static GenericEntry drive_kI;
    private static GenericEntry drive_kD;

    private static ShuffleboardLayout swerveAngleTuningList;
    private static GenericEntry angle_kP;
    private static GenericEntry angle_kI;   
    private static GenericEntry angle_kD;
    private static GenericEntry angle_kF;

    private static ShuffleboardLayout armTuningList;
    private static GenericEntry arm_kP;
    private static GenericEntry arm_kI;
    private static GenericEntry arm_kD;

    private static ShuffleboardLayout wristTuningList;
    private static GenericEntry wrist_kP;
    private static GenericEntry wrist_kI;
    private static GenericEntry wrist_kD;

    public static void init() {
        tuningTab = Shuffleboard.getTab("TuningTab");
        /*
         * Drivetrain constants
         */

         drivetrainTuningList = tuningTab.getLayout("Drivetrain PID Tuning", BuiltInLayouts.kList);

         drive_kP = drivetrainTuningList.add("drive_kP", Constants.Swerve.driveKP).getEntry();
         drive_kI = drivetrainTuningList.add("drive_kI", Constants.Swerve.driveKI).getEntry();
         drive_kD = drivetrainTuningList.add("drive_kD", Constants.Swerve.driveKD).getEntry();
 
         /*
          * Angle adjustment constants
          */
 
         swerveAngleTuningList = tuningTab.getLayout("Swerve Angle PID Tuning", BuiltInLayouts.kList);
 
         angle_kP = swerveAngleTuningList.add("angle_kP", Constants.Swerve.angleKP).getEntry();
         angle_kI = swerveAngleTuningList.add("angle_kI", Constants.Swerve.angleKI).getEntry();
         angle_kD = swerveAngleTuningList.add("angle_kD", Constants.Swerve.angleKD).getEntry();
         angle_kF = swerveAngleTuningList.add("angle_kF", Constants.Swerve.angleKF).getEntry();
 
         armTuningList = tuningTab.getLayout("Arm PID Tuning");
         
         arm_kP = armTuningList.add("arm_kP", Constants.Arm.kShoulderP).getEntry();
         arm_kI = armTuningList.add("arm_kI", Constants.Arm.kShoulderI).getEntry();
         arm_kD = armTuningList.add("arm_kD", Constants.Arm.kShoulderD).getEntry();

         wristTuningList = tuningTab.getLayout("Wrist PID Tuning");

         wrist_kP = wristTuningList.add("wrist_kP", Constants.Arm.kWristP).getEntry();
         wrist_kP = wristTuningList.add("wrist_kI", Constants.Arm.kWristP).getEntry();
         wrist_kP = wristTuningList.add("wrist_kD", Constants.Arm.kWristP).getEntry();
    }

    public static double getDriveP() {
        return drive_kP.getDouble(Constants.Swerve.driveKP);
    }

    public static double getDriveI() {
        return drive_kI.getDouble(Constants.Swerve.driveKI);
    }

    public static double getDriveD() {
        return drive_kD.getDouble(Constants.Swerve.driveKD);
    }

    public static double getAngleP() {
        return angle_kP.getDouble(Constants.Swerve.angleKP);
    }

    public static double getAngleI() {
        return angle_kI.getDouble(Constants.Swerve.angleKI);
    }

    public static double getAngleD() {
        return angle_kD.getDouble(Constants.Swerve.angleKD);
    }

    public static double getAngleF() {
        return angle_kF.getDouble(Constants.Swerve.angleKF);
    }

    public static double getArmP() {
        return arm_kP.getDouble(Constants.Arm.kShoulderP);
    }

    public static double getArmI() {
        return arm_kI.getDouble(Constants.Arm.kShoulderI);
    }

    public static double getArmD() {
        return arm_kD.getDouble(Constants.Arm.kShoulderD);
    }

    public static double getWristP() {
        return wrist_kP.getDouble(Constants.Arm.kWristP);
    }

    public static double getWristI() {
        return wrist_kI.getDouble(Constants.Arm.kWristI);
    }

    public static double getWristD() {
        return wrist_kD.getDouble(Constants.Arm.kWristD);
    }
}