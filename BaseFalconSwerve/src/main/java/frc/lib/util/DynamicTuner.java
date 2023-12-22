package frc.lib.util;

import frc.robot.Constants;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;

public final class DynamicTuner {
    static ShuffleboardTab tuningTab;
        
    static ShuffleboardLayout drivetrainTuningList;
    static GenericEntry drive_kP;
    static GenericEntry drive_kI;
    static GenericEntry drive_kD;

    static ShuffleboardLayout swerveAngleTuningList;
    static GenericEntry angle_kP;
    static GenericEntry angle_kI;   
    static GenericEntry angle_kD;
    static GenericEntry angle_kF;

    public static void initialize() {
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
}