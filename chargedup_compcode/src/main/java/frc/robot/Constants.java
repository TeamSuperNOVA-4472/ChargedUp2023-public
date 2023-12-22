package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

import edu.wpi.first.util.InterpolatingTreeMap; 

public final class Constants {
    public static final double stickDeadband = 0.1;

    /* Roller Intake */
    public static final int kRollerIntakePort = 25;
    public static final double kRollerHoldPct = 0.050;
    public static final double kIntakeSpeed = 0.5;
    public static final double kIntakeVolts = 7;

    
    public static final class Arm {
        public static final double kShoulderAnalogFactor = 1;
        public static final double kShoulderMaxSpeedDegSec = 600;
        public static final double kShoulderMaxAccDegSec = 600;
        public static final double kShoulderOffsetDeg = 239.6;
        public static final double kShoulderEncRotDeg = 360;
        public static final double kShoulderFeedForwardPct = 0.09;
        public static final int kShoulderMain = 22;
        public static final int kShoulderFollow = 23;
        public static final int kShoulderEncoderChannel = 1;

        public static final double kWristAnalogFactor = 4;
        public static final double kWristMaxSpeedDegSec = 1000;
        public static final double kWristMaxAccDegSec = 1000;
        public static final double kWrist0ffsetMargin = 30;
        public static final double kWrist0ffsetDeg = 25.7 + kWrist0ffsetMargin + 4.2;
        public static final double kWristEncRotDeg = 360;
        public static final double kWristFeedForwardPct = 0.02;
        public static final int kWristMain = 21;
        public static final int kWristFollow = 24;
        public static final int kWristEncoderChannel = 0;
    
        public static final double kArmDeadband = 0.01;
        public static final double kControllerDeadband = 0.15;

        public static final double kShoulderLowGoal = 0;
        public static final double kShoulderMiddleGoal = 75.3;
        public static final double kShoulderHighGoal = 90;
        public static final double kShoulderMinimum = 0;
        public static final double kShoulderMaximum = 91.5;

        public static final double kWristMiddleGoal = -119;
        public static final double kWristHighGoal = -133;
        public static final double kWristMinimum = -97;
        public static final double kWristSubstationAngle = -45.5;
        public static final double kWristScoringPreset = -135;
        public static final double kWristMaximum = -2;

        public static final double kShoulderP = 0.016;
        public static final double kShoulderI = 0.0;
        public static final double kShoulderD = 0.00012;

        public static final double kWristP = 0.01;
        public static final double kWristI = 0.0;
        public static final double kWristD = 0.0;
    }

    public static final class Swerve {
        //public static final int pigeonID = 1;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(22.5); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(22.5); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.19169 / 12);
        public static final double driveKV = (2.2596 / 12);
        public static final double driveKA = (0.36511 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.8; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Brake;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(191.42);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(273.25);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        
        /* Rear Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 8;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(173.23-90);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Rear Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(199.42);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
              
        
    }

    public static final class VisionConstants {
        /* ALL OF THESE MUST BE ADJUSTED TO THE ROBOT */
        public static final double kCameraHeightMeters = Units.inchesToMeters(24);
        public static final double kTargetHeightMeters = Units.inchesToMeters(5);
        public static final double kCameraPitchDegs = 15.0;
        public static final double kAprilTagOffsetDegs = 4.74;
        public static final double kRetroreflectiveOffsetDegs = 0;

        public static final double kDesiredRangeMeters = Units.inchesToMeters(8);
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
        public static final double kMetersPerSecondToDegreesPerSecond = (360.0) / Swerve.wheelCircumference;

        public static final double kGyroTurnToleranceDegs = 2;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final InterpolatingTreeMap<Double,Double> CONTROLLER_PROFILE_MAP = new InterpolatingTreeMap<>();
    public static final InterpolatingTreeMap<Double,Double> LIMELIGHT_DISTANCE_MAP = new InterpolatingTreeMap<>();
    static {
        CONTROLLER_PROFILE_MAP.put(0.0,0.0);
        CONTROLLER_PROFILE_MAP.put(0.8,1.0/3.0);
        CONTROLLER_PROFILE_MAP.put(1.0,1.0);

        /* Limelight values */
        LIMELIGHT_DISTANCE_MAP.put(0.05, Units.inchesToMeters(86.5));
        LIMELIGHT_DISTANCE_MAP.put(0.15, Units.inchesToMeters(58));
        LIMELIGHT_DISTANCE_MAP.put(0.373, Units.inchesToMeters(37.0));
        LIMELIGHT_DISTANCE_MAP.put(1.05, Units.inchesToMeters(24.25));
    }
}