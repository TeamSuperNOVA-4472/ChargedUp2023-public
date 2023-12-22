package frc.robot;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController driver = new XboxController(0);
    private final XboxController mController = new XboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    private DoubleSupplier rightTrigger = () -> driver.getRightTriggerAxis();

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton aButton = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton bButton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton aprilTagAlign = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final Trigger limeLightAlign = new Trigger(() ->  {
        SmartDashboard.putNumber("Right Axis trigger value", rightTrigger.getAsDouble());
        return rightTrigger.getAsDouble() != 0;
    });


    private final JoystickButton partnerStart = new JoystickButton(mController, XboxController.Button.kStart.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final VisionSubsystem vision = new VisionSubsystem();
    private final Arm mArm = new Arm();
    private final RollerIntake mRollerIntake = new RollerIntake();

    /* Commands */
    private final ArmTeleop mArmCommand = new ArmTeleop(mController, mArm);
    private final RollerIntakeTeleop mIntakeCommand = new RollerIntakeTeleop(mController, mRollerIntake);

    /* Auto selector */
    ShuffleboardTab driverStationTab = Shuffleboard.getTab("Driver Station Tab");
    SendableChooser<Command> autoSelection = new SendableChooser<Command>();
    private final Command scoreAndDriveBackToChargingStation = new ScoreAndDriveBackToChargingStation(s_Swerve, mArm, mRollerIntake);
    private final Command scorePreloadAuton = new ScorePreloadAuton(s_Swerve, mArm, mRollerIntake, -5, 4.75);
    private final Command Two = new Two(s_Swerve, mArm, mRollerIntake);
    private final Command ScoreDriveGrabBalance = new ScoreDriveGrabBalance(s_Swerve, mArm, mRollerIntake);
    private final Command TwoScoreOther = new TwoScoreOther(s_Swerve, mArm,mRollerIntake);
    private final Command scoreAndBalance = new ScoreAndBalance(s_Swerve, mArm, mRollerIntake);
    //private final Command scoreAndDriveBack = new ScorePreloadAuton(s_Swerve, mArm, mRollerIntake, -4.2, 4.75);
    private final Command autoBalance = new AutoBalance(s_Swerve);
    //private final Command driveStraight = new GoStraight(s_Swerve, 1, 10, Double.MAX_VALUE);
    private final Command scorePreloadOnly = new ScorePreloadOnly(s_Swerve, mArm, mRollerIntake);
    private final Command gyro180 = new GyroTurn(s_Swerve,180, Double.MAX_VALUE);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new SwerveTeleop(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                () -> zeroGyro.getAsBoolean()
            )
        );
        mArm.setDefaultCommand(mArmCommand);
        mRollerIntake.setDefaultCommand(mIntakeCommand);

        autoSelection.setDefaultOption("Score And Drive Back To Charging Station", scoreAndDriveBackToChargingStation);
        autoSelection.addOption("Score Preload Auton", scorePreloadAuton);
        autoSelection.addOption("Two", Two);
        //autoSelection.addOption("Score Preload Auton", scorePreloadAuton);
        autoSelection.addOption("Score And Balance", scoreAndBalance);
        autoSelection.addOption("Score Preload Only", scorePreloadOnly);
        autoSelection.addOption("Auto Balancing", autoBalance);
        autoSelection.addOption("TwoScoreOther", TwoScoreOther);
        autoSelection.addOption("Score - Drive - Grab - Balance lol", ScoreDriveGrabBalance);
        autoSelection.addOption("Auto Field Oriented Test", new InstantCommand(() -> s_Swerve.setGyroOffset(180)));
        driverStationTab.add("Selected Autonomous", autoSelection).withSize(4, 2).withPosition(0, 2).withWidget(BuiltInWidgets.kSplitButtonChooser);
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        aButton.onTrue(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()));

        aprilTagAlign.whileTrue(new AprilTagTurn(s_Swerve, vision, 3));
        limeLightAlign.whileTrue(new LimeLightTurn(s_Swerve, vision, 3));

        bButton.whileTrue(gyro180);

        partnerStart.onTrue(new InstantCommand(() -> mArm.zeroShoulder()));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoSelection.getSelected();
    }
}
