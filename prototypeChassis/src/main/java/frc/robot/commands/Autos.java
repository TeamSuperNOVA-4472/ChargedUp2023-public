package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos
{
    /** Example static factory for an autonomous command. */
    public static CommandBase exampleAuto(DriveTrain subsystem) {
        return Commands.waitSeconds(1.0);
    }

    // Unused: This is a class with only static components.
    private Autos() { }
}
