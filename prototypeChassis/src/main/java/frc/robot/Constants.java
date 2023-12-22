package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double TICS_PER_DEGREE = 118;
    public static final double TURN_P = 0.5;
    public static final double TURN_D = 0.005;

    public static final int FL_DRIVE_PORT = 0;
    public static final int FL_TURN_PORT = 1;
    public static final int FL_ENCODER = 8;
    public static final double FL_OFFSET = 173.23;

    public static final int FR_DRIVE_PORT = 2;
    public static final int FR_TURN_PORT = 3;
    public static final int FR_ENCODER = 9;
    public static final double FR_OFFSET = 109.42;

    public static final int RL_DRIVE_PORT = 4;
    public static final int RL_TURN_PORT = 5;
    public static final int RL_ENCODER = 10;
    public static final double RL_OFFSET = 93.25;

    public static final int RR_DRIVE_PORT = 6;
    public static final int RR_TURN_PORT = 7;
    public static final int RR_ENCODER = 11;
    public static final double RR_OFFSET = 11.42;

    public static final int[] ARM_PORTS = { 20 };
    public static final int ARM_ENCODER_CHANNEL = 0;
    public static final int[] WRIST_PORTS = { 21 }; // TODO: Find the actual ports.
    public static final int WRIST_ENCODER_CHANNEL = 1; // TODO: Find the actual channel.

    public static final double SWERVE_LENGTH_IN = 23.75;

    // Unused: This is a class with only static components.
    private Constants() { }

    public static final class Operator {
        public static final int DRIVER_CONTROLLER_PORT = 0;

        // Unused: This is a class with only static components.
        private Operator() { }
    }
}
