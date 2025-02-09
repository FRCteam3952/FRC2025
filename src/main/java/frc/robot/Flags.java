package frc.robot;

/**
 * Control flags for the robot.
 * These flags should be changed to enable or disable functionality from a centralized location rather than digging through robot code.
 * <p><br>
 * REMEMBER TO RESET THESE BEFORE COMPETITION GAME BEGINS UNLESS YOU ARE ABSOLUTELY CERTAIN YOU WANT IT DISABLED.
 */
public final class Flags {
    /**
     * Flags relating to printing/information useful for debugging.
     */
    public static class Debug {
        public static final boolean CLEAR_PDP_STICKY_FAULTS = true;
        public static final boolean PRINT_PDP_STICKY_FAULTS = false;
    }

    /**
     * Flags relating to the Operator console (i.e. relating to the Driver Station laptop).
     */
    public static class Operator {
        public static final boolean NINTENDO_SWITCH_CONTROLLER_AS_PRIMARY = false;
    }

    public static class TelescopingArm {
        public static final boolean IS_ATTACHED = true;
        public static final boolean ENABLED = true;
        public static final boolean USE_TEST_ELEVATOR_COMMAND = false;
    }

    public static class Climber {
        public static final boolean IS_ATTACHED = true;
        public static final boolean ENABLED = true;
        public static final boolean USE_TEST_CLIMBER_COMMAND = false;
    }

    public static class CoralIntake {
        public static final boolean IS_ATTACHED = true;
        public static final boolean ENABLED = true;
        public static final boolean USE_TEST_CORAL_COMMAND = false;
    }

    /**
     * Flags relating to the drive train.
     */
    public static class DriveTrain {
        /**
         * Whether the drive train is physically attached and existing. If false, no motor controllers are initialized since they are assumed to be nonexistent.
         */
        public static final boolean IS_ATTACHED = false;

        /**
         * Whether the drive train should be allowed to send power to motor controllers. If false, motors will not be set to any power and PID requests will not be sent.
         */
        public static final boolean ENABLED = false;

        /**
         * Whether the drive motors should be allowed to run. If false, drive motors will not be set to any power and PID requests will not be sent.
         */
        public static final boolean ENABLE_DRIVE_MOTORS = true;
        /**
         * Whether the turn motors should be allowed to run. If false, turn motors will not be set to any power and PID requests will not be sent.
         */
        public static final boolean ENABLE_TURN_MOTORS = true;

        /**
         * Whether the drive motors should be allowed to run based on PID control. If false, drive motors can only be run by sending raw power values/voltages.
         */
        public static final boolean DRIVE_PID_CONTROL = true;
        /**
         * Whether the turn motors should be allowed to run based on PID control. If false, drive motors can only be run by sending raw power values/voltages.
         */
        public static final boolean TURN_PID_CONTROL = true;

        /**
         * Whether to lower the maximum speed of the drive train.
         */
        public static final boolean LOWER_MAX_SPEED = false;

        /**
         * Whether to enable optimizing swerve module rotations.
         */
        public static final boolean SWERVE_MODULE_OPTIMIZATION = true;
        /**
         * Whether to only optimize swerve modules if the speeds are below a certain value.
         */
        public static final boolean SPEED_BASED_SWERVE_MODULE_OPTIMIZATION = false;

        /**
         * Whether to enable the test drive command rather than the normal command.
         */
        public static final boolean USE_TEST_DRIVE_COMMAND = false;

        /**
         * Enables locked heading mode: when the rotation joystick is not moved, it uses gyroscope to
         * lock the heading (more in drive train code).
         *
         * @implNote: if the robot were to bounce off of something while in locked heading mode, AND the driver doesn't
         * adjust the heading, this would cause kinda unexpected behavior because the robot would try to reassume
         * its initial heading.
         */
        public static final boolean ENABLE_LOCKED_HEADING_MODE = false;

        public static final boolean ENABLE_COSINE_COMPENSATOR = false;

        public static final boolean ENABLE_ANGULAR_VELOCITY_COMPENSATION_TELEOP = false;

        // True if the auton selection GUI in Shuffleboard is active and we're choosing autons from it
        public static final boolean ENABLE_AUTON_CHOOSER = false;
    }
}