package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ElevatorControlCommand;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.testers.TestDriveCommand;
import frc.robot.commands.testers.TestElevatorCommand;
import frc.robot.controllers.AbstractController;
import frc.robot.controllers.Controller;
import frc.robot.controllers.FlightJoystick;
import frc.robot.controllers.NintendoProController;
import frc.robot.controllers.PS5ControllerIO;
import frc.robot.subsystems.PowerHandler;
import frc.robot.subsystems.staticsubsystems.LimeLight;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;
import frc.robot.util.NetworkTablesUtil;
import frc.robot.util.Util;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;


public class RobotContainer {
    //private static final GenericPublisher COLOR_SENSOR_PUB = NetworkTablesUtil.getPublisher("robot", "color_sensor_sees_note", NetworkTableType.kBoolean);

    /*YAGSL Variables 
    double maximumSpeed = Units.feetToMeters(4.5);
    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    SwerveDrive swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed)();
    */

    // private final FlightJoystick sideJoystick = new FlightJoystick(new CommandJoystick(OperatorConstants.RIGHT_JOYSTICK_PORT));

    // private final NintendoProController nintendoProController = new NintendoProController(new CommandXboxController(OperatorConstants.NINTENDO_PRO_CONTROLLER));
    private final Controller primaryController = new Controller(new PS5ControllerIO(new CommandPS5Controller(OperatorConstants.PS5_CONTROLLER)));

    // private final AbstractController primaryController = Flags.Operator.NINTENDO_SWITCH_CONTROLLER_AS_PRIMARY ? this.nintendoProController : this.ps5Controller;
    private final PowerHandler powerHandler = new PowerHandler();
    // private final AprilTagHandler aprilTagHandler = new AprilTagHandler();

    // private final SendableChooser<Command> autonChooser;

    private final DriveTrainSubsystem driveTrain;
    private final ElevatorSubsystem telescopingArm;
    private final ClimberSubsystem climber;
    private final CoralIntakeSubsystem coralIntake;

    public RobotContainer() {
        this.driveTrain = Util.createIfFlagElseNull(DriveTrainSubsystem::new, Flags.DriveTrain.IS_ATTACHED);
        this.telescopingArm = Util.createIfFlagElseNull(ElevatorSubsystem::new, Flags.TelescopingArm.IS_ATTACHED);
        this.climber = Util.createIfFlagElseNull(ClimberSubsystem::new, Flags.Climber.IS_ATTACHED);
        this.coralIntake = Util.createIfFlagElseNull(CoralIntakeSubsystem::new, Flags.CoralIntake.IS_ATTACHED);

        configureBindings();

        // Initialize static subsystems (this is a Java thing don't worry about it just copy it so that static blocks run on startup)
        LimeLight.poke();
        // RobotGyro.poke();
        // ColorSensor.poke();

        // if (Flags.DriveTrain.IS_ATTACHED && Flags.DriveTrain.ENABLE_AUTON_CHOOSER) {
        //     this.autonChooser = AutoBuilder.buildAutoChooser();
        //     SmartDashboard.putData("choose your auto", this.autonChooser);
        // } else {
        //     this.autonChooser = null;
        // }

        NetworkTablesUtil.getConnections();
    }

    private void configureBindings() {
        // TODO: bindings need to be re-implemented if still in use for YAGSL drive
        // These old bindings are only for the nintendo pro controller, which we no longer use.

        // if (Flags.DriveTrain.IS_ATTACHED) {
        //     ControlHandler.get(this.nintendoProController, ControllerConstants.ZERO_SWERVE_MODULES).onTrue(this.driveTrain.rotateToAbsoluteZeroCommand());
        // }
        // ControlHandler.get(this.nintendoProController, ControllerConstants.ZERO_GYRO).onTrue(Commands.runOnce(() -> {
        //     if(Util.onBlueTeam()) {
        //         RobotGyro.resetGyroAngle();
        //     } else {
        //         RobotGyro.setGyroAngle(180);
        //     }
        //     // TODO: implement heading lock mode on YAGSL drive
        //     //this.driveTrain.setHeadingLockMode(false);
        // }));

        // if(Flags.DriveTrain.IS_ATTACHED) {
        //     ControlHandler.get(this.nintendoProController, ControllerConstants.RESET_POSE_ESTIMATOR).onTrue(new InstantCommand(this.driveTrain::resetPoseToMidSubwoofer));
        // }
    }

    public void onRobotInit() {
        FlagUploader.uploadFlagsClass();
    }

    public void onTeleopInit() {
        if (this.getAutonomousCommand() != null) {
            this.getAutonomousCommand().cancel();
        }

        if (Flags.TelescopingArm.IS_ATTACHED) {
            if (Flags.TelescopingArm.USE_TEST_ELEVATOR_COMMAND) {
                this.telescopingArm.setDefaultCommand(new TestElevatorCommand(this.telescopingArm, this.primaryController));
                // NOTE: this command uses the joystick, so it is MUTUALLY EXCLUSIVE with other commands
                // Therefore, we immediately return so we don't run two commands using the same joysticks, because that would be weird.
                return;
            } else {
                this.telescopingArm.setDefaultCommand(new ElevatorControlCommand(this.telescopingArm, this.primaryController));
            }
        }
        if (Flags.DriveTrain.IS_ATTACHED) {
            if (Flags.DriveTrain.USE_TEST_DRIVE_COMMAND) {
                this.driveTrain.setDefaultCommand(new TestDriveCommand(this.driveTrain, this.primaryController));
            } else {
                this.driveTrain.setDefaultCommand(new ManualDriveCommand(this.driveTrain, this.primaryController));
            }
        }
        /* // put another slash to undestroy this
        if (Flags.Climber.IS_ATTACHED) {
            if (Flags.Climber.USE_TEST_CLIMBER_COMMAND) {
                this.climber.setDefaultCommand(new TestClimberCommand(this.climber, this.primaryController));
            } else {
                this.climber.setDefaultCommand(new ManualClimberCommand(this.climber, this.primaryController));
            }
        }
        if (Flags.CoralIntake.IS_ATTACHED) {
            if (Flags.CoralIntake.USE_TEST_CORAL_COMMAND) {
                this.coralIntake.setDefaultCommand(new TestCoralIntakeCommand(this.coralIntake, this.primaryController));
            } else {
                this.coralIntake.setDefaultCommand(new ManualCoralIntakeCommand(this.coralIntake, this.primaryController));
            }
        }
        // */
    }

    public Command getAutonomousCommand() {
        // This method will return an actual auton path once we implement it & switch in the comment.
        return new InstantCommand(); //this.autonChooser.getSelected();
    }

    public void onTeleopPeriodic() {
        // this.powerHandler.updateNT();
    }

    public void onRobotPeriodic() {

    }
}
