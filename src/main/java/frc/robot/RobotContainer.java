package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OperatorConstants.ControllerConstants;
import frc.robot.commands.*;
import frc.robot.controllers.AbstractController;
import frc.robot.controllers.FlightJoystick;
import frc.robot.controllers.NintendoProController;
import frc.robot.controllers.PS5Controller;
import frc.robot.subsystems.PowerHandler;
import frc.robot.subsystems.staticsubsystems.ColorSensor;
import frc.robot.subsystems.staticsubsystems.LimeLight;
import frc.robot.subsystems.staticsubsystems.RobotGyro;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;
import frc.robot.util.AprilTagHandler;
import frc.robot.util.ControlHandler;
import frc.robot.util.NetworkTablesUtil;
import frc.robot.util.Util;

public class RobotContainer {
    private static final GenericPublisher COLOR_SENSOR_PUB = NetworkTablesUtil.getPublisher("robot", "color_sensor_sees_note", NetworkTableType.kBoolean);

    private final DriveTrainSubsystem driveTrain;

    private final FlightJoystick sideJoystick = new FlightJoystick(new CommandJoystick(OperatorConstants.RIGHT_JOYSTICK_PORT));
    private final NintendoProController nintendoProController = new NintendoProController(new CommandXboxController(OperatorConstants.NINTENDO_PRO_CONTROLLER));
    private final PS5Controller ps5Controller = new PS5Controller(new CommandPS5Controller(OperatorConstants.PS5_CONTROLLER));
    private final AbstractController primaryController = Flags.Operator.NINTENDO_SWITCH_CONTROLLER_AS_PRIMARY ? this.nintendoProController : this.ps5Controller;
    private final PowerHandler powerHandler = new PowerHandler();
    private final AprilTagHandler aprilTagHandler = new AprilTagHandler();

    private final SendableChooser<Command> autonChooser;

    public RobotContainer() {
        this.driveTrain = Util.createIfFlagElseNull(() -> new DriveTrainSubsystem(aprilTagHandler), Flags.DriveTrain.IS_ATTACHED);

        configureBindings();

        // Initialize static subsystems (this is a Java thing don't worry about it just copy it so that static blocks run on startup)
        LimeLight.poke();
        RobotGyro.poke();
        ColorSensor.poke();

        if(Flags.DriveTrain.IS_ATTACHED) {
            this.autonChooser = AutoBuilder.buildAutoChooser();
            SmartDashboard.putData("choose your auto", this.autonChooser);
        } else {
            this.autonChooser = null;
        }

        NetworkTablesUtil.getConnections();
    }

    private void configureBindings() {
        if (Flags.DriveTrain.IS_ATTACHED) {
            ControlHandler.get(this.nintendoProController, ControllerConstants.ZERO_SWERVE_MODULES).onTrue(this.driveTrain.rotateToAbsoluteZeroCommand());
        }
        ControlHandler.get(this.nintendoProController, ControllerConstants.ZERO_GYRO).onTrue(Commands.runOnce(() -> {
            if(Util.onBlueTeam()) {
                RobotGyro.resetGyroAngle();
            } else {
                RobotGyro.setGyroAngle(180);
            }
            this.driveTrain.setHeadingLockMode(false);
        }));

        if(Flags.DriveTrain.IS_ATTACHED) {
            ControlHandler.get(this.nintendoProController, ControllerConstants.RESET_POSE_ESTIMATOR).onTrue(new InstantCommand(this.driveTrain::resetPoseToMidSubwoofer));
        }
    }

    public void onRobotInit() {
        FlagUploader.uploadFlagsClass();
    }

    public void onTeleopInit() {
        this.getAutonomousCommand().cancel();

        if (Flags.DriveTrain.IS_ATTACHED) {
            RobotGyro.setGyroAngle(this.driveTrain.getPose().getRotation().getDegrees());
            if (Flags.DriveTrain.USE_TEST_DRIVE_COMMAND) {
                this.driveTrain.setDefaultCommand(new TestDriveCommand(this.driveTrain, this.primaryController));
            } else {
                this.driveTrain.setDefaultCommand(new ManualDriveCommand(this.driveTrain, this.primaryController, this.aprilTagHandler));
            }
        }
    }

    public Command getAutonomousCommand() {
        return this.autonChooser.getSelected();
    }

    public void onTeleopPeriodic() {
        this.powerHandler.updateNT();
    }
    
    public void onRobotPeriodic() {
        COLOR_SENSOR_PUB.setBoolean(ColorSensor.isNoteColor());
    }
}
