package frc.robot.commands.testers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controllers.Controller;
import frc.robot.subsystems.swerve.DriveTrainSubsystem;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.util.Util;

public class TestDriveCommand extends Command {
    private final DriveTrainSubsystem driveTrain;
    private final Controller joystick;

    public TestDriveCommand(DriveTrainSubsystem driveTrain, Controller joystick) {
        this.driveTrain = driveTrain;
        this.joystick = joystick;

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double joystickToVoltageCoefficient = 6.0; // made up constant by ivan, half of what addison chan said it should be
        this.driveTrain.directDriveSpeed(-this.joystick.getLeftVerticalMovement() * joystickToVoltageCoefficient);
        this.driveTrain.directTurnSpeed(this.joystick.getRightHorizontalMovement() * joystickToVoltageCoefficient);

        for (SwerveModule module : this.driveTrain.swerveModules) {
            System.out.println("name: " + module.getName() + ", abs abs: " + Util.nearestHundredth(module.getTurnAbsolutelyAbsolutePosition()) + ", abs: " + Util.nearestHundredth(module.getTurnAbsEncoderPosition()) + ", rel: " + Util.nearestHundredth(module.getTurnRelativePosition()));
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
