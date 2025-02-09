package frc.robot.commands.testers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.controllers.Controller;

public class TestElevatorCommand extends Command {
    private final ElevatorSubsystem telescopingArm;
    private final Controller joystick;

    public TestElevatorCommand(ElevatorSubsystem telescopingArm, Controller joystick) {
        this.telescopingArm = telescopingArm;
        this.joystick = joystick;

        addRequirements(telescopingArm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double rightSpeed = this.joystick.getRightVerticalMovement();
        double leftSpeed  = this.joystick.getLeftVerticalMovement();

        // right is lead, left is follow.
        this.telescopingArm.setRawSpeeds(rightSpeed, leftSpeed);
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
