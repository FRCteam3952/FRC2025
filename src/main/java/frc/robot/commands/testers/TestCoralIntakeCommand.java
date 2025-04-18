package frc.robot.commands.testers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controllers.AbstractController;
import frc.robot.subsystems.CoralIntakeSubsystem;

public class TestCoralIntakeCommand extends Command {
    private final CoralIntakeSubsystem coralIntake;
    private final AbstractController joystick;

    private final Trigger r;
    public TestCoralIntakeCommand(CoralIntakeSubsystem coralIntake, AbstractController joystick) {
        this.coralIntake = coralIntake;
        this.joystick = joystick;

        r = joystick.rightButton();

        addRequirements(coralIntake);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        coralIntake.setRawPivotSpeed(joystick.getRightVerticalMovement());
        if(r.getAsBoolean()) {
            coralIntake.setIntakeSpeed(0.3);
        } else {
            coralIntake.setIntakeSpeed(0);
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
