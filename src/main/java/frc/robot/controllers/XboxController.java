package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A wrapper around {@link CommandXboxController}.
 */
public class XboxController extends AbstractController {
    public static final double IGNORE_DELTA = 0.08;

    private final CommandXboxController controller;

    public XboxController(CommandXboxController controller) {
        this.controller = controller;
    }

    @Override
    public double getRightHorizontalMovement() {
        return AbstractController.deadzone(controller.getRightX(), IGNORE_DELTA);
    }

    @Override
    public double getRightVerticalMovement() {
        return AbstractController.deadzone(controller.getRightY(), IGNORE_DELTA);
    }

    @Override
    public double getLeftHorizontalMovement() {
        return AbstractController.deadzone(controller.getLeftX(), IGNORE_DELTA);
    }

    @Override
    public double getLeftVerticalMovement() {
        return AbstractController.deadzone(controller.getLeftY(), IGNORE_DELTA);
    }

    @Override
    public boolean getRawButtonWrapper(int button) {
        return controller.getHID().getRawButton(button);
    }

    @Override
    public boolean getRawButtonReleasedWrapper(int button) {
        return controller.getHID().getRawButtonReleased(button);
    }

    @Override
    public boolean getRawButtonPressedWrapper(int button) {
        return controller.getHID().getRawButtonPressed(button);
    }

    @Override
    public Trigger button(int button) {
        return this.controller.button(button);
    }

    @Override
    public Trigger upperButton() {
        return this.controller.x();
    }

    @Override
    public Trigger leftButton() {
        return this.controller.y();
    }

    @Override
    public Trigger rightButton() {
        return this.controller.a();
    }

    @Override
    public Trigger lowerButton() {
        return this.controller.b();
    }

    @Override
    public int getPOV() {
        return this.controller.getHID().getPOV();
    }

    @Override
    public Trigger leftShoulderButton() {
        return this.controller.leftBumper();
    }

    @Override
    public Trigger rightShoulderButton() {
        return this.controller.rightBumper();
    }

    @Override
    public Trigger leftShoulderTrigger() {
        return this.controller.leftTrigger();
    }

    @Override
    public Trigger rightShoulderTrigger() {
        return this.controller.rightTrigger();
    }
}
