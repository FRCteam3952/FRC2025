package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A wrapper around {@link CommandPS5Controller}. Our left joystick is not working right now though, so I'm just going to use the right side one for now.
 */
public class PS5ControllerIO implements ControllerIO {
    public static final double IGNORE_DELTA = 0.08;

    private final CommandPS5Controller controller;

    // Because we have to funnel all the inputs through the AbstractControllerInputs
    // class, we have to have one set of triggers to update inputs
    // and then another set of inputs that get their values from AbstractControllerInputs
    // in the 'frontend'
    public final Trigger leftButtonTrigger;
    public final Trigger rightButtonTrigger;
    public final Trigger upperButtonTrigger;
    public final Trigger lowerButtonTrigger;
    public final Trigger leftShoulderButtonTrigger;
    public final Trigger rightShoulderButtonTrigger;
    public final Trigger leftShoulderTriggerTrigger;
    public final Trigger rightShoulderTriggerTrigger;

    public PS5ControllerIO(CommandPS5Controller controller) {
        this.controller = controller;

        this.leftButtonTrigger = this.controller.square();
        this.rightButtonTrigger = this.controller.circle();
        this.upperButtonTrigger = this.controller.triangle();
        this.lowerButtonTrigger = this.controller.cross();
        this.leftShoulderButtonTrigger = this.controller.L1();
        this.rightShoulderButtonTrigger = this.controller.R1();
        this.leftShoulderTriggerTrigger = this.controller.L2();
        this.rightShoulderTriggerTrigger = this.controller.R2();
    }

    @Override
    public void updateInputs(ControllerInputsAutoLogged inputs)
    {
        inputs.leftVerticalMovement = this.controller.getLeftY();
        inputs.leftHorizontalMovement = this.controller.getLeftX();
        inputs.rightVerticalMovement = this.controller.getRightY();
        inputs.rightHorizontalMovement = this.controller.getRightX();

        inputs.leftButton = this.leftButtonTrigger.getAsBoolean();
        inputs.upperButton = this.upperButtonTrigger.getAsBoolean();
        inputs.rightButton = this.rightButtonTrigger.getAsBoolean();
        inputs.lowerButton = this.lowerButtonTrigger.getAsBoolean();

        inputs.leftShoulderButton = this.leftShoulderButtonTrigger.getAsBoolean();
        inputs.leftShoulderTrigger = this.leftShoulderTriggerTrigger.getAsBoolean();
        inputs.rightShoulderButton = this.rightShoulderButtonTrigger.getAsBoolean();
        inputs.rightShoulderTrigger = this.rightShoulderTriggerTrigger.getAsBoolean();

        inputs.POV = this.controller.getHID().getPOV();
    }
}
