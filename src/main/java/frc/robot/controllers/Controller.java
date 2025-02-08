package frc.robot.controllers;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/*
 * Note: MUST CALL PERIODIC() every frame! 
 */
public class Controller {
    public static final double IGNORE_DELTA = 0.08;

    private final ControllerIO io;
    private ControllerInputsAutoLogged inputs;
    public final String name;

    public Optional<Trigger> leftButtonTrigger = Optional.empty();
    public Optional<Trigger> rightButtonTrigger = Optional.empty();
    public Optional<Trigger> upperButtonTrigger = Optional.empty();
    public Optional<Trigger> lowerButtonTrigger = Optional.empty();
    public Optional<Trigger> leftShoulderButtonTrigger = Optional.empty();
    public Optional<Trigger> rightShoulderButtonTrigger = Optional.empty();
    public Optional<Trigger> leftShoulderTriggerTrigger = Optional.empty();
    public Optional<Trigger> rightShoulderTriggerTrigger = Optional.empty();

    public Controller(ControllerIO io)
    {
        this.io = io;
        this.inputs = new ControllerInputsAutoLogged();
        this.name = this.io.getClass().toString().split(" ")[1];
    }

    public void periodic() {
        this.io.updateInputs(inputs);
        Logger.processInputs("Controller/" + this.name, inputs);
    }

    private static double deadzone(double val) {
        if (Math.abs(val) < IGNORE_DELTA) {
            return 0;
        }
        return val;
    }
    
    /**
     * Gets the horizontal movement of the right-side joystick of the controller.
     *
     * @return The horizontal movement of the right-side joystick. Positive value means the stick was moved to the right.
     */
    public double getRightHorizontalMovement()
    {
        return deadzone(this.inputs.rightHorizontalMovement);
    }

    /**
     * Gets the vertical movement of the right-side joystick of the controller.
     *
     * @return The vertical movement of the right-side joystick. Positive value means the stick was moved up.
     */
    public double getRightVerticalMovement()
    {
        return deadzone(this.inputs.rightVerticalMovement);
    }

    /**
     * Gets the horizontal movement of the left-side joystick of the controller.
     *
     * @return The horizontal movement of the left-side joystick. Positive value means the stick was moved right.
     */
    public double getLeftHorizontalMovement()
    {
        return deadzone(this.inputs.leftHorizontalMovement);
    }

    /**
     * Gets the vertical movement of the left-side joystick of the controller.
     *
     * @return The vertical movement of the left-side joystick. Positive value means the stick was moved up.
     */
    public double getLeftVerticalMovement()
    {
        return deadzone(this.inputs.leftVerticalMovement);
    }

    /**
     * On the right hand side on controllers, there's four buttons arranged in a diamond shape. Due to what I can only assume to be a stylistic decision, the PS5 controller decided to not go with X/Y/A/B, hence the interesting name choice.
     *
     * @return A bindable {@link Trigger} for the button at the top of the diamond on the right side of the controller.
     */
    public Trigger upperButton()
    {
        return this.upperButtonTrigger.orElseGet(() -> {
            Trigger trigger = new Trigger(() -> this.inputs.upperButton);
            this.upperButtonTrigger = Optional.of(trigger);
            return trigger;
        });
    }

    /**
     * On the right hand side on controllers, there's four buttons arranged in a diamond shape. Due to what I can only assume to be a stylistic decision, the PS5 controller decided to not go with X/Y/A/B, hence the interesting name choice.
     *
     * @return A bindable {@link Trigger} for the button on the left of the diamond on the right side of the controller.
     */
    public Trigger leftButton()
    {
        return this.leftButtonTrigger.orElseGet(() -> {
            Trigger trigger = new Trigger(() -> this.inputs.leftButton);
            this.leftButtonTrigger = Optional.of(trigger);
            return trigger;
        });
    }

    /**
     * On the right hand side on controllers, there's four buttons arranged in a diamond shape. Due to what I can only assume to be a stylistic decision, the PS5 controller decided to not go with X/Y/A/B, hence the interesting name choice.
     *
     * @return A bindable {@link Trigger} for the button on the right of the diamond on the right side of the controller.
     */
    public Trigger rightButton()
    {
        return this.rightButtonTrigger.orElseGet(() -> {
            Trigger trigger = new Trigger(() -> this.inputs.rightButton);
            this.rightButtonTrigger = Optional.of(trigger);
            return trigger;
        });
    }

    /**
     * On the right hand side on controllers, there's four buttons arranged in a diamond shape. Due to what I can only assume to be a stylistic decision, the PS5 controller decided to not go with X/Y/A/B, hence the interesting name choice.
     *
     * @return A bindable {@link Trigger} for the button at the bottom of the diamond on the right side of the controller.
     */
    public Trigger lowerButton()
    {
        return this.lowerButtonTrigger.orElseGet(() -> {
            Trigger trigger = new Trigger(() -> this.inputs.lowerButton);
            this.lowerButtonTrigger = Optional.of(trigger);
            return trigger;
        });
    }

    /**
     * Get the POV value.
     *
     * @return The POV value. -1 if not pressed, else 0 for up and increasing clockwise.
     * @see GenericHID#getPOV()
     */
    public int getPOV()
    {
        return this.inputs.POV;
    }

    /**
     * While holding the controller, the buttons on the far side of the controller. Gets the far button on the left side that is higher up. (bumper?)
     *
     * @return The upper button on the far side on the left.
     */
    public Trigger leftShoulderButton()
    {
        return this.leftShoulderButtonTrigger.orElseGet(() -> {
            Trigger trigger = new Trigger(() -> this.inputs.leftShoulderButton);
            this.leftShoulderButtonTrigger = Optional.of(trigger);
            return trigger;
        });
    }

    /**
     * While holding the controller, the buttons on the far side of the controller. Gets the far button on the right side that is higher up. (bumper?)
     *
     * @return The upper button on the far side on the right.
     */
    public Trigger rightShoulderButton()
    {
        return this.rightShoulderButtonTrigger.orElseGet(() -> {
            Trigger trigger = new Trigger(() -> this.inputs.rightShoulderButton);
            this.rightShoulderButtonTrigger = Optional.of(trigger);
            return trigger;
        });
    }

    /**
     * While holding the controller, the buttons on the far side of the controller. Gets the far button on the left side that is lower.
     *
     * @return The upper button on the far side on the left.
     */
    public Trigger leftShoulderTrigger()
    {
        return this.leftShoulderTriggerTrigger.orElseGet(() -> {
            Trigger trigger = new Trigger(() -> this.inputs.leftShoulderTrigger);
            this.leftShoulderTriggerTrigger = Optional.of(trigger);
            return trigger;
        });
    }

    /**
     * While holding the controller, the buttons on the far side of the controller. Gets the far button on the right side that is lower.
     *
     * @return The upper button on the far side on the right.
     */
    public Trigger rightShoulderTrigger()
    {
        return this.rightShoulderTriggerTrigger.orElseGet(() -> {
            Trigger trigger = new Trigger(() -> this.inputs.rightShoulderTrigger);
            this.rightShoulderTriggerTrigger = Optional.of(trigger);
            return trigger;
        });
    }
}
