package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * An abstract class for the joystick wrappers. Contains the common methods that we wrap around.
 */
public abstract class AbstractController {
    static double deadzone(double val, double deadzone) {
        if(Math.abs(val) < deadzone) {
            return 0;
        }
        return val;
    }

    /**
     * Gets the horizontal movement of the right-side joystick of the controller.
     *
     * @return The horizontal movement of the right-side joystick. Positive value means the stick was moved to the right.
     */
    public abstract double getRightHorizontalMovement();

    /**
     * Gets the vertical movement of the right-side joystick of the controller.
     *
     * @return The vertical movement of the right-side joystick. Positive value means the stick was moved up.
     */
    public abstract double getRightVerticalMovement();

    /**
     * Gets the horizontal movement of the left-side joystick of the controller.
     *
     * @return The horizontal movement of the left-side joystick. Positive value means the stick was moved right.
     */
    public abstract double getLeftHorizontalMovement();

    /**
     * Gets the vertical movement of the left-side joystick of the controller.
     *
     * @return The vertical movement of the left-side joystick. Positive value means the stick was moved up.
     */
    public abstract double getLeftVerticalMovement();

    /**
     * This method should not be used due to the lack of button number standardization in WPILIB, and it is preferable to use a wrapper that returns a {@link Trigger} for better usage of command-based functionality (or call {@link Trigger#getAsBoolean()}).
     * <p>
     * If a method like this is still desired, use a wrapper method instead so that buttons are applicable to different controller implementations.
     *
     * @param button The raw button number (check Driver Station)
     * @return Whether the button at a raw button index is being held down.
     */
    public abstract boolean getRawButtonWrapper(int button);

    /**
     * This method should not be used due to the lack of button number standardization in WPILIB, and it is preferable to use a wrapper that returns a {@link Trigger} for better usage of command-based functionality (or call {@link Trigger#getAsBoolean()}).
     * <p>
     * If a method like this is still desired, use a wrapper method instead so that buttons are applicable to different controller implementations.
     *
     * @param button The raw button number (check Driver Station)
     * @return Whether the button at a raw button index was newly released in the last tick (i.e. you want to check for a button release without constantly triggering when the button isn't pressed).
     */
    public abstract boolean getRawButtonReleasedWrapper(int button);

    /**
     * This method should not be used due to the lack of button number standardization in WPILIB, and it is preferable to use a wrapper that returns a {@link Trigger} for better usage of command-based functionality (or call {@link Trigger#getAsBoolean()}).
     * <p>
     * If a method like this is still desired, use a wrapper method instead so that buttons are applicable to different controller implementations.
     *
     * @param button The raw button number (check Driver Station)
     * @return Whether the button at a raw button index was newly pressed in the last tick (i.e. you want to check for a button press without constantly triggering when the button is pressed).
     */
    public abstract boolean getRawButtonPressedWrapper(int button);

    /**
     * This method should not be used due to the lack of button number standardization in WPILIB, and it is preferable to use one of the wrappers so that buttons are applicable to different controller implementations.
     *
     * @param button The raw button number (check Driver Station)
     * @return A bindable {@link Trigger} for the requested button ID.
     */
    public abstract Trigger button(int button);

    /**
     * On the right hand side on controllers, there's four buttons arranged in a diamond shape. Due to what I can only assume to be a stylistic decision, the PS5 controller decided to not go with X/Y/A/B, hence the interesting name choice.
     *
     * @return A bindable {@link Trigger} for the button at the top of the diamond on the right side of the controller.
     */
    public abstract Trigger upperButton();

    /**
     * On the right hand side on controllers, there's four buttons arranged in a diamond shape. Due to what I can only assume to be a stylistic decision, the PS5 controller decided to not go with X/Y/A/B, hence the interesting name choice.
     *
     * @return A bindable {@link Trigger} for the button on the left of the diamond on the right side of the controller.
     */
    public abstract Trigger leftButton();

    /**
     * On the right hand side on controllers, there's four buttons arranged in a diamond shape. Due to what I can only assume to be a stylistic decision, the PS5 controller decided to not go with X/Y/A/B, hence the interesting name choice.
     *
     * @return A bindable {@link Trigger} for the button on the right of the diamond on the right side of the controller.
     */
    public abstract Trigger rightButton();

    /**
     * On the right hand side on controllers, there's four buttons arranged in a diamond shape. Due to what I can only assume to be a stylistic decision, the PS5 controller decided to not go with X/Y/A/B, hence the interesting name choice.
     *
     * @return A bindable {@link Trigger} for the button at the bottom of the diamond on the right side of the controller.
     */
    public abstract Trigger lowerButton();

    /**
     * Get the POV value.
     *
     * @return The POV value. -1 if not pressed, else 0 for up and increasing clockwise.
     * @see GenericHID#getPOV()
     */
    public abstract int getPOV();

    /**
     * While holding the controller, the buttons on the far side of the controller. Gets the far button on the left side that is higher up. (bumper?)
     *
     * @return The upper button on the far side on the left.
     */
    public abstract Trigger leftShoulderButton();

    /**
     * While holding the controller, the buttons on the far side of the controller. Gets the far button on the right side that is higher up. (bumper?)
     *
     * @return The upper button on the far side on the right.
     */
    public abstract Trigger rightShoulderButton();

    /**
     * While holding the controller, the buttons on the far side of the controller. Gets the far button on the left side that is lower.
     *
     * @return The upper button on the far side on the left.
     */
    public abstract Trigger leftShoulderTrigger();

    /**
     * While holding the controller, the buttons on the far side of the controller. Gets the far button on the right side that is lower.
     *
     * @return The upper button on the far side on the right.
     */
    public abstract Trigger rightShoulderTrigger();
}
