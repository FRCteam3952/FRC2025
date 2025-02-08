// package frc.robot.controllers;

// import org.littletonrobotics.junction.AutoLog;

// import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
// import edu.wpi.first.wpilibj2.command.button.Trigger;

// /**
//  * A wrapper around {@link CommandPS5Controller}. Our left joystick is not working right now though, so I'm just going to use the right side one for now.
//  */
// public class PS5Controller extends AbstractController {
//     @AutoLog
//     public static class PS5ControllerInputs {
//         public double leftVerticalMovement;
//         public double leftHorizontalMovement;
//         public double rightVerticalMovement;
//         public double rightHorizontalMovement;

//         public boolean leftButton;
//         public boolean upperButton;
//         public boolean rightButton;
//         public boolean bottomButton;

//         public boolean leftShoulderButton;
//         public boolean leftShoulderTrigger;
//         public boolean rightShoulderButton;
//         public boolean rightShoulderTrigger;

//         public int POV;
//     }
//     public static final double IGNORE_DELTA = 0.08;

//     private final CommandPS5Controller controller;

//     public PS5Controller(CommandPS5Controller controller) {
//         this.controller = controller;
//     }

//     private static double deadzone(double val) {
//         if (Math.abs(val) < IGNORE_DELTA) {
//             return 0;
//         }
//         return val;
//     }

//     @Override
//     public double getRightHorizontalMovement() {
//         return deadzone(controller.getRightX());
//     }

//     @Override
//     public double getRightVerticalMovement() {
//         return deadzone(controller.getRightY());
//     }

//     @Override
//     public double getLeftHorizontalMovement() {
//         return deadzone(controller.getLeftX());
//     }

//     @Override
//     public double getLeftVerticalMovement() {
//         return deadzone(controller.getLeftY());
//     }

//     @Override
//     public boolean getRawButtonWrapper(int button) {
//         return controller.getHID().getRawButton(button);
//     }

//     @Override
//     public boolean getRawButtonReleasedWrapper(int button) {
//         return controller.getHID().getRawButtonReleased(button);
//     }

//     @Override
//     public boolean getRawButtonPressedWrapper(int button) {
//         return controller.getHID().getRawButtonPressed(button);
//     }

//     @Override
//     public Trigger button(int button) {
//         return this.controller.button(button);
//     }

//     @Override
//     public Trigger upperButton() {
//         return this.controller.triangle();
//     }

//     @Override
//     public Trigger leftButton() {
//         return this.controller.square();
//     }

//     @Override
//     public Trigger rightButton() {
//         return this.controller.circle();
//     }

//     @Override
//     public Trigger lowerButton() {
//         return this.controller.cross();
//     }

//     @Override
//     public int getPOV() {
//         return this.controller.getHID().getPOV();
//     }

//     @Override
//     public Trigger leftShoulderButton() {
//         return this.controller.L1();
//     }

//     @Override
//     public Trigger rightShoulderButton() {
//         return this.controller.R1();
//     }

//     @Override
//     public Trigger leftShoulderTrigger() {
//         return this.controller.L2();
//     }

//     @Override
//     public Trigger rightShoulderTrigger() {
//         return this.controller.R2();
//     }
// }
