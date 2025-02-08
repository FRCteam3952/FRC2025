package frc.robot.controllers;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface ControllerIO {
    @AutoLog
    public static class ControllerInputs {
        public double leftVerticalMovement = 0;
        public double leftHorizontalMovement = 0;
        public double rightVerticalMovement = 0;
        public double rightHorizontalMovement = 0;

        public boolean leftButton = false;
        public boolean upperButton = false;
        public boolean rightButton = false;
        public boolean lowerButton = false;

        public boolean leftShoulderButton = false;
        public boolean leftShoulderTrigger = false;
        public boolean rightShoulderButton = false;
        public boolean rightShoulderTrigger = false;

        public int POV = 0;
    }

    public abstract void updateInputs(ControllerInputsAutoLogged inputs);
}
