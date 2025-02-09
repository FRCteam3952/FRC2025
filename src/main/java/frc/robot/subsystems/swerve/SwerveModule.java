// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableType;
import frc.robot.Constants.NetworkTablesConstants;
import frc.robot.Flags;
import frc.robot.util.NetworkTablesUtil;
import org.littletonrobotics.junction.Logger;

/**
 * There are three existing readouts for the module's rotational position.
 *
 * <p>
 * The first is the integrated encoder on the motor controller (CANSparkMax), which uses integration of velocity to determine position.
 * <p>
 * The second is the CANcoder, which offers a physically measured value for the position.
 * The CANcoder has two output modes:
 *
 * <ul>
 *     <li> one mode which only measures from [-PI, PI] or from [0, 2 * PI] (i.e. there exists a discontinuity where the values will loop around to remain within a unit circle),
 *     <li> and a second mode which does not contain any discontinuity.
 * </ul>
 * <p>
 * The non-discontinuous mode's value can be set in code (the discontinuous cannot), but this should be avoided unless resetting to the value of the discontinuous output.
 * <p>
 * The CANcoder mode containing the discontinuity should always be considered the most accurate reference, and is the one we use when initializing everything on robot code startup.
 * <p>
 * The CANcoder mode without the discontinuity is still considered accurate, though should be reset on robot code startup since its value is only properly reset by firmware when power-cycled.
 * <p>
 * For the purposes of a standardized way to reference these three encoders, the motor controller's encoder will be referred to as the "relative encoder" (as that is the type name of the object returned by REV's libraries),
 * the CANcoder's continuous (non-looping) encoder will be referred to as the "absolute encoder" (since its value is still considered absolute as long as it isn't erroneously changed in code),
 * and the CANcoder's discontinuous encoder will be referred to as the "absolutely-absolute encoder".
 * <p>
 * The drive motors do not follow this naming scheme as they only have a relative encoder.
 */
public class SwerveModule {
    private static final double SWERVE_ROTATION_OPTIMIZATION_THRESH_DEG = 90;

    private final String name;
    private final GenericPublisher rotationPublisher;
    private final SwerveModuleIO io;
    private SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    /**
     * @param driveMotorCANID     CAN ID for the drive motor.
     * @param turningMotorCANID   CAN ID for the turning motor.
     * @param turningEncoderCANID CAN ID for the turning absolute encoder.
     * @param name                The name of this module.
     * @param invertDriveMotor    Whether to invert the drive motor. This should be the same across all modules.
     * @param invertTurnMotor     Whether to invert the turn motor. This should be the same across all modules.
     */
    public SwerveModule(String name, SwerveModuleIO io) {
        this.io = io;
        this.name = name;

        rotationPublisher = NetworkTablesUtil.getPublisher(NetworkTablesConstants.MAIN_TABLE_NAME, name + "_rot", NetworkTableType.kDouble);

        // TODO: fix these print statements that we had before for new IO paradigm!!!!!!:
        // System.out.println(name + " is at abs-abs " + this.turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() + ", abs " + this.turnAbsoluteEncoder.getPosition().getValueAsDouble());
        // System.out.println("magnet health of " + name + " is " + this.turnAbsoluteEncoder.getMagnetHealth().getValue());
        // if (this.turnAbsoluteEncoder.getFault_BadMagnet().getValue()) {
        //     System.out.println(name + " has a bad magnet");
        // }


        // io.setDriveMotorInverted(invertDriveMotor);
        // io.setTurnMotorInverted(invertTurnMotor);

        // this.io.setDriveMotorSmartCurrentLimit(30);
        // this.io.setDriveMotorSecondaryCurrentLimit(100);

        // this.io.setTurnMotorSmartCurrentLimit(30);
        // this.io.setTurnMotorSecondaryCurrentLimit(100);

        // Circumference / Gear Ratio (L2 of MK4i). This evaluates to ~1.86 inches/rotation, which is close to experimental values.
        // We are therefore using the calculated value. (Thanks Ivan)
        // Since everything else is in meters, convert to meters.
        //this.driveEncoder.setPositionConversionFactor(Units.inchesToMeters(4 * Math.PI / 6.75));
        //this.driveEncoder.setVelocityConversionFactor(Units.inchesToMeters(4 * Math.PI / 6.75) / 60);


        //System.out.println(this.name + " inverts drive: " + this.driveMotor.getInverted() + " turn: " + this.turnMotor.getInverted());
        // System.out.println(this.name + " abs pos " + RobotMathUtil.roundNearestHundredth(this.turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()));
    }

    /**
     * ORIGINAL: {@link SwerveModuleState#optimize(SwerveModuleState, Rotation2d)}
     * <p>
     * Minimize the change in heading the desired swerve module state would require by potentially
     * reversing the direction the wheel spins. If this is used with the PIDController class's
     * continuous input functionality, the furthest a wheel will ever rotate is {@link SwerveModule#SWERVE_ROTATION_OPTIMIZATION_THRESH_DEG SWERVE_ROTATION_OPTIMIZATION_THRESH_DEG} degrees.
     *
     * <p>
     * NOTE: Team 3419 suggested this change on ChiefDelphi for mechanical reasons (sending the motors in the opposite direction at top speed is bad),
     * but we are primarily using this to resolve a separate issue where occasionally one swerve module would turn in a direction opposite of the other modules when executing a direction change of 90 degrees.
     *
     * <p>
     * ChiefDelphi comment: <a href="https://www.chiefdelphi.com/t/swerve-pid-continuous-input-and-swerve-state-optimization/416292/6">...</a>
     * <p>
     * Team 3419's code with the relevant change: <a href="https://github.com/RoHawks/UniversalSwerve/blob/2fb7c0c9c9d7d3def7ba680bcd48b4b5456f09e1/src/main/java/universalSwerve/SwerveDrive.java#L267">...</a>
     *
     * <p>
     * Change made: only optimize the rotation direction when the angle is greater than a certain amount (higher than 90deg).
     * Original value in WPILIB code: 90 degrees.
     * New value determined by {@link SwerveModule#SWERVE_ROTATION_OPTIMIZATION_THRESH_DEG SWERVE_ROTATION_OPTIMIZATION_THRESH_DEG}
     *
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
     * @return Optimized swerve module state.
     */
    private static SwerveModuleState optimize(
            SwerveModuleState desiredState, Rotation2d currentAngle) {
        var delta = desiredState.angle.minus(currentAngle);
        // System.out.println(" -------------------------------------------------------- ");
        // System.out.println("delta: " + delta);
        if (Math.abs(delta.getDegrees()) > SWERVE_ROTATION_OPTIMIZATION_THRESH_DEG) {
            // System.out.println("optimizing!");
            // System.out.println(" -------------------------------------------------------- ");
            return new SwerveModuleState(
                    -desiredState.speedMetersPerSecond,
                    desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
        } else {
            // System.out.println("not optimizing :(");
            // System.out.println(" -------------------------------------------------------- ");
            return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        }
    }

    private SwerveModuleState getCosineCompensatedState(SwerveModuleState desiredState) {
        double cosineScalar = 1;
        // Taken from the CTRE SwerveModule class.
        // https://api.ctr-electronics.com/phoenix6/release/java/src-html/com/ctre/phoenix6/mechanisms/swerve/SwerveModule.html#line.46
        /* From FRC 900's whitepaper, we add a cosine compensator to the applied drive velocity */
        /* To reduce the "skew" that occurs when changing direction */
        /* If error is close to 0 rotations, we're already there, so apply full power */
        /* If the error is close to 0.25 rotations, then we're 90 degrees, so movement doesn't help us at all */
        cosineScalar = desiredState.angle.minus(this.getAbsoluteModuleState().angle).getCos();
        /* Make sure we don't invert our drive, even though we shouldn't ever target over 90 degrees anyway */
        // Are we optimizing target angles??
        if (cosineScalar < 0.0) {
            cosineScalar = 1;
        }

        return new SwerveModuleState(desiredState.speedMetersPerSecond * cosineScalar, desiredState.angle);
    }

    public double getDriveAmperage() {
        return this.inputs.driveCurrentAmps;
    }

    /**
     * Sets all non-absolutely-absolute encoders to the absolutely-absolute CANcoder value.
     *
     * @see SwerveModule
     */
    public void resetEncodersToAbsoluteValue() {
        // this.turnAbsoluteEncoder.setPosition(this.turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble());
        this.io.setTurnEncoderPosition(this.getTurnAbsEncoderPosition());
    }

    /**
     * @return The name of this module.
     */
    public String getName() {
        return this.name;
    }

    /**
     * @return The number of meters the drive motor has traveled from the starting position.
     */
    public double getDrivePosition() {
        return this.inputs.drivePositionRad;
    }

    /**
     * @return The absolute encoder's position in radians
     * @see SwerveModule
     */
    public double getTurnAbsEncoderPosition() {
        // ORIGINAL UNITS: rotations. Converted to radians.
        return this.inputs.turnPosition.getRadians();
    }

    /**
     * @return The absolute encoder's velocity in radians per second.
     * @see SwerveModule
     */
    public double getTurnAbsEncoderVelocity() {
        // ORIGINAL UNITS: rotations per second. Converted to radians per second.
        return this.inputs.turnVelocityRadPerSec;
    }

    /**
     * This value is exact if the CANcoder is tuned properly in Phoenix Tuner X. Use this value for all further calibration.
     *
     * @return The absolutely-absolute position of the CANcoder in radians.
     * @see SwerveModule
     */
    public double getTurnAbsolutelyAbsolutePosition() {
        // ORIGINAL UNITS: rotations. Converted to radians.
        return this.inputs.absoluteEncoderAbsolutePosition.getRadians();
    }

    /**
     * @return The velocity of the drive encoder, in m/sec.
     */
    public double getDriveVelocity() {
        return this.inputs.driveVelocityRadPerSec;
    }

    /**
     * @return The relative encoder's position in radians.
     * @see SwerveModule
     */
    public double getTurnRelativePosition() {
        return this.inputs.turnPosition.getRadians();
    }

    /**
     * @return The relative encoder's velocity in radians/sec.
     */
    public double getTurnRelativeVelocity() {
        return this.inputs.turnVelocityRadPerSec;
    }

    /**
     * Uses the absolute encoder position.
     *
     * @return The current state of the module using the absolute encoder's position.
     */
    
    public SwerveModuleState getAbsoluteModuleState() {
        return new SwerveModuleState(inputs.driveVelocityRadPerSec, new Rotation2d(this.getTurnAbsEncoderPosition()));
    }

    /**
     * Uses the absolutely-absolute encoder position.
     *
     * @return The current state of the module using the absolutely-absolute encoder's position.
     */
    public SwerveModuleState getAbsoluteAbsoluteModuleState() {
        return new SwerveModuleState(inputs.driveVelocityRadPerSec, new Rotation2d(this.getTurnAbsolutelyAbsolutePosition()));
    }

    /**
     * Uses the absolute encoder position.
     *
     * @return The current position of the module using the absolute encoder's position.
     */
    public SwerveModulePosition getAbsoluteModulePosition() {
        return new SwerveModulePosition(this.inputs.drivePositionRad, new Rotation2d(this.getTurnAbsEncoderPosition()));
    }

    /**
     * Uses the relative encoder position.
     *
     * @return The current state of the module using the relative encoder's position.
     */
    public SwerveModuleState getRelativeModuleState() {
        return new SwerveModuleState(inputs.driveVelocityRadPerSec, this.inputs.turnPosition);
    }

    /**
     * Uses the relative encoder position.
     *
     * @return The current position of the module using the relative encoder's position.
     */
    public SwerveModulePosition getRelativeModulePosition() {
        return new SwerveModulePosition(this.inputs.drivePositionRad, this.inputs.turnPosition);
    }

    /**
     * Directly set the voltage outputs of the motors.
     *
     * @param driveVoltage Voltage of drive motor
     * @param turnVoltage  Voltage of turn motor
     */
    public void setVoltages(double driveVoltage, double turnVoltage) {
        io.setDriveMotorVoltage(driveVoltage);
        io.setTurnMotorVoltage(turnVoltage);
    }

    /**
     * Stops the motors by sending a voltage output of 0 to both the drive and turn motors.
     */
    public void stop() {
        setVoltages(0, 0);
    }

    /**
     * Sets the module's target state to absolute zero.
     */
    public void rotateToAbsoluteZero() {
        SwerveModuleState zeroedState = new SwerveModuleState();
        this.setDesiredStateNoOptimize(zeroedState);
    }

    /**
     * Sets the module's target state to absolute zero. This method accepts a debug index.
     *
     * @param debugIdx The debug index of the array.
     * @see DriveTrainSubsystemOld#optimizedTargetStates
     */
    public void rotateToAbsoluteZero(int debugIdx) {
        SwerveModuleState zeroedState = new SwerveModuleState();
        this.setDesiredStateNoOptimize(zeroedState, debugIdx);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state;
        if (Flags.DriveTrain.SWERVE_MODULE_OPTIMIZATION) {
            state = optimize(desiredState, new Rotation2d(this.getTurnAbsEncoderPosition()));
        } else {
            state = desiredState;
        }

        this.setDesiredStateNoOptimize(state);
    }

    /**
     * Sets the desired state for the model. This method accepts a debug index.
     *
     * @param desiredState Desired state with speed and angle.
     * @param debugIdx     The debug index of the array.
     * @see DriveTrainSubsystem#optimizedTargetStates
     */
    public void setDesiredState(SwerveModuleState desiredState, int debugIdx) {
        SwerveModuleState state;
        if (Flags.DriveTrain.SWERVE_MODULE_OPTIMIZATION) {
            state = optimize(desiredState, new Rotation2d(this.getTurnAbsEncoderPosition()));
        } else {
            state = desiredState;
        }

        this.setDesiredStateNoOptimize(state, debugIdx);
    }

    /**
     * Sets the desired state without optimizing for efficiency.
     *
     * @param desiredState Desired state with speed and angle.
     * @see SwerveModule#optimize(SwerveModuleState, Rotation2d)
     */
    public void setDesiredStateNoOptimize(SwerveModuleState desiredState) {
        if (Flags.DriveTrain.ENABLE_COSINE_COMPENSATOR) {
            desiredState = getCosineCompensatedState(desiredState);
        }

        this.setDriveDesiredState(desiredState);
        this.setRotationDesiredState(desiredState);

        if (Math.abs(desiredState.speedMetersPerSecond) < 0.01 && Math.abs(this.getTurnRelativeVelocity()) < 0.01) {
            this.resetEncodersToAbsoluteValue();
            // System.out.println("resetting encoders");
        }
    }

    /**
     * Sets the desired state without optimizing for efficiency. This method accepts a debug index
     *
     * @param desiredState Desired state with speed and angle.
     * @param debugIdx     The debug index of the array.
     * @see SwerveModule#optimize(SwerveModuleState, Rotation2d)
     * @see DriveTrainSubsystem#optimizedTargetStates
     */
    public void setDesiredStateNoOptimize(SwerveModuleState desiredState, int debugIdx) {
        DriveTrainSubsystem.optimizedTargetStates[debugIdx] = desiredState;
        setDesiredState(desiredState);
    }

    /**
     * Sets the drive motors to follow a desired state. The state will NOT be optimized.
     *
     * @param optimizedDesiredState The desired module state.
     * @see SwerveModule#setDesiredState(SwerveModuleState)
     */
    private void setDriveDesiredState(SwerveModuleState optimizedDesiredState) {
        // Calculate the drive output from the drive PID controller.
        if (Flags.DriveTrain.ENABLED && Flags.DriveTrain.ENABLE_DRIVE_MOTORS && Flags.DriveTrain.DRIVE_PID_CONTROL) {
            io.setDriveVelocity(optimizedDesiredState.speedMetersPerSecond);
        }

        // TODO: add flag for the print statements and stuff here turning on
        // double vel = inputs.driveVelocityRadPerSec;
        // double tar = optimizedDesiredState.speedMetersPerSecond;
        // double ratio = 0;
        // if (Math.abs(tar) > 0.01) {
        //     ratio = vel / tar;
        // }
        //System.out.println(this.name + " velocity: " + nearestHundredth(inputs.driveVelocityRadPerSec) + " target speed: " + nearestHundredth(optimizedDesiredState.speedMetersPerSecond) + ", ratio: " + nearestHundredth(ratio));
        //System.out.println(this.name + ", position: " + this.driveEncoder.getPosition());
    }

    /**
     * Sets the rotation motors to follow a desired state. The state will NOT be optimized.
     *
     * @param optimizedDesiredState The desired module state.
     * @see SwerveModule#setDesiredState(SwerveModuleState)
     */
    private void setRotationDesiredState(SwerveModuleState optimizedDesiredState) {
        // System.out.println("turn encoder at: " + RobotMathUtil.roundNearestHundredth(this.turnEncoder.getPosition()) + ", abs val: " + RobotMathUtil.roundNearestHundredth(this.getTurningAbsEncoderPositionConverted()));
        if (Flags.DriveTrain.ENABLED && Flags.DriveTrain.ENABLE_TURN_MOTORS && Flags.DriveTrain.TURN_PID_CONTROL) {
            io.setTurnPosition(optimizedDesiredState.angle);
        }

        rotationPublisher.setDouble(this.getTurnRelativePosition());
        // System.out.println("target: " + nearestHundredth(bringAngleWithinUnitCircle(optimizedDesiredState.angle.getDegrees())) + ", rel: " + nearestHundredth(bringAngleWithinUnitCircle(this.getTurnRelativePosition() * 180 / Math.PI)) + ", abs: " + nearestHundredth(bringAngleWithinUnitCircle(this.getTurnAbsEncoderPosition() * 180 / Math.PI)));
    }

    /**
     * Sends a direct speed to the drive motor.
     *
     * @param speed The desired speed, [-1, 1]
     */
    public void directDrive(double voltage) {
        if (Flags.DriveTrain.ENABLED && Flags.DriveTrain.ENABLE_DRIVE_MOTORS) {
            this.io.setDriveMotorVoltage(voltage);
        }
    }

    /**
     * Sends a direct speed to the turn motor.
     *
     * @param speed The desired speed, [-1, 1]
     */
    public void directTurn(double voltage) {
        if (Flags.DriveTrain.ENABLED && Flags.DriveTrain.ENABLE_TURN_MOTORS) {
            this.io.setTurnMotorVoltage(voltage);
        }
    }

    public void periodic() {
        this.io.updateInputs(inputs);
        Logger.processInputs("Drive/" + this.name, inputs);
    }
}
