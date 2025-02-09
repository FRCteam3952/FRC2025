package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;

public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
        // public boolean driveConnected = false;
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        // public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        // public boolean turnConnected = false;
        public Rotation2d turnPosition = new Rotation2d();
        public double turnVelocityRadPerSec = 0.0;
        // public double turnAppliedVolts = 0.0;
        // public double turnCurrentAmps = 0.0;

        public Rotation2d absoluteEncoderAbsolutePosition = new Rotation2d();

        // public double[] odometryTimestamps = new double[] {};
        // public double[] odometryDrivePositionsRad = new double[] {};
        // public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    /** Updates the set of loggable inputs. */
    public abstract void updateInputs(SwerveModuleIOInputs inputs);
  
    /** Run the drive motor at the specified open loop value. */
    // public abstract void setDriveOpenLoop(double output);
  
    /** Run the turn motor at the specified open loop value. */
    // public abstract void setTurnOpenLoop(double output);
  
    /** Run the drive motor at the specified velocity. */
    public abstract void setDriveVelocity(double velocityRadPerSec);
  
    /** Run the turn motor to the specified rotation. */
    public abstract void setTurnPosition(Rotation2d rotation);

    // Begin our methods:
    public abstract void setTurnEncoderPosition(double position);

    public abstract void setDriveMotorVoltage(double input);

    public abstract void setTurnMotorVoltage(double input);
}
