package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.geometry.Rotation2d;

public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
        public boolean driveConnected = false;
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public boolean turnConnected = false;
        public Rotation2d turnPosition = new Rotation2d();
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;

        public Rotation2d absoluteEncoderAbsolutePosition = new Rotation2d();

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(SwerveModuleIOInputs inputs) {}
  
    /** Run the drive motor at the specified open loop value. */
    public default void setDriveOpenLoop(double output) {}
  
    /** Run the turn motor at the specified open loop value. */
    public default void setTurnOpenLoop(double output) {}
  
    /** Run the drive motor at the specified velocity. */
    public default void setDriveVelocity(double velocityRadPerSec) {}
  
    /** Run the turn motor to the specified rotation. */
    public default void setTurnPosition(Rotation2d rotation) {}

    // Begin our methods:
    public void setTurnEncoderPosition(double position);

    public void setDriveMotorVoltage(double input);

    public void setTurnMotorVoltage(double input);

    public void setDrivePIDControllerReference(double desiredSpeed, SparkBase.ControlType controlType);
    
    public void setTurnPIDControllerReference(double desiredAngle, SparkBase.ControlType controlType);
}
