package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;


public interface GyroIO {
    @AutoLog
    class GyroIOInputs {
        public boolean connected = false;
        public double yawVelocityDegreesPerSecond = 0.0;
        public Rotation2d yawPosition = new Rotation2d();
        public double xAcceleration = 0.0;
        public double yAcceleration = 0.0;
        public double zAcceleration = 0.0;
    }

    default void updateInputs(GyroIOInputs inputs) {}
    default void resetGyroAngle() {}
    default void setGyroAngle(double deg) {}
}