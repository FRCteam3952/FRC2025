package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.MutAngularVelocity;
import org.littletonrobotics.junction.AutoLog;
import frc.robot.subsystems.staticsubsystems.RobotGyro;

import static edu.wpi.first.units.Units.DegreesPerSecond;

public interface GyroIO {
    @AutoLog
    class GyroIOInputs {
        public boolean connected = false;
        public MutAngularVelocity yawVel = new MutAngularVelocity(0, 0, DegreesPerSecond);
        public Rotation2d yawPosition = new Rotation2d();
        public double accelX;
        public double accelY;
        public double accelZ;
//        public double yawVelocityRadPerSec = 0.0;
//        public double[] odometryYawTimestamps = new double[] {};
//        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    default void updateInputs(GyroIOInputs inputs) {}
    default void resetGyroAngle() {}
    default void setGyroAngle(double deg) {}
}