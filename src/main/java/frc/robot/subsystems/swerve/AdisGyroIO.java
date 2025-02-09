package frc.robot.subsystems.swerve;

// import frc.robot.subsystems.staticsubsystems.RobotGyro; // worst
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;


public class AdisGyroIO implements GyroIO{
    private final ADIS16470_IMU gyro = new ADIS16470_IMU();

    public AdisGyroIO() {
        gyro.calibrate();
        gyro.reset();
    }

    public void updateInputs(GyroIOInputs inputs) {
        inputs.yawVelocityDegreesPerSecond = gyro.getRate();
        inputs.yawPosition = new Rotation2d(-Math.toRadians(gyro.getAngle(IMUAxis.kZ)));
        inputs.xAcceleration = gyro.getAccelX();
        inputs.yAcceleration = gyro.getAccelY();
        inputs.zAcceleration = gyro.getAccelZ();
    }

    /**
     * Reset the gyro such that the current heading is equal to 0.
     */
    public void resetGyroAngle() {
        gyro.reset();
    }

    /**
     * Set the gyro's current heading to a specific value.
     *
     * @param deg The desired current heading, in degrees.
     */
    public void setGyroAngle(double deg) {
        gyro.setGyroAngle(IMUAxis.kZ, deg);
    }

    /**
     * Rerun gyro calibration. The robot should not be moving, or need to be moving soon, when this occurs.
     */
    public void robotCalibrate() {
        gyro.calibrate();
    }
}