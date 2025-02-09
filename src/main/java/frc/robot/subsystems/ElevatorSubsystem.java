package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Robot;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.NetworkTablesUtil;
import frc.robot.util.ThroughboreEncoder;

public class ElevatorSubsystem extends SubsystemBase {

    private static final double MAX_HEIGHT = 1.0;
    private static final double MIN_HEIGHT = 0.0;

    private final SparkMax rightMotor;
    private final SparkMax leftMotor;

    private final RelativeEncoder rightMotorEncoder;
    private final RelativeEncoder leftMotorEncoder;

    private final ThroughboreEncoder rightThroughboreEncoder;
    private final ThroughboreEncoder leftThroughboreEncoder;

    private final DigitalInput leftLimitSwitch = new DigitalInput(Constants.PortConstants.DIO.LEFT_ELEVATOR_LIMIT); // dont forgor active low
    private final DigitalInput rightLimitSwitch = new DigitalInput(Constants.PortConstants.DIO.RIGHT_ELEVATOR_LIMIT);

    private final SparkClosedLoopController rightPIDController;
    private final SparkClosedLoopController leftPIDController;

    private final static double ABSOLUTE_DEGREES_PER_RELATIVE_DEGREES = 1426.64 / 8254.24;
    private final static double ROTATIONS_PER_METER_ASCENDED = Rotation2d.fromDegrees(742.5).getRotations() / 0.5;
    private final static double METERS_ASCENDED_PER_ROTATION = 1 / ROTATIONS_PER_METER_ASCENDED;

    // Guaranteed not to send values 
    public static final double MAX_SPEED = 0.15;

    private static final GenericPublisher leftHeightAbsPub = NetworkTablesUtil.getPublisher("robot", "leftElevAbsH", NetworkTableType.kDouble);
    private static final GenericPublisher rightHeightAbsPub = NetworkTablesUtil.getPublisher("robot", "rightElevAbsH", NetworkTableType.kDouble);
    private static final GenericPublisher leftHeightRelPub = NetworkTablesUtil.getPublisher("robot", "leftElevRelH", NetworkTableType.kDouble);
    private static final GenericPublisher rightHeightRelPub = NetworkTablesUtil.getPublisher("robot", "rightElevRelH", NetworkTableType.kDouble);

    public static final GenericPublisher leftHeightAbsRotPub = NetworkTablesUtil.getPublisher("robot", "leftElevAR", NetworkTableType.kDouble);
    public static final GenericPublisher rightHeightAbsRotPub = NetworkTablesUtil.getPublisher("robot", "rightElevAR", NetworkTableType.kDouble);

    public ElevatorSubsystem() {
        rightMotor = new SparkMax(Constants.PortConstants.CAN.RIGHT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        leftMotor = new SparkMax(Constants.PortConstants.CAN.LEFT_ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        rightMotorEncoder = rightMotor.getEncoder();
        leftMotorEncoder = leftMotor.getEncoder();

        rightThroughboreEncoder = new ThroughboreEncoder(Constants.PortConstants.DIO.RIGHT_CLIMB_ABS_ENCODER_ABS_PORT, 5, 4, Rotation2d.fromDegrees(9.40).getRotations(), false, true, true); // 742.5 deg = 50 cm up
        leftThroughboreEncoder = new ThroughboreEncoder(Constants.PortConstants.DIO.LEFT_CLIMB_ABS_ENCODER_ABS_PORT, 2, 3, -Rotation2d.fromDegrees(124.5).getRotations(), true, false, true); // 742.5 deg = 50 cm up

        this.rightPIDController = this.rightMotor.getClosedLoopController();
        this.leftPIDController = this.leftMotor.getClosedLoopController();

        // hard vertical limit 1830 degrees
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        SparkMaxConfig leftConfig = new SparkMaxConfig();

        leftConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(12);

        leftConfig.encoder
            .positionConversionFactor(360d * ABSOLUTE_DEGREES_PER_RELATIVE_DEGREES / 762.183 * METERS_ASCENDED_PER_ROTATION)
            .velocityConversionFactor(360d * ABSOLUTE_DEGREES_PER_RELATIVE_DEGREES / 60d / 762.183 * METERS_ASCENDED_PER_ROTATION);

        leftConfig.closedLoop
            .pidf(1, 0, 0, 0)
            .outputRange(-0.6, 0.6);

        
        rightConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(12);

        rightConfig.encoder
            .positionConversionFactor(360d * ABSOLUTE_DEGREES_PER_RELATIVE_DEGREES / 762.183 * METERS_ASCENDED_PER_ROTATION)
            .velocityConversionFactor(360d * ABSOLUTE_DEGREES_PER_RELATIVE_DEGREES / 60d / 762.183 * METERS_ASCENDED_PER_ROTATION);

        rightConfig.closedLoop
            .pidf(1, 0, 0, 0)
            .outputRange(-0.6, 0.6);

        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightMotorEncoder.setPosition(0);
        leftMotorEncoder.setPosition(0);

        this.rightThroughboreEncoder.name = "right";
        this.leftThroughboreEncoder.name = "left";

        //Robot.INSTANCE.addPeriodic(() -> {
        //    this.updateEncoders();
        //}, 1.0/200, 0);
    }

    private void resetEncodersIfLimit() {
        if (getLeftLimitSwitch() || getRightLimitSwitch()) {
            resetEncoders();
        }
    }

    private void updateEncoders() {
        this.leftThroughboreEncoder.periodic();
        this.rightThroughboreEncoder.periodic();
    }

    private void resetEncoders() {
        rightMotorEncoder.setPosition(0);
        leftMotorEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        updateEncoders();
        /* <--- do not comment out the slash asterisk, just add a double slash in front
        System.out.println("Right position = " + this.getRightThroughboreEncoderDistance() + "\n" + 
                           "Left position = " + this.getLeftThroughboreEncoderDistance() + "\n" + 
                           "Right position (meters)= " + this.getRightMetersAscended() + "\n" + 
                           "Left position (meters)= " + this.getLeftMetersAscended() + "\n" + 
                           "Right position (relative)= " + this.getRightRelativePosition() + "\n" + 
                           "Left position (relative)= " + this.getLeftRelativePosition() + "\n");
        // <-- leave both of these --> */
        resetEncodersIfLimit();

        rightHeightAbsPub.setDouble(this.getRightMetersAscended());
        leftHeightAbsPub.setDouble(this.getLeftMetersAscended());
        rightHeightRelPub.setDouble(this.getRightRelativePosition());
        leftHeightRelPub.setDouble(this.getLeftRelativePosition());
        //leftHeightAbsRotPub.setDouble(this.leftThroughboreEncoder.getAbsolutePosition().getDegrees());
        //rightHeightAbsRotPub.setDouble(this.leftThroughboreEncoder.getAbsolutePosition().getDegrees());
    }

    public void setTargetHeight(double heightMeters) {
        if(heightMeters <= MAX_HEIGHT && heightMeters >= MIN_HEIGHT){
            this.rightPIDController.setReference(heightMeters, SparkBase.ControlType.kPosition);
            this.leftPIDController.setReference(heightMeters, SparkBase.ControlType.kPosition);
        }
    }

    public void setRelativeEncodersToAbsolute() {
        rightMotorEncoder.setPosition(getRightMetersAscended());
        leftMotorEncoder.setPosition(getLeftMetersAscended());
    }

    public void setRawSpeeds(double rightSpeed, double leftSpeed) {
        this.rightMotor.set(rightSpeed);
        this.leftMotor.set(leftSpeed);
    }

    public Rotation2d getRightThroughboreEncoderDistance() {
        return this.rightThroughboreEncoder.getTotalDistance();
    }

    public Rotation2d getLeftThroughboreEncoderDistance() {
        return this.leftThroughboreEncoder.getTotalDistance();
    }

    public double getRightMetersAscended() {
        return this.getRightThroughboreEncoderDistance().getRotations() * METERS_ASCENDED_PER_ROTATION;
    }

    public double getLeftMetersAscended() {
        return this.getLeftThroughboreEncoderDistance().getRotations() * METERS_ASCENDED_PER_ROTATION;
    }

    public double getLeftRelativePosition() {
        return this.leftMotorEncoder.getPosition();
    }

    public double getRightRelativePosition() {
        return this.rightMotorEncoder.getPosition();
    }

    public boolean getLeftLimitSwitch() {
        return !this.leftLimitSwitch.get();
    }

    public boolean getRightLimitSwitch() {
        return !this.rightLimitSwitch.get();
    }
}
