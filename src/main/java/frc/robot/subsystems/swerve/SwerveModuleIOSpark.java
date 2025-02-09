package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class SwerveModuleIOSpark implements SwerveModuleIO {
    private final SparkMax driveMotor;
    private final SparkMax turnMotor;

    private final SparkClosedLoopController drivePIDController;
    private final SparkClosedLoopController turnPIDController;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder; // + power = CCW, - power = CW

    private final CANcoder turnAbsoluteEncoder;
    
    public SwerveModuleIOSpark(
                int driveMotorCANID, 
                int turningMotorCANID, 
                int turningEncoderCANID, 
                boolean invertDriveMotor, 
                boolean invertTurnMotor
        ) {

        turnAbsoluteEncoder = new CANcoder(turningEncoderCANID);        
        driveMotor = new SparkMax(driveMotorCANID, MotorType.kBrushless);
        turnMotor = new SparkMax(turningMotorCANID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        SparkMaxConfig driveConfig = new SparkMaxConfig();
        SparkMaxConfig turnConfig = new SparkMaxConfig();
        driveConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(40)
                .voltageCompensation(10)
                .inverted(invertDriveMotor);

        turnConfig
                .smartCurrentLimit(40)
                .voltageCompensation(10)
                .inverted(invertTurnMotor);

        driveConfig.encoder
                .positionConversionFactor(Units.inchesToMeters(4 * Math.PI / 6.75))
                .velocityConversionFactor(Units.inchesToMeters(4 * Math.PI / 6.75) / 60);

        turnConfig.encoder
                .positionConversionFactor(150d / 7d * Math.PI / 180 / 1.28)
        // 2/8/2025 Addison testifies this conversion factor will output degrees.
                .velocityConversionFactor(150d / 7d / 60d * Math.PI / 180 / 1.28);

        //this.turnEncoder.setPositionConversionFactor(150d / 7d * Math.PI / 180 / 1.28); // ???
        //this.turnEncoder.setVelocityConversionFactor(150d / 7d / 60d * Math.PI / 180 / 1.28);

        this.driveEncoder.setPosition(0);
        // this.turnEncoder.setPosition(0);

        this.turnAbsoluteEncoder.setPosition(this.turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble());


        // I'm pretty sure violating the strict through-autologged-input guidelines isn't going to cause problems if it's just for setup.+
        this.turnEncoder.setPosition(this.turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble());

        // this.driveMotor.enableVoltageCompensation(10);
        this.drivePIDController = this.driveMotor.getClosedLoopController();

        driveConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pidf(0.3, 0, 0.2, 0.25) // p0.3, d 0.2 ff0.25
                .outputRange(-1, 1);

        // These numbers are recently made up and subject to change.
        driveConfig.closedLoop.maxMotion
                .maxVelocity(3.5)
                .maxAcceleration(4.0)
                // TODO: tune closed loop error constant
                .allowedClosedLoopError(0.1);

        //this.driveMotor.enableVoltageCompensation(10);
        //this.turnMotor.enableVoltageCompensation(10);
        this.turnPIDController = this.turnMotor.getClosedLoopController();

        //this.driveMotor.setSmartCurrentLimit(40);
        //this.turnMotor.setSmartCurrentLimit(40);
        //this.driveMotor.setIdleMode(IdleMode.kCoast);

        turnConfig.closedLoop
                .positionWrappingEnabled(true)
                .positionWrappingMinInput(-Math.PI)
                .positionWrappingMaxInput(Math.PI)
                .pidf(0.55, 0, 0.3, 0) //Do not use ff because it will cause the motors to spin in the wrong direction :)
                .outputRange(-1, 1);// used to be 0.55 0 0.3
        // TODO: add some more config for MAXMOTION

        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setTurnEncoderPosition(double position) {
        this.turnEncoder.setPosition(position);
    }

    @Override
    public void setDriveMotorVoltage(double input) {
        this.driveMotor.setVoltage(input);
    }

    @Override
    public void setTurnMotorVoltage(double input) {
        this.turnMotor.setVoltage(input);
    }

    @Override
    public void setDriveVelocity(double desiredSpeed) {
        this.drivePIDController.setReference(desiredSpeed, ControlType.kMAXMotionVelocityControl);
    }

    @Override
    public void setTurnPosition(Rotation2d desiredAngle) {
        this.turnPIDController.setReference(desiredAngle.getRadians(), ControlType.kPosition);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePositionRad = this.driveEncoder.getPosition();
        inputs.driveVelocityRadPerSec = this.driveEncoder.getVelocity();
        inputs.driveCurrentAmps = this.driveMotor.getOutputCurrent();

        inputs.turnPosition = Rotation2d.fromRotations(this.turnEncoder.getPosition());
        inputs.turnVelocityRadPerSec = this.turnEncoder.getVelocity() / 180 * Math.PI;
        inputs.absoluteEncoderAbsolutePosition = Rotation2d.fromRotations(
            this.turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble());
    }
}
