package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Flags;
import frc.robot.util.NetworkTablesUtil;

public class CoralIntakeSubsystem extends SubsystemBase {
    private static final double VERY_HARD_BACK_LIMIT = 0.03; // to make sure we don't roll-over the intake
    private static final double FRONT_LIMIT = 0.55;

    private final SparkMax coralPivotMotor;
    private final RelativeEncoder coralPivotEncoder;
    private final AbsoluteEncoder coralPivotAbsoluteEncoder;
    private final SparkLimitSwitch coralLimitSwitch;

    private final SparkClosedLoopController coralPivotPIDController;

    private static final GenericPublisher pivotAnglePublisher = NetworkTablesUtil.getPublisher("robot", "coralPivotAbsoluteEncoderPosition", NetworkTableType.kDouble);
    //private static final GenericPublisher coralIntakeMotorPublisher = NetworkTablesUtil.getPublisher("robot", "coralIntakeMotorPosition", NetworkTableType.kDouble);

    private final SparkMax coralIntakeMotor;

    public CoralIntakeSubsystem() {
        coralPivotMotor = new SparkMax(Constants.PortConstants.CAN.CORAL_PIVOT_MOTOR_ID, MotorType.kBrushless);
        coralPivotEncoder = coralPivotMotor.getEncoder();
        coralPivotAbsoluteEncoder = coralPivotMotor.getAbsoluteEncoder();
        // throughboreEncoder = new ThroughboreEncoder(Constants.PortConstants.DIO.CORAL_ABSOLUTE_ENCODER_ABS_ID, 0, false);
        coralPivotPIDController = coralPivotMotor.getClosedLoopController();

        SparkMaxConfig coralPivotMotorConfig = new SparkMaxConfig();

        coralPivotMotorConfig
                .inverted(false)
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .voltageCompensation(12);
        coralPivotMotorConfig.absoluteEncoder
                .setSparkMaxDataPortConfig()
                .zeroOffset(0.6);
        coralPivotMotorConfig.encoder
                .positionConversionFactor(1d / 48)
                .velocityConversionFactor(1d / 48 / 60);
        coralPivotMotorConfig.closedLoop
                .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
                .pidf(2.5, 0, 0, 0)
                .outputRange(-0.2, 0.2);
        coralPivotMotorConfig.closedLoop.maxMotion
                .maxVelocity(0.5 * 60)
                .maxAcceleration(0.5 * 60);

        coralPivotEncoder.setPosition(coralPivotAbsoluteEncoder.getPosition());
        coralPivotMotor.configure(coralPivotMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);


        coralIntakeMotor = new SparkMax(Constants.PortConstants.CAN.CORAL_INTAKE_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig coralIntakeMotorConfig = new SparkMaxConfig();
        coralIntakeMotorConfig
                .inverted(true)
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(20)
                .voltageCompensation(12);
        coralIntakeMotorConfig.limitSwitch
                .reverseLimitSwitchEnabled(false)
                .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);
        this.coralLimitSwitch = coralIntakeMotor.getReverseLimitSwitch();
        coralIntakeMotor.configure(coralIntakeMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        // throughboreEncoder.name = "climber";
        // front limit 0.55 rot
        // back limit 0.01 rot
    }

    @Override
    public void periodic() {
        // System.out.println(coralPivotEncoder.getPosition());
        // System.out.println(this.coralPivotAbsoluteEncoder.getPosition());
        // throughboreEncoder.periodic();
        // System.out.println(this.coralLimitSwitch.isPressed());
        pivotAnglePublisher.setDouble(this.coralPivotAbsoluteEncoder.getPosition());
    }

    public boolean hasCoral() {
        return this.coralLimitSwitch.isPressed();
    }

    public Rotation2d getPivotAngle() {
        return Rotation2d.fromRotations(this.coralPivotAbsoluteEncoder.getPosition());
    }

    public void setPivotTargetAngle(Rotation2d target) {
        if (Flags.CoralIntake.ENABLED) {
            double rot = MathUtil.clamp(target.getRotations(), VERY_HARD_BACK_LIMIT, FRONT_LIMIT);
            // System.out.println("i set the target angle to " + rot);
            coralPivotPIDController.setReference(rot, SparkBase.ControlType.kPosition);
        }
    }

    public void setIntakeSpeed(double speed) {
        if (Flags.CoralIntake.ENABLED) {
            coralIntakeMotor.set(speed);
        }
    }

    public void setRawSpeed(double speed) {
        if (Flags.CoralIntake.ENABLED) {
            coralPivotMotor.set(speed);
        }
    }

    public Rotation2d getThroughboreEncoderDistance() {
        return new Rotation2d();// throughboreEncoder.getTotalDistance();
    }
}
