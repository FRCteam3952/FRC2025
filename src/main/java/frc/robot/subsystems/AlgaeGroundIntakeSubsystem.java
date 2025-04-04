package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PortConstants;
import frc.robot.Flags;
import frc.robot.util.Util;

public class AlgaeGroundIntakeSubsystem extends SubsystemBase {
    private final SparkMax algaeIntakeMotor;
    private final Servo algaeLeftServo;
    private final Servo algaeRightServo;

    public AlgaeGroundIntakeSubsystem() {
        algaeIntakeMotor = new SparkMax(PortConstants.CAN.ALGAE_GROUND_INTAKE_MOTOR_ID, MotorType.kBrushless);
        // leaving these undefined seems better than adding them with bad values
        algaeLeftServo = new Servo(PortConstants.PWM.ALGAE_LEFT_SERVO_PORT);
        algaeRightServo = new Servo(PortConstants.PWM.ALGAE_RIGHT_SERVO_PORT);
        
        algaeLeftServo.setBoundsMicroseconds(2500, 0, 0, 0, 500);
        algaeRightServo.setBoundsMicroseconds(2500, 0, 0, 0, 500);

        SparkMaxConfig algaeMotorConfig = new SparkMaxConfig();

        algaeMotorConfig
                .inverted(false)
                .idleMode(SparkBaseConfig.IdleMode.kCoast)
                .smartCurrentLimit(20)
                .voltageCompensation(12);
        
        Util.configureSparkMotor(algaeIntakeMotor, algaeMotorConfig);

        flapToValue(0.95, 0.05);
        // algaeIntakeMotor.configure(algaeMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
    }

    public void setIntakeSpeed(double speed) {
        if(Flags.AlgaeGroundIntake.ENABLED) {
            // System.out.println("setting intake to " + speed);
            algaeIntakeMotor.set(speed);
        }
    }

    /**
     * Logically, left + right = 1.0
     * @param left  Value [0,1]
     * @param right Value [0,1]
     */
    public void flapToValue(double left, double right) {
        // System.out.println("left: " + this.leftServo.getAngle() + ", right: " + this.rightServo.getAngle());
        if(Flags.AlgaeGroundIntake.ENABLED) {
            this.algaeLeftServo.set(left);
            this.algaeRightServo.set(right);
        }
    }
}
