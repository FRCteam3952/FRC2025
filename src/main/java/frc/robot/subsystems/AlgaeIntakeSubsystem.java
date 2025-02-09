package frc.robot.subsystems;


import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants;
import frc.robot.Constants.PortConstants;

public class AlgaeIntakeSubsystem extends SubsystemBase{ // This subsystem needing much more feature only has 1 neo currently
    // public class 

    private final SparkMax algaeIntakeMotor;
    private final Servo    algaeLeftServo;
    private final Servo    algaeRightServo;

    public AlgaeIntakeSubsystem(){
        algaeIntakeMotor = new SparkMax(Constants.PortConstants.CAN.ALGAE_MOTOR_ID, MotorType.kBrushless);
        // leaving these undefined seems better than adding them with bad values
        if (PortConstants.PWM.ALGAE_LEFT_SERVO_PORT != -1 || PortConstants.PWM.ALGAE_RIGHT_SERVO_PORT != -1) {
            throw new UnsupportedOperationException("You need to set the port for the AlgaeIntakeSubsystem.");
        }
        algaeLeftServo = new Servo(PortConstants.PWM.ALGAE_LEFT_SERVO_PORT);
        algaeRightServo = new Servo(PortConstants.PWM.ALGAE_RIGHT_SERVO_PORT);
        
        SparkMaxConfig algaeMotorConfig = new SparkMaxConfig();

        algaeMotorConfig
                .inverted(false)
                .idleMode(SparkBaseConfig.IdleMode.kCoast)
                .voltageCompensation(12);
//        algaeMotorConfig.closedLoop
//                .pidf(1, 0, 0, 0)
//                .outputRange(-0.6, 0.6);

        algaeIntakeMotor.configure(algaeMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
    }

    public void setRawSpeed(double speed) {
        algaeIntakeMotor.set(speed);
    }

    public void flapToAngle(double degreesL, double degreesR) {
        // System.out.println("left: " + this.leftServo.getAngle() + ", right: " + this.rightServo.getAngle());
        this.algaeLeftServo.setAngle(degreesL);
        this.algaeRightServo.setAngle(degreesR);
    }
}
