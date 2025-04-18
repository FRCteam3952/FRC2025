package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Flags;
import frc.robot.util.Util;

public class AlgaeReefRemoverSubsystem extends SubsystemBase {
    private final SparkMax algaeRemoverMotor;

    public AlgaeReefRemoverSubsystem() {
        algaeRemoverMotor = new SparkMax(Constants.PortConstants.CAN.ALGAE_REMOVER_MOTOR_ID, MotorType.kBrushless);

        SparkMaxConfig algaeMotorConfig = new SparkMaxConfig();

        algaeMotorConfig
                .inverted(true)
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(20)
                .voltageCompensation(12);
        
        Util.configureSparkMotor(algaeRemoverMotor, algaeMotorConfig);
        // algaeRemoverMotor.configure(algaeMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
    }

    public void setIntakeSpeed(double speed) {
        if (Flags.AlgaeReefRemover.ENABLED) {
            algaeRemoverMotor.set(speed);
        }
    }
}
