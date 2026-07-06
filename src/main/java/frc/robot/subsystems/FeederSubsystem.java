package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {
  private final TalonFX m_feederMotor = new TalonFX(FeederConstants.kFeederMotorCanId);
  private final TalonFX m_feederWheelMotor = new TalonFX(FeederConstants.kFeederWheelMotorCanId);

  public FeederSubsystem() {
    // Feeder (pad) motor config
    TalonFXConfiguration feederConfig = new TalonFXConfiguration();
    feederConfig.CurrentLimits =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(FeederConstants.kFeederMotorCurrentLimit)
            .withStatorCurrentLimitEnable(true);
    feederConfig.MotorOutput.Inverted = FeederConstants.kFeederMotorInverted
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    m_feederMotor.getConfigurator().apply(feederConfig);

    // Feeder wheel motor config (X44, opposite rotation)
    TalonFXConfiguration wheelConfig = new TalonFXConfiguration();
    wheelConfig.CurrentLimits =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(FeederConstants.kFeederWheelCurrentLimit)
            .withStatorCurrentLimitEnable(true);
    wheelConfig.MotorOutput.Inverted = FeederConstants.kFeederWheelMotorInverted
        ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    m_feederWheelMotor.getConfigurator().apply(wheelConfig);
  }

  /**
   * Runs the feeder pad motor at the given speed.
   *
   * @param speed Percent output from -1.0 to 1.0
   */
  public void run(double speed) {
    m_feederMotor.set(Math.max(-FeederConstants.kFeederMaxOutput, Math.min(speed, FeederConstants.kFeederMaxOutput)));
  }

  /**
   * Runs the feeder wheel motor at the given speed.
   *
   * @param speed Percent output from -1.0 to 1.0
   */
  public void runWheel(double speed) {
    m_feederWheelMotor.set(Math.max(-FeederConstants.kFeederWheelMaxOutput, Math.min(speed, FeederConstants.kFeederWheelMaxOutput)));
  }

  /** Stops both feeder motors. */
  public void stop() {
    m_feederMotor.set(0);
    m_feederWheelMotor.set(0);
  }

  @Override
  public void periodic() {
  }
}
