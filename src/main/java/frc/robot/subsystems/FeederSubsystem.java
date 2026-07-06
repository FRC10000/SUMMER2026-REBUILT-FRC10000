package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {
  private final TalonFX m_feederMotor = new TalonFX(FeederConstants.kFeederMotorCanId);
  private final TalonFX m_feederWheelMotor = new TalonFX(FeederConstants.kFeederWheelMotorCanId);
  private final TalonFX m_feederWheelFollower = new TalonFX(FeederConstants.kFeederWheelFollowerCanId);

  public FeederSubsystem() {
    TalonFXConfiguration feederConfig = new TalonFXConfiguration();
    feederConfig.CurrentLimits =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(FeederConstants.kFeederMotorCurrentLimit)
            .withStatorCurrentLimitEnable(true);
    m_feederMotor.getConfigurator().apply(feederConfig);

    TalonFXConfiguration wheelConfig = new TalonFXConfiguration();
    wheelConfig.CurrentLimits =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(FeederConstants.kFeederWheelCurrentLimit)
            .withStatorCurrentLimitEnable(true);
    m_feederWheelMotor.getConfigurator().apply(wheelConfig);

    m_feederWheelFollower.setControl(
        new Follower(FeederConstants.kFeederWheelMotorCanId, MotorAlignmentValue.Opposed));
  }

  public void run(double speed) {
    m_feederMotor.set(Math.max(-FeederConstants.kFeederMaxOutput, Math.min(speed, FeederConstants.kFeederMaxOutput)));
  }

  public void runWheel(double speed) {
    m_feederWheelMotor.set(Math.max(-FeederConstants.kFeederWheelMaxOutput, Math.min(speed, FeederConstants.kFeederWheelMaxOutput)));
  }

  public void stop() {
    m_feederMotor.set(0);
    m_feederWheelMotor.set(0);
  }

  @Override
  public void periodic() {
  }
}
