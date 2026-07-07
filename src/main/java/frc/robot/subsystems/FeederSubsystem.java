package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FeederConstants;

@SuppressWarnings("removal")
public class FeederSubsystem extends SubsystemBase {
  private final TalonFX m_feederMotor = new TalonFX(FeederConstants.kFeederMotorCanId, "canivore");
  private final TalonFX m_feederWheelMotor = new TalonFX(FeederConstants.kFeederWheelMotorCanId, "canivore");
  private final TalonFX m_feederWheelFollower = new TalonFX(FeederConstants.kFeederWheelFollowerCanId, "canivore");

  public FeederSubsystem() {
    TalonFXConfiguration feederConfig = new TalonFXConfiguration();
    feederConfig.CurrentLimits =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(FeederConstants.kFeederMotorCurrentLimit)
            .withStatorCurrentLimitEnable(true);
    m_feederMotor.getConfigurator().apply(feederConfig);

    // Wheel motor: anti-clockwise = forward
    TalonFXConfiguration wheelConfig = new TalonFXConfiguration();
    wheelConfig.CurrentLimits =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(FeederConstants.kFeederWheelCurrentLimit)
            .withStatorCurrentLimitEnable(true);
    wheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    m_feederWheelMotor.getConfigurator().apply(wheelConfig);

    // Follower: mirrors wheel motor but spins opposite
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

  /** Idle: both motors slowly reverse to prevent ball buildup. */
  public Command idleCommand() {
    return Commands.runEnd(
        () -> {
          run(FeederConstants.kIdleFeederSpeed);
          runWheel(FeederConstants.kIdleWheelSpeed);
        },
        () -> stop(),
        this
    );
  }

  /** Shooting: wheel at 80%, feeder at 60%. */
  public Command shootCommand() {
      return Commands.sequence(
          // 1. 先启动飞轮（runOnce 发送指令后立即进入下一步）
          this.runOnce(() -> runWheel(FeederConstants.kShootWheelSpeed)),
          
          // 2. 等待 0.5 秒（给飞轮留出起步提速的时间）
          new WaitCommand(0.5),
          
          // 3. 保持飞轮转动的同时，启动供弹轮送弹（这个命令会一直维持，直到按键被松开）
          this.run(() -> {
              runWheel(FeederConstants.kShootWheelSpeed); // 保持飞轮速度
              run(FeederConstants.kShootFeederSpeed);    // 启动供弹轮
          })
      )
      // 4. 核心安全机制：无论由于松开按键正常结束，还是被其他命令打断，都会执行 stop
      .finallyDo((interrupted) -> stop()); 
  }

  @Override
  public void periodic() {
  }
}
