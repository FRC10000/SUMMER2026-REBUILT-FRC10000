package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
    
    private final TalonFX m_turretMotor = new TalonFX(TurretConstants.TURRET_ID, "canivore");
    
    private final MotionMagicVoltage m_positionRequest = new MotionMagicVoltage(0).withSlot(0);

    public TurretSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Feedback.SensorToMechanismRatio = TurretConstants.GEAR_RATIO;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.MAX_ROTATIONS;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.MIN_ROTATIONS;

        // 【修改1：大幅提高 kP，并加入 kS 和 kD】
        // 对于 Voltage 控制，kP 推荐在 40.0 到 100.0 之间起步测试
        config.Slot0.kP = 60.0; // 每偏差 1 圈，输出 60V (由于被电瓶 12V 截断，相当于偏差 0.2 圈就能给满 12V)
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 5.0;  // 增加 kD 防止目标位置震荡
        
        // kS 是克服静态摩擦力的基础电压。如果你的炮塔很重，可以给 0.2V ~ 0.5V，它会在电机启动瞬间推一把。
        config.Slot0.kS = 0.25; 
        
        // kV 是速度前馈，Motion Magic 必备。你可以先给个基础值，或后续用 SysId 测算
        config.Slot0.kV = 2.0;

        // 【修改2：加快你的 Motion Magic 加速度】
        // 之前 0.5 的加速度意味着要花整整 1 秒钟才能达到 180度/秒 的速度，太肉了。
        config.MotionMagic.MotionMagicCruiseVelocity = 1.0; // 巡航速度：每秒 1 圈 (360度/秒)
        config.MotionMagic.MotionMagicAcceleration = 2.0;   // 加速度：每秒 2 圈 (0.5秒达到最高速)
        config.MotionMagic.MotionMagicJerk = 0.0;           // 先把 Jerk 设为 0 测试，调顺了再加

        m_turretMotor.getConfigurator().apply(config);
        
        m_turretMotor.setNeutralMode(NeutralModeValue.Brake);
        m_turretMotor.setPosition(0.0);
    }

    public void setTargetAngle(double targetDegrees) {
        double targetRotations = targetDegrees / 360.0;
        m_turretMotor.setControl(m_positionRequest.withPosition(targetRotations));
    }

    public double getCurrentAngle() {
        return m_turretMotor.getPosition().getValueAsDouble() * 360.0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret/Current Angle", getCurrentAngle());
    }
}
