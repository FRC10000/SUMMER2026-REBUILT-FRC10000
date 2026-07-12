package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@SuppressWarnings("removal")
public class FlywheelSubsystem extends SubsystemBase {

    private final TalonFX m_frontRight = new TalonFX(FlywheelConstants.FRONT_RIGHT_ID);
    private final TalonFX m_backLeft = new TalonFX(FlywheelConstants.BACK_LEFT_ID);
    private final TalonFX m_frontLeft = new TalonFX(FlywheelConstants.FRONT_LEFT_ID);
    private final TalonFX m_backRight = new TalonFX(FlywheelConstants.BACK_RIGHT_ID);

    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0);

    private int m_periodicCount = 0;
    private static final int SMARTDASHBOARD_INTERVAL = 5; // every 5 cycles = 100ms

    public FlywheelSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = FlywheelConstants.STATOR_CURRENT_LIMIT;

        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.12;

        m_frontRight.getConfigurator().apply(config);
        m_backLeft.getConfigurator().apply(config);
        m_frontLeft.getConfigurator().apply(config);
        m_backRight.getConfigurator().apply(config);

        m_frontRight.setNeutralMode(NeutralModeValue.Coast);
        m_backLeft.setNeutralMode(NeutralModeValue.Coast);
        m_frontLeft.setNeutralMode(NeutralModeValue.Coast);
        m_backRight.setNeutralMode(NeutralModeValue.Coast);

        m_backLeft.setControl(new Follower(FlywheelConstants.FRONT_RIGHT_ID, MotorAlignmentValue.Aligned));
        m_frontLeft.setControl(new Follower(FlywheelConstants.FRONT_RIGHT_ID, MotorAlignmentValue.Opposed));
        m_backRight.setControl(new Follower(FlywheelConstants.FRONT_RIGHT_ID, MotorAlignmentValue.Opposed));
    }

    public void setTargetRPM(double targetRPM) {
        double targetRPS = targetRPM / 60.0;
        m_frontRight.setControl(m_velocityRequest.withVelocity(targetRPS));
    }
    
    public double getCurrentRPM() {
        // 1. 获取电机当前速度 (RPS - 每秒圈数)
        // 注意：如果你使用了内部减速比配置 (SensorToMechanismRatio)，
        // getVelocity() 返回的直接就是机构输出轴的 RPS。
        double currentRPS = m_frontRight.getVelocity().getValueAsDouble();
        
        // 2. 将 RPS 转换为 RPM (乘以 60)
        return currentRPS * 60.0; 
    }

    /**
     * (可选) 检查飞轮是否已经达到目标转速
     */
    public boolean isAtTargetRPM(double targetRPM, double toleranceRPM) {
        return Math.abs(getCurrentRPM() - targetRPM) <= toleranceRPM;
    }

    public void stop() {
        m_frontRight.setControl(m_velocityRequest.withVelocity(0));
    }

    @Override
    public void periodic() {
        if (++m_periodicCount >= SMARTDASHBOARD_INTERVAL) {
            m_periodicCount = 0;
            SmartDashboard.putNumber("Shooter/Target RPM", m_velocityRequest.Velocity * 60.0);
            SmartDashboard.putNumber("Shooter/Actual RPM", m_frontRight.getVelocity().getValueAsDouble() * 60.0);
        }
    }
}
