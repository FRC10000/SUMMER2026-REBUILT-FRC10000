package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

@SuppressWarnings("removal")
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

        config.Slot0.kP = 4.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.1;
        
        // Smoother motion profile
        config.MotionMagic.MotionMagicCruiseVelocity = 0.5;
        config.MotionMagic.MotionMagicAcceleration = 0.5;
        config.MotionMagic.MotionMagicJerk = 5.0;

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
