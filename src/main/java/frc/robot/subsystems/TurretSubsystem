package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase {
    
    private final TalonFX m_turretMotor = new TalonFX(TurretConstants.TURRET_ID);
    
    // Motion Magic request for smooth, profiled positional targeting
    private final MotionMagicVoltage m_positionRequest = new MotionMagicVoltage(0).withSlot(0);

    public TurretSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // 1. Tell the Kraken about the 400:14 gear ratio
        // Now, 1 "Position" request = 1 full rotation of the turret, not the motor.
        config.Feedback.SensorToMechanismRatio = TurretConstants.GEAR_RATIO;

        // 2. Hardware Soft Limits (CRITICAL: Protects your Limelight wires!)
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TurretConstants.MAX_ROTATIONS; // +90 degrees
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TurretConstants.MIN_ROTATIONS; // -90 degrees

        // 3. PID and Motion Magic Configuration
        config.Slot0.kP = 4.0; // Proportional gain
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.1;
        
        // Motion Magic constraints (Rotations per second)
        config.MotionMagic.MotionMagicCruiseVelocity = 1.0; // Max speed (1 turret rot/sec)
        config.MotionMagic.MotionMagicAcceleration = 2.0;   // Acceleration
        config.MotionMagic.MotionMagicJerk = 10.0;          // Smooth out the starts/stops

        // Apply config
        m_turretMotor.getConfigurator().apply(config);
        
        // Use Brake mode to hold heading firmly
        m_turretMotor.setNeutralMode(NeutralModeValue.Brake);

        // ZERO THE ENCODER (Assuming manual alignment straight forward on boot)
        m_turretMotor.setPosition(0.0);
    }

    /**
     * Aims the turret.
     * @param targetDegrees Angle to aim (-90 to +90)
     */
    public void setTargetAngle(double targetDegrees) {
        // Convert degrees to rotations for Phoenix 6
        double targetRotations = targetDegrees / 360.0;
        m_turretMotor.setControl(m_positionRequest.withPosition(targetRotations));
    }

    public double getCurrentAngle() {
        // Convert Phoenix 6 rotations back to degrees for easy reading
        return m_turretMotor.getPosition().getValueAsDouble() * 360.0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret/Current Angle", getCurrentAngle());
    }
}