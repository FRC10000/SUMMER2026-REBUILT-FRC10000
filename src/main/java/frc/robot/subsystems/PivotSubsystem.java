package frc.robot.subsystems;

// Update these to the new REVLib API structures
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class PivotSubsystem extends SubsystemBase {
    // 1. Instantiated using SparkMax instead of CANSparkMax
    private final SparkMax m_pivotMotor = new SparkMax(PivotConstants.PIVOT_ID, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_pivotMotor.getEncoder();
    private final SparkClosedLoopController m_pidController = m_pivotMotor.getClosedLoopController();
    private final SparkMaxConfig m_config = new SparkMaxConfig();

    public PivotSubsystem() {
        // 2. Build the configuration object cleanly
        m_config.idleMode(IdleMode.kBrake);
        m_config.smartCurrentLimit(20); // Protect NEO 550 from stalling/burning out

        // Mapped conversion factor: Degrees per motor rotation
        m_config.encoder.positionConversionFactor(360.0 / PivotConstants.GEAR_RATIO);

        // Position Closed Loop PID
        m_config.closedLoop.p(0.02);
        m_config.closedLoop.i(0.0);
        m_config.closedLoop.d(0.0);

        // Soft Limits setup
        m_config.softLimit.forwardSoftLimit(PivotConstants.MAX_ANGLE_DEG);
        m_config.softLimit.reverseSoftLimit(PivotConstants.MIN_ANGLE_DEG);
        m_config.softLimit.forwardSoftLimitEnabled(true);
        m_config.softLimit.reverseSoftLimitEnabled(true);

        // 3. Push the single monolithic configuration block to the hardware controller
        m_pivotMotor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Zero encoder assuming manual starting placement on boot
        m_encoder.setPosition(0.0);
    }

    public void setTargetAngle(double targetDegrees) {
        // ControlType is handled by ControlType enum or method names in new REVLib
        m_pidController.setReference(targetDegrees, SparkMax.ControlType.kPosition);
    }

    public double getCurrentAngle() {
        return m_encoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot/Current Angle", getCurrentAngle());
    }
}