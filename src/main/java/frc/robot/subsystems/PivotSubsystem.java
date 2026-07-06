package frc.robot.subsystems;

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

@SuppressWarnings("removal")
public class PivotSubsystem extends SubsystemBase {
    // Note: SPARK MAX does not support CANivore bus selection via constructor in REVLib 2026.0.5
    // If pivot is on CANivore, use Phoenix 6 TalonFX instead or configure via SPARK MAX utility
    private final SparkMax m_pivotMotor = new SparkMax(PivotConstants.PIVOT_ID, MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_pivotMotor.getEncoder();
    private final SparkClosedLoopController m_pidController = m_pivotMotor.getClosedLoopController();
    private final SparkMaxConfig m_config = new SparkMaxConfig();

    public PivotSubsystem() {
        m_config.idleMode(IdleMode.kBrake);
        m_config.smartCurrentLimit(20);

        m_config.encoder.positionConversionFactor(360.0 / PivotConstants.GEAR_RATIO);

        m_config.closedLoop.p(0.04);
        m_config.closedLoop.i(0.0);
        m_config.closedLoop.d(0.0);

        m_config.softLimit.forwardSoftLimit(PivotConstants.MAX_ANGLE_DEG);
        m_config.softLimit.reverseSoftLimit(PivotConstants.MIN_ANGLE_DEG);
        m_config.softLimit.forwardSoftLimitEnabled(true);
        m_config.softLimit.reverseSoftLimitEnabled(true);

        m_pivotMotor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_encoder.setPosition(0.0);
    }

    public void setTargetAngle(double targetDegrees) {
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
