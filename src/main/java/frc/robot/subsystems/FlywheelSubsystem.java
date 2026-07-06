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

public class FlywheelSubsystem extends SubsystemBase {

    // Initialize the 4 Kraken X60s
    private final TalonFX m_frontRight = new TalonFX(FlywheelConstants.FRONT_RIGHT_ID);
    private final TalonFX m_backLeft = new TalonFX(FlywheelConstants.BACK_LEFT_ID);
    private final TalonFX m_frontLeft = new TalonFX(FlywheelConstants.FRONT_LEFT_ID);
    private final TalonFX m_backRight = new TalonFX(FlywheelConstants.BACK_RIGHT_ID);

    // The control request we will use to maintain speed using voltage
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0);

    public FlywheelSubsystem() {
        // 1. Create a configuration object
        TalonFXConfiguration config = new TalonFXConfiguration();

        // 2. SAFETY FIRST: Stator Current Limits
        // 4 Krakens spinning up simultaneously will instantly brown out your robot.
        // We limit them to 60 Amps each to protect the battery while keeping spin-up fast.
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = FlywheelConstants.STATOR_CURRENT_LIMIT;

        // 3. PID and Feedforward Configuration (You will tune these later)
        config.Slot0.kP = 0.1; // Proportional gain
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.12; // Feedforward (Crucial for flywheels!)

        // 4. Apply the configuration to all 4 motors
        m_frontRight.getConfigurator().apply(config);
        m_backLeft.getConfigurator().apply(config);
        m_frontLeft.getConfigurator().apply(config);
        m_backRight.getConfigurator().apply(config);

        // 5. Set to Coast Mode (You do NOT want flywheels violently braking)
        m_frontRight.setNeutralMode(NeutralModeValue.Coast);
        m_backLeft.setNeutralMode(NeutralModeValue.Coast);
        m_frontLeft.setNeutralMode(NeutralModeValue.Coast);
        m_backRight.setNeutralMode(NeutralModeValue.Coast);

        // ---------------------------------------------------------
        // 6. MASTER / FOLLOWER SETUP (Based on your mechanical specs)
        // ---------------------------------------------------------
        // Master: frontRight (Clockwise)
        
        // backLeft moves SAME direction as Master (Clockwise) -> opposed = false
        m_backLeft.setControl(new Follower(FlywheelConstants.FRONT_RIGHT_ID, MotorAlignmentValue.Aligned));
        
        // frontLeft & backRight move OPPOSITE of Master -> Opposed
        m_frontLeft.setControl(new Follower(FlywheelConstants.FRONT_RIGHT_ID, MotorAlignmentValue.Opposed));
        m_backRight.setControl(new Follower(FlywheelConstants.FRONT_RIGHT_ID, MotorAlignmentValue.Opposed));
    }

    /**
     * Sets the target speed of the flywheels in RPM.
     * @param targetRPM The target speed (e.g., 3000)
     */
    public void setTargetRPM(double targetRPM) {
        // Phoenix 6 uses RPS (Rotations Per Second), so we divide RPM by 60
        double targetRPS = targetRPM / 60.0;
        m_frontRight.setControl(m_velocityRequest.withVelocity(targetRPS));
    }

    /**
     * Stops the flywheels gracefully.
     */
    public void stop() {
        m_frontRight.setControl(m_velocityRequest.withVelocity(0));
    }

    @Override
    public void periodic() {
        // Send the Master motor's actual speed to Shuffleboard so you can see if it's dropping
        SmartDashboard.putNumber("Shooter/Target RPM", m_velocityRequest.Velocity * 60.0);
        SmartDashboard.putNumber("Shooter/Actual RPM", m_frontRight.getVelocity().getValueAsDouble() * 60.0);
    }
}