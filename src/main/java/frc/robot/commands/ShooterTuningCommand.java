package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.RawFiducial;
import frc.robot.util.VisionUtil;

/**
 * Shooter tuning command for far-end shooting calibration.
 * - Turret: auto-aims toward hub center using Limelight (parallax-corrected)
 * - Pivot: auto-sets from vision distance lookup; D-pad Up/Down adds manual offset
 * - Flywheel: locked at 6000 RPM, D-pad Left decreases by 300 RPM per press
 * - Shoot: right trigger to fire
 * - Dashboard: publishes predicted horizontal distance to hub center
 *
 * Bind as whileHeld on Start button.
 */
public class ShooterTuningCommand extends Command {

    private static final String LIMELIGHT_NAME = VisionConstants.BACK_LIMELIGHT;

    private static final double FLYWHEEL_DEFAULT_RPM = 6000.0;
    private static final double RPM_STEP = 300.0;
    private static final double PIVOT_DEFAULT_ANGLE = 35.0;
    private static final double PIVOT_STEP = 1.0;
    private static final double TURRET_DEADBAND = 1.5;

    private static final int POV_UP = 0;
    private static final int POV_LEFT = 270;
    private static final int POV_DOWN = 180;

    private final XboxController m_controller;
    private final TurretSubsystem m_turret;
    private final PivotSubsystem m_pivot;
    private final FlywheelSubsystem m_flywheel;
    private final FeederSubsystem m_feeder;

    private double m_currentRPM;
    private double m_currentPivotAngle;
    private double m_pivotOffset = 0;
    private double m_predictedHorizDist = -1;
    private int m_execCount = 0;
    private RawFiducial[] m_cachedFiducials = new RawFiducial[0];
    private int m_lastPOV = -1;

    public ShooterTuningCommand(XboxController controller,
                                 TurretSubsystem turret, PivotSubsystem pivot,
                                 FlywheelSubsystem flywheel, FeederSubsystem feeder) {
        m_controller = controller;
        m_turret = turret;
        m_pivot = pivot;
        m_flywheel = flywheel;
        m_feeder = feeder;
        addRequirements(turret, pivot, flywheel, feeder);
    }

    @Override
    public void initialize() {
        m_currentRPM = FLYWHEEL_DEFAULT_RPM;
        m_currentPivotAngle = PIVOT_DEFAULT_ANGLE;
        m_pivotOffset = 0;
        m_predictedHorizDist = -1;
        m_execCount = 0;
        m_lastPOV = -1;
    }

    @Override
    public void execute() {
        // === 1. Manual adjustments via D-pad (rising-edge) ===
        int pov = m_controller.getPOV();
        if (pov != m_lastPOV) {
            switch (pov) {
                case POV_UP:   m_pivotOffset += PIVOT_STEP; break;
                case POV_DOWN: m_pivotOffset -= PIVOT_STEP; break;
                case POV_LEFT: m_currentRPM = Math.max(0, m_currentRPM - RPM_STEP); break;
            }
        }
        m_lastPOV = pov;

        // === 2. Auto turret aim via hub center (parallax-corrected) ===
        if (++m_execCount >= 3) {
            m_execCount = 0;
            m_cachedFiducials = LimelightHelpers.getRawFiducials(LIMELIGHT_NAME);
        }

        int[] hubTagIds = getHubTagIds();
        Translation2d hubCenter = getHubPosition();
        RawFiducial bestTag = VisionUtil.findBestHubTag(m_cachedFiducials, hubTagIds);
        boolean hubVisible = bestTag != null;

        if (hubVisible) {
            double hubTxnc = VisionUtil.computeHubCenterTxnc(bestTag, hubCenter);
            double currentTurretAngle = m_turret.getCurrentAngle();
            if (Math.abs(hubTxnc) > TURRET_DEADBAND) {
                m_turret.setTargetAngle(currentTurretAngle + VisionConstants.TURRET_KP * hubTxnc);
            }
            // Parallax-corrected horizontal distance to hub center
            m_predictedHorizDist = VisionUtil.computeHubCenterDistance(bestTag, hubCenter);
            if (m_predictedHorizDist > 0) {
                m_currentPivotAngle = VisionUtil.lookupPivotAngle(m_predictedHorizDist) + m_pivotOffset;
            }
        } else {
            m_predictedHorizDist = -1;
        }

        // === 3. Apply flywheel + pivot ===
        m_flywheel.setTargetRPM(m_currentRPM);
        m_pivot.setTargetAngle(m_currentPivotAngle);

        // === 4. Shoot with right trigger ===
        boolean triggerPressed = m_controller.getRightTriggerAxis() > 0.5;
        if (triggerPressed) {
            m_feeder.shootFeederWheel();
            m_feeder.shootFeeder();
        } else {
            m_feeder.idleMod();
        }

        // === 5. Dashboard ===
        SmartDashboard.putNumber("Tuning/TargetRPM", m_currentRPM);
        SmartDashboard.putNumber("Tuning/ActualRPM", m_flywheel.getCurrentRPM());
        SmartDashboard.putNumber("Tuning/PivotAngle", m_currentPivotAngle);
        SmartDashboard.putNumber("Tuning/PivotOffset", m_pivotOffset);
        SmartDashboard.putNumber("Tuning/HorizDist", m_predictedHorizDist);
        SmartDashboard.putBoolean("Tuning/HubVisible", hubVisible);
        SmartDashboard.putBoolean("Tuning/Shooting", triggerPressed);
    }

    @Override
    public void end(boolean interrupted) {
        m_flywheel.stop();
        m_feeder.idleMod();
    }

    private int[] getHubTagIds() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return VisionConstants.RED_HUB_TAGS;
        }
        return VisionConstants.BLUE_HUB_TAGS;
    }

    private Translation2d getHubPosition() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return VisionConstants.HUB_POSITION_RED;
        }
        return VisionConstants.HUB_POSITION_BLUE;
    }
}
