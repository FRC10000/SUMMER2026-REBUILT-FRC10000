package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.RawFiducial;
import frc.robot.util.VisionUtil;

/**
 * Combined AutoAim + AutoShoot command:
 * - Turret + pivot vision alignment (from AutoAim)
 * - When aligned, flywheel spins up and feeder shoots (from AutoShoot)
 * - Alliance-aware tag filtering: Red {10,11} priority 10, Blue {26,27} priority 26
 * - If no target visible, stays still (does nothing)
 */
public class AutoAimAndShootCommand extends Command {

    private static final String LIMELIGHT_NAME = VisionConstants.BACK_LIMELIGHT;
    private static final double FLYWHEEL_TOLERANCE_RPM = 200.0;

    private final SwerveSubsystem m_drivebase;
    private final TurretSubsystem m_turret;
    private final PivotSubsystem m_pivot;
    private final FlywheelSubsystem m_flywheel;
    private final FeederSubsystem m_feeder;

    private int m_execCount = 0;
    private RawFiducial[] m_cachedFiducials = new RawFiducial[0];
    private boolean m_isShooting = false;
    private final Timer shootTimer = new Timer();

    public AutoAimAndShootCommand(SwerveSubsystem drive, TurretSubsystem turret, PivotSubsystem pivot,
                                   FlywheelSubsystem flywheel, FeederSubsystem feeder) {
        m_drivebase = drive;
        m_turret = turret;
        m_pivot = pivot;
        m_flywheel = flywheel;
        m_feeder = feeder;
        addRequirements(turret, pivot, flywheel, feeder);
    }

    @Override
    public void initialize() {
        shootTimer.stop();
        shootTimer.reset();
        m_isShooting = false;
    }

    @Override
    public void execute() {
        // 1. 读取 limelight fiducials (每3周期)
        if (++m_execCount >= 3) {
            m_execCount = 0;
            m_cachedFiducials = LimelightHelpers.getRawFiducials(LIMELIGHT_NAME);
        }

        // 2. 根据联盟选择目标 tag IDs
        int[] targetIds = getTargetTagIds();

        // 3. 找最佳目标
        RawFiducial bestTarget = VisionUtil.findNearestTarget(m_cachedFiducials, LIMELIGHT_NAME, targetIds);

        // 4. 没目标且不在射击中 → 什么都不做 (原地不动)
        if (bestTarget == null && !m_isShooting) {
            return;
        }

        double targetFlywheelRpm = 1500;

        if (bestTarget != null) {
            // === 瞄准逻辑 (from AutoAim) ===

            // Turret 左右锁定
            double currentTurretAngle = m_turret.getCurrentAngle();
            double turretCorrection = Math.abs(bestTarget.txnc) > 1.5
                ? VisionConstants.TURRET_KP * bestTarget.txnc : 0;
            m_turret.setTargetAngle(currentTurretAngle + turretCorrection);

            // Pivot 上下
            double horizontalDistance = VisionUtil.computeHorizontalDistance(bestTarget.tync);
            SmartDashboard.putNumber("AutoAimShoot/TagID", bestTarget.id);
            SmartDashboard.putNumber("AutoAimShoot/horizDist", horizontalDistance);
            SmartDashboard.putNumber("AutoAimShoot/txnc", bestTarget.txnc);
            if (horizontalDistance > 0) {
                double pivotAngle = VisionUtil.lookupPivotAngle(horizontalDistance);
                m_pivot.setTargetAngle(pivotAngle);
                targetFlywheelRpm = VisionUtil.lookupFlywheelRPM(horizontalDistance);
            }

            // === 射击逻辑 (from AutoShoot) ===
            m_flywheel.setTargetRPM(targetFlywheelRpm);
        }

        // 飞轮就绪或已在射击序列中
        boolean flywheelReady = Math.abs(m_flywheel.getCurrentRPM() - targetFlywheelRpm) <= FLYWHEEL_TOLERANCE_RPM;

        if (flywheelReady || m_isShooting) {
            if (!m_isShooting) {
                m_isShooting = true;
                shootTimer.start();
            }

            // 小轮立刻正转
            m_feeder.shootFeederWheel();

            // 0.3s 后大轮加入
            if (shootTimer.get() >= 0.3) {
                m_feeder.shootFeeder();
            }
        } else if (bestTarget != null) {
            // 有目标但飞轮还没到，feeder 保持 idle
            m_feeder.idleMod();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shootTimer.stop();
        shootTimer.reset();
        m_flywheel.stop();
        m_feeder.idleMod();
        m_turret.setTargetAngle(0.0);
        m_pivot.setTargetAngle(0.0);
        m_isShooting = false;
    }

    private int[] getTargetTagIds() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return VisionConstants.RED_TAG_IDS;
        }
        return VisionConstants.BLUE_TAG_IDS;
    }
}
