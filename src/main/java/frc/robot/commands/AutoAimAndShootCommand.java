package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
 * - Turret + pivot vision alignment using Limelight
 * - When no limelight target: odometry fallback to alliance hub angle
 * - When aligned, flywheel spins up and feeder shoots
 * - Alliance-aware tag filtering: Red {10,11} priority 10, Blue {26,27} priority 26
 */
public class AutoAimAndShootCommand extends Command {

    private static final String LIMELIGHT_NAME = VisionConstants.BACK_LIMELIGHT;
    private static final double FLYWHEEL_RPM = 6000.0;
    private static final double SHOOT_RPM_THRESHOLD = 5600.0;

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

        // 2. Find best hub tag and get hub center from JSON
        int[] hubTagIds = getHubTagIds();
        Translation2d hubCenter = getHubPosition();
        RawFiducial bestTag = VisionUtil.findBestHubTag(m_cachedFiducials, hubTagIds);

        // 3. Aim: limelight vision (parallax-corrected) or odometry fallback
        if (bestTag != null) {
            // === Limelight vision — parallax-corrected hub center aiming ===
            double hubTxnc = VisionUtil.computeHubCenterTxnc(bestTag, hubCenter);
            double currentTurretAngle = m_turret.getCurrentAngle();
            double turretCorrection = Math.abs(hubTxnc) > 1.5
                ? VisionConstants.TURRET_KP * hubTxnc : 0;
            m_turret.setTargetAngle(currentTurretAngle + turretCorrection);

            // Parallax-corrected distance using tag→hub offset from field layout JSON
            double horizontalDistance = VisionUtil.computeHubCenterDistance(bestTag, hubCenter);
            SmartDashboard.putNumber("AutoAimShoot/bestTagID", bestTag.id);
            SmartDashboard.putNumber("AutoAimShoot/hubCenterTxnc", hubTxnc);
            SmartDashboard.putNumber("AutoAimShoot/horizDist", horizontalDistance);
            if (horizontalDistance > 0) {
                double pivotAngle = VisionUtil.lookupPivotAngle(horizontalDistance);
                m_pivot.setTargetAngle(pivotAngle);
            }
        } else if (!m_isShooting) {
            // === Odometry fallback — compute angle to alliance hub ===
            Pose2d robotPose = m_drivebase.getPose();
            Translation2d hubPos = getHubPosition();
            double fieldAngle = VisionUtil.computeOdometryAngleToTarget(robotPose, hubPos);
            double robotHeading = m_drivebase.getHeading().getRadians();
            double turretAngle = VisionUtil.computeTurretAngle(fieldAngle, robotHeading);
            m_turret.setTargetAngle(turretAngle);
            return; // no target → don't shoot
        }

        // === Shoot logic ===
        m_flywheel.setTargetRPM(FLYWHEEL_RPM);

        // 飞轮就绪或已在射击序列中
        boolean flywheelReady = m_flywheel.getCurrentRPM() >= SHOOT_RPM_THRESHOLD;

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
        } else if (bestTag != null) {
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
