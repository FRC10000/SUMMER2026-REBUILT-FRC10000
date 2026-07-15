package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterLookupConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.RawFiducial;

/**
 * 全自动纯飞轮射击指令 (Auto Shoot)：
 * - 移除了所有 Pivot 和 Turret 角度依赖
 * - tync -> 计算水平距离 -> 查表定 Flywheel 飞轮转速
 * - 当 Flywheel 达到目标值时，先启动小轮，0.3秒后大轮加入完成供弹
 */
public class AutoShootCommand extends Command {

    private static final String LIMELIGHT_NAME = VisionConstants.BACK_LIMELIGHT;

    private final SwerveSubsystem m_drivebase;
    private final FlywheelSubsystem m_flywheel;
    private final FeederSubsystem m_feeder;
    private final IntakeSubsystem m_intake;

    private static final double FLYWHEEL_TOLERANCE_RPM = 200.0;
    private static final double HALF_DEPLOY_FRACTION = 0.5;
    private static final double ROLLER_REVERSE_POWER = -0.2;
    private final Timer shootTimer = new Timer();

    private int m_execCount = 0;
    private RawFiducial[] m_cachedFiducials = new RawFiducial[0];
    private boolean m_isShooting = false;

    public AutoShootCommand(SwerveSubsystem drive, FlywheelSubsystem flywheel, FeederSubsystem feeder,
                            IntakeSubsystem intake) {
        m_drivebase = drive;
        m_flywheel = flywheel;
        m_feeder = feeder;
        m_intake = intake;
        addRequirements(flywheel, feeder, intake);
    }

    @Override
    public void initialize() {
        shootTimer.stop();
        shootTimer.reset();
        m_isShooting = false; // 初始化重置
    }

    @Override
    public void execute() {
        // 1. 获取检测到的 AprilTag (每3周期读一次)
        if (++m_execCount >= 3) {
            m_execCount = 0;
            m_cachedFiducials = LimelightHelpers.getRawFiducials(LIMELIGHT_NAME);
        }
        RawFiducial bestTarget = findNearestTarget(m_cachedFiducials);

        if (bestTarget == null && !m_isShooting) { // 如果没目标且当前不在射击流中，才待机
            m_flywheel.setTargetRPM(1500);
            m_feeder.idleMod();
            shootTimer.stop();
            shootTimer.reset();
            return;
        }

        // 2. 距离计算与飞轮转速查表（如果在射击中，保持上一刻的转速设定，避免查表乱跳）
        double targetFlywheelRpm = 1500;
        if (bestTarget != null) {
            double horizontalDistance = computeHorizontalDistance(bestTarget.tync);
            if (horizontalDistance > 0) {
                targetFlywheelRpm = lookupFlywheelRPM(horizontalDistance);
                m_flywheel.setTargetRPM(targetFlywheelRpm);
            }
        }

        // 3. 核心修改：检查飞轮是否准备就绪，或者已经处于射击序列中
        boolean flywheelReady = Math.abs(m_flywheel.getCurrentRPM() - targetFlywheelRpm) <= FLYWHEEL_TOLERANCE_RPM;

        // 只要准备好了，或者一旦开启了射击流，就无视掉速，一条路走到黑
        if (flywheelReady || m_isShooting) {
            if (!m_isShooting) {
                m_isShooting = true; // 锁定状态，一旦触发绝不回头
                shootTimer.start();
            }

            // 射击 2 秒后：intake 往内伸缩 + 滚轴反转
            if (shootTimer.get() >= 2.0) {
                m_intake.setDeployAngle(IntakeSubsystem.FULL_DEPLOY_DEGREES * HALF_DEPLOY_FRACTION);
                m_intake.setRollerSpeed(ROLLER_REVERSE_POWER);
            }

            // 步骤 A：小轮立刻正转
            m_feeder.shootFeederWheel();

            // 步骤 B：当小轮转过 0.3 秒后，大轮加入
            if (shootTimer.get() >= 0.3) {
                m_feeder.shootFeeder();
            }
        } else {
            // 只有在既没准备好，又没开始射击的情况下，才处于待机状态
            shootTimer.stop();
            shootTimer.reset();
            m_feeder.idleMod();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shootTimer.stop();
        shootTimer.reset();
        m_flywheel.stop();
        m_feeder.idleMod();
        m_intake.setRollerSpeed(0);
        m_intake.setDeployAngle(IntakeSubsystem.FULL_DEPLOY_DEGREES);
        m_isShooting = false;
    }

    // ================= 辅助方法 (保持不变) =================

    private RawFiducial findNearestTarget(RawFiducial[] fiducials) {
        if (fiducials == null || fiducials.length == 0) return null;

        int primaryId = (int) LimelightHelpers.getFiducialID(LIMELIGHT_NAME);

        // 先匹配 Limelight 的 primary 目标
        for (RawFiducial f : fiducials) {
            if (f.id == primaryId && isTargetTag(f.id)) {
                return f;
            }
        }

        // 按距离兜底
        RawFiducial nearest = null;
        double minDist = Double.MAX_VALUE;
        for (RawFiducial f : fiducials) {
            if (!isTargetTag(f.id)) continue;
            if (f.distToRobot < minDist) {
                minDist = f.distToRobot;
                nearest = f;
            }
        }
        return nearest;
    }

    private boolean isTargetTag(int id) {
        for (int tid : VisionConstants.TARGET_TAG_IDS) {
            if (id == tid) return true;
        }
        return false;
    }

    private double computeHorizontalDistance(double tync) {
        double angleFromHoriz = VisionConstants.CAM_MOUNT_ANGLE_DEG + tync;
        double angleRad = Math.toRadians(angleFromHoriz);
        if (angleRad < 0.01) return 5.0;
        return VisionConstants.HEIGHT_DIFF_METERS / Math.tan(angleRad);
    }

    private double lookupFlywheelRPM(double distance) {
        double[] distances = ShooterLookupConstants.DISTANCE_TO_PIVOT_DISTANCE; 
        double[] rpms = ShooterLookupConstants.DISTANCE_TO_RPM_RPM; 

        if (distance <= distances[0]) return rpms[0];
        if (distance >= distances[distances.length - 1]) return rpms[rpms.length - 1];

        for (int i = 0; i < distances.length - 1; i++) {
            if (distance >= distances[i] && distance <= distances[i + 1]) {
                double ratio = (distance - distances[i]) / (distances[i + 1] - distances[i]);
                return rpms[i] + ratio * (rpms[i + 1] - rpms[i]);
            }
        }
        return rpms[0];
    }
}