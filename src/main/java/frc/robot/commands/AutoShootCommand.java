package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterLookupConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
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

    private static final double FLYWHEEL_TOLERANCE_RPM = 100.0;
    private final Timer shootTimer = new Timer();

    public AutoShootCommand(SwerveSubsystem drive, FlywheelSubsystem flywheel, FeederSubsystem feeder) {
        m_drivebase = drive;
        m_flywheel = flywheel;
        m_feeder = feeder;
        addRequirements(flywheel, feeder);
    }

    @Override
    public void initialize() {
        shootTimer.stop();
        shootTimer.reset();
    }

    @Override
    public void execute() {
        // 1. 获取检测到的 AprilTag
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(LIMELIGHT_NAME);
        RawFiducial bestTarget = findNearestTarget(fiducials);

        if (bestTarget == null) {
            // 找不到目标时保持基础怠速，停止所有供弹轮
            m_flywheel.setTargetRPM(1500); 
            m_feeder.idle(); 
            shootTimer.stop();
            shootTimer.reset();
            return;
        }

        // 2. 距离计算与飞轮转速查表
        double horizontalDistance = computeHorizontalDistance(bestTarget.tync);
        double targetFlywheelRpm = 0.0;

        if (horizontalDistance > 0) {
            targetFlywheelRpm = lookupFlywheelRPM(horizontalDistance);
            m_flywheel.setTargetRPM(targetFlywheelRpm);
        }

        // 3. 核心：检查飞轮转速是否达标
        boolean flywheelReady = Math.abs(m_flywheel.getCurrentRPM() - targetFlywheelRpm) <= FLYWHEEL_TOLERANCE_RPM;

        if (flywheelReady) {
            // 【条件满足】启动启动发射计时逻辑
            if (shootTimer.get() == 0.0) {
                shootTimer.start();
            }

            // 步骤 A：小轮（供弹）立刻正转
            m_feeder.shootFeederWheel();

            // 步骤 B：当小轮转过 0.3 秒后，大轮立刻加入
            if (shootTimer.get() >= 0.3) {
                // 这里的 runWheel 对应你原 feeder 里 kShootWheelSpeed 的大轮控制方法
                m_feeder.shootFeeder();
            }
        } else {
            // 【条件不满足】转速掉落或者还在提速，重置计时器并待机
            shootTimer.stop();
            shootTimer.reset();
            m_feeder.idle();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shootTimer.stop();
        shootTimer.reset();
        m_flywheel.stop();
        m_feeder.idle(); // 自动回退到默认慢反转或停止状态
    }

    // ================= 辅助方法 (保持不变) =================

    private RawFiducial findNearestTarget(RawFiducial[] fiducials) {
        if (fiducials == null || fiducials.length == 0) return null;
        RawFiducial nearest = null;
        double minDist = Double.MAX_VALUE;
        for (RawFiducial f : fiducials) {
            boolean isTarget = false;
            for (int id : VisionConstants.TARGET_TAG_IDS) {
                if (f.id == id) { isTarget = true; break; }
            }
            if (!isTarget) continue;
            if (f.distToRobot < minDist) {
                minDist = f.distToRobot;
                nearest = f;
            }
        }
        return nearest;
    }

    private double computeHorizontalDistance(double tync) {
        double angleRad = Math.toRadians(Math.abs(tync));
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