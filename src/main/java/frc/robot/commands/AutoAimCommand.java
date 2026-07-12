package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterLookupConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.RawFiducial;

/**
 * 原始 AprilTag 相机空间自瞄：
 * - 读取 limelight-back 的 rawfiducials，过滤目标 ID，选最近的 tag
 * - txnc PID 闭环锁云台左右
 * - tync → 水平距离 → 查表定 pivot 角度
 * - 飞轮不接入，保持手动控制
 */
public class AutoAimCommand extends Command {

    private static final String LIMELIGHT_NAME = VisionConstants.BACK_LIMELIGHT;

    private final SwerveSubsystem m_drivebase;
    private final TurretSubsystem m_turret;
    private final PivotSubsystem m_pivot;

    private int m_execCount = 0;
    private RawFiducial[] m_cachedFiducials = new RawFiducial[0];

    public AutoAimCommand(SwerveSubsystem drive, TurretSubsystem turret, PivotSubsystem pivot) {
        m_drivebase = drive;
        m_turret = turret;
        m_pivot = pivot;
        addRequirements(turret, pivot);
    }

    @Override
    public void execute() {
        // 1. 获取所有检测到的 AprilTag (每3周期读一次，约60ms)
        if (++m_execCount >= 3) {
            m_execCount = 0;
            m_cachedFiducials = LimelightHelpers.getRawFiducials(LIMELIGHT_NAME);
        }

        // 2. 过滤目标 ID，选最近的
        RawFiducial bestTarget = findNearestTarget(m_cachedFiducials);
        if (bestTarget == null) {
            // 没看到目标 tag，保持当前位置不动
            return;
        }

        // 3. Turret 左右锁定：PID 闭环把 txnc 归零
        //    txnc > 0 → 目标在画面右侧 → 需要云台向右转（减小当前角度）
        //    具体正负号需要根据实测确认，先用 +txnc，如果反了改 -txnc
        double currentTurretAngle = m_turret.getCurrentAngle();
        double turretCorrection = (bestTarget.txnc < 1.5) ? VisionConstants.TURRET_KP * bestTarget.txnc : 0;
        m_turret.setTargetAngle(currentTurretAngle + turretCorrection);

        // 4. Pivot 上下：tync → 水平距离 → 查表
        double horizontalDistance = computeHorizontalDistance(bestTarget.tync);
        if (horizontalDistance > 0) {
            double pivotAngle = lookupPivotAngle(horizontalDistance);
            m_pivot.setTargetAngle(pivotAngle);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // 松手后保持当前位置，不归零
    }

    /**
     * 从 rawfiducials 中找到目标 ID 里离机器人最近的那个
     */
    private RawFiducial findNearestTarget(RawFiducial[] fiducials) {
        if (fiducials == null || fiducials.length == 0) {
            return null;
        }

        RawFiducial nearest = null;
        double minDist = Double.MAX_VALUE;

        for (RawFiducial f : fiducials) {
            // 检查 ID 是否在目标列表中
            boolean isTarget = false;
            for (int id : VisionConstants.TARGET_TAG_IDS) {
                if (f.id == id) {
                    isTarget = true;
                    break;
                }
            }
            if (!isTarget) continue;

            // 选最近的
            if (f.distToRobot < minDist) {
                minDist = f.distToRobot;
                nearest = f;
            }
        }

        return nearest;
    }

    /**
     * 通过 tync 和已知高度差计算水平距离
     * tync 是相机坐标系下的垂直偏角（度），tag 在视线上方时为负值
     * distance = heightDiff / tan(|tync| 的弧度)
     */
    private double computeHorizontalDistance(double tync) {
        // tync 通常为负值（tag 在相机视线上方），取绝对值
        double angleRad = Math.toRadians(Math.abs(tync));
        if (angleRad < 0.01) {
            // 角度太小，距离趋于无穷，返回一个安全的大值
            return 5.0;
        }
        return VisionConstants.HEIGHT_DIFF_METERS / Math.tan(angleRad);
    }

    /**
     * 线性插值查表：距离 → pivot 角度
     */
    private double lookupPivotAngle(double distance) {
        double[] distances = ShooterLookupConstants.DISTANCE_TO_PIVOT_DISTANCE;
        double[] angles = ShooterLookupConstants.DISTANCE_TO_PIVOT_ANGLE;

        // 距离小于最小值 → 返回最小角度
        if (distance <= distances[0]) {
            return angles[0];
        }
        // 距离大于最大值 → 返回最大角度
        if (distance >= distances[distances.length - 1]) {
            return angles[angles.length - 1];
        }

        // 线性插值
        for (int i = 0; i < distances.length - 1; i++) {
            if (distance >= distances[i] && distance <= distances[i + 1]) {
                double ratio = (distance - distances[i]) / (distances[i + 1] - distances[i]);
                return angles[i] + ratio * (angles[i + 1] - angles[i]);
            }
        }

        return angles[0]; // fallback
    }
}
