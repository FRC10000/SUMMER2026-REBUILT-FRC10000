package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.RawFiducial;
import frc.robot.util.VisionUtil;

/**
 * Vision-based turret + pivot alignment.
 * - Reads limelight-back rawfiducials, filters by alliance tag IDs
 * - txnc PID for turret left/right
 * - tync → distance → pivot angle lookup
 * - Flywheel not controlled here
 */
public class AutoAimCommand extends Command {

    private static final String LIMELIGHT_NAME = VisionConstants.BACK_LIMELIGHT;

    private final TurretSubsystem m_turret;
    private final PivotSubsystem m_pivot;

    private int m_execCount = 0;
    private RawFiducial[] m_cachedFiducials = new RawFiducial[0];

    public AutoAimCommand(TurretSubsystem turret, PivotSubsystem pivot) {
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

        // 2. Find best hub tag and get hub center position from JSON
        int[] hubTagIds = getHubTagIds();
        Translation2d hubCenter = getHubPosition();
        RawFiducial bestTag = VisionUtil.findBestHubTag(m_cachedFiducials, hubTagIds);

        if (bestTag == null) {
            return; // no hub tags visible
        }

        // 3. Parallax-corrected hub center aiming (uses tag→hub offset from field layout JSON)
        double hubTxnc = VisionUtil.computeHubCenterTxnc(bestTag, hubCenter);
        double horizontalDistance = VisionUtil.computeHubCenterDistance(bestTag, hubCenter);

        // 4. Turret 左右锁定：PID 闭环把 hub center txnc 归零
        double currentTurretAngle = m_turret.getCurrentAngle();
        double turretCorrection = Math.abs(hubTxnc) > 1.5 ? VisionConstants.TURRET_KP * hubTxnc : 0;
        m_turret.setTargetAngle(currentTurretAngle + turretCorrection);

        // 5. Pivot 上下：hub center 水平距离 → 查表
        SmartDashboard.putNumber("AutoAim/bestTagID", bestTag.id);
        SmartDashboard.putNumber("AutoAim/hubCenterTxnc", hubTxnc);
        SmartDashboard.putNumber("AutoAim/horizDist", horizontalDistance);
        if (horizontalDistance > 0) {
            double pivotAngle = VisionUtil.lookupPivotAngle(horizontalDistance);
            m_pivot.setTargetAngle(pivotAngle);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.setTargetAngle(0.0);
        m_pivot.setTargetAngle(0.0);
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
