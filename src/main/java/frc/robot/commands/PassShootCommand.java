package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PassShootConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * Pass shoot command: turret counter-rotates to cancel car heading,
 * then after a delay, pivot goes to preset angle and shoots.
 *
 * Turret target = -(car heading), so turret + heading ≈ 0 (always faces original forward).
 * If |car heading| > 90°, turret can't compensate → refuse to shoot.
 *
 * Phases:
 *   Phase 1 (0-0.3s): Turret moves to counter-rotate, flywheel starts spinning
 *   Phase 2 (0.3s+):  Pivot goes to preset angle, feeder starts when flywheel ready
 */
public class PassShootCommand extends Command {

    private final SwerveSubsystem m_drivebase;
    private final TurretSubsystem m_turret;
    private final FlywheelSubsystem m_flywheel;
    private final PivotSubsystem m_pivot;
    private final FeederSubsystem m_feeder;

    private final Timer timer = new Timer();
    private boolean m_isShooting = false;

    public PassShootCommand(SwerveSubsystem drivebase, TurretSubsystem turret,
                            FlywheelSubsystem flywheel, PivotSubsystem pivot, FeederSubsystem feeder) {
        m_drivebase = drivebase;
        m_turret = turret;
        m_flywheel = flywheel;
        m_pivot = pivot;
        m_feeder = feeder;
        addRequirements(turret, flywheel, pivot, feeder);
    }

    @Override
    public void initialize() {
        timer.stop();
        timer.reset();
        m_isShooting = false;
    }

    @Override
    public void execute() {
        // 1. 读取车身朝向
        double carHeading = m_drivebase.getHeading().getDegrees();

        // 2. 计算 turret 目标角度 = -(车身朝向)
        double turretTarget = -carHeading;

        // 3. 检查 turret 限制：|turretTarget| > 90° → 拒绝射击
        if (Math.abs(turretTarget) > PassShootConstants.MAX_TURRET_HEADING_OFFSET) {
            // 角度超出范围，什么都不做
            return;
        }

        // 4. 设置 turret 角度
        m_turret.setTargetAngle(turretTarget);

        // 5. 启动飞轮 (节省时间，一开始就转)
        m_flywheel.setTargetRPM(PassShootConstants.PASS_RPM);

        // 6. 延迟后开始射击序列
        if (!timer.isRunning()) {
            timer.start();
        }

        if (timer.get() >= PassShootConstants.PASS_DELAY_SECONDS) {
            // Phase 2: pivot 到预设角度
            m_pivot.setTargetAngle(PassShootConstants.PASS_PIVOT_ANGLE);

            // 飞轮就绪后启动 feeder
            boolean flywheelReady = Math.abs(m_flywheel.getCurrentRPM() - PassShootConstants.PASS_RPM) <= 200.0;
            if (flywheelReady || m_isShooting) {
                if (!m_isShooting) {
                    m_isShooting = true;
                }
                m_feeder.shootFeederWheel();
                if (timer.get() >= PassShootConstants.PASS_DELAY_SECONDS + 0.3) {
                    m_feeder.shootFeeder();
                }
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();
        m_flywheel.stop();
        m_feeder.idleMod();
        m_pivot.setTargetAngle(0);
        m_turret.setTargetAngle(0);
        m_isShooting = false;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
