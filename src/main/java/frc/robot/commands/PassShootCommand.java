package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Preset pass/shoot command. Spins flywheel at a fixed RPM and sets pivot to a preset angle.
 * Used for passing to队友 or quick shots without vision aiming.
 */
public class PassShootCommand extends Command {

    private final TurretSubsystem turret;
    private final FlywheelSubsystem flywheel;
    private final PivotSubsystem pivot;
    private final double rpm;
    private final double pivotAngle;
    private final double turretAngle;

    /**
     * @param turret      Turret subsystem
     * @param flywheel    Flywheel subsystem
     * @param pivot       Pivot subsystem
     * @param rpm         Flywheel speed in RPM
     * @param pivotAngle  Pivot angle in degrees
     * @param turretAngle Turret angle in degrees (0 = centered)
     */
    public PassShootCommand(TurretSubsystem turret, FlywheelSubsystem flywheel,
                             PivotSubsystem pivot, double rpm, double pivotAngle, double turretAngle) {
        this.turret = turret;
        this.flywheel = flywheel;
        this.pivot = pivot;
        this.rpm = rpm;
        this.pivotAngle = pivotAngle;
        this.turretAngle = turretAngle;
        addRequirements(turret, flywheel, pivot);
    }

    @Override
    public void execute() {
        flywheel.setTargetRPM(rpm);
        pivot.setTargetAngle(pivotAngle);
        turret.setTargetAngle(turretAngle);
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stop();
        pivot.setTargetAngle(0);
        turret.setTargetAngle(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
