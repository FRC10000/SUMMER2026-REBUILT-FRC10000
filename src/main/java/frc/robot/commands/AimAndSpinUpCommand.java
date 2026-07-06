package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.LimelightVisionSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.ShooterLookup;
import java.util.function.Supplier;

/**
 * On-demand targeting command that links Vision, Turret, Pivot, and Flywheel.
 * Hold the trigger to aim and spin up. Release to stow.
 * Also drives the chassis rotation via vision (drive-by-aiming).
 * Claims SwerveSubsystem to suspend the default drive command during aiming.
 */
public class AimAndSpinUpCommand extends Command {

    private final TurretSubsystem turret;
    private final FlywheelSubsystem flywheel;
    private final PivotSubsystem pivot;
    private final LimelightVisionSubsystem vision;
    private final SwerveSubsystem drivebase;
    private final Supplier<ChassisSpeeds> translationSupplier;

    /**
     * @param turret              Turret subsystem
     * @param flywheel            Flywheel subsystem
     * @param pivot               Pivot subsystem
     * @param vision              Limelight vision subsystem
     * @param drivebase           SwerveSubsystem (claimed to suspend default drive)
     * @param translationSupplier Supplier of translation ChassisSpeeds from SwerveInputStream
     */
    public AimAndSpinUpCommand(TurretSubsystem turret, FlywheelSubsystem flywheel,
                                PivotSubsystem pivot, LimelightVisionSubsystem vision,
                                SwerveSubsystem drivebase,
                                Supplier<ChassisSpeeds> translationSupplier) {
        this.turret = turret;
        this.flywheel = flywheel;
        this.pivot = pivot;
        this.vision = vision;
        this.drivebase = drivebase;
        this.translationSupplier = translationSupplier;
        addRequirements(turret, flywheel, pivot);
        if (drivebase != null) {
            addRequirements(drivebase);
        }
    }

    @Override
    public void execute() {
        if (vision != null && vision.hasTarget()) {
            // Aim turret to center the target
            turret.setTargetAngle(-vision.getTargetTx());

            // Lookup shooter params from distance
            double distance = vision.getEstimatedDistance();
            pivot.setTargetAngle(ShooterLookup.getPivotAngle(distance));
            flywheel.setTargetRPM(ShooterLookup.getRPM(distance));

            // Drive-by-aiming
            if (drivebase != null && translationSupplier != null) {
                ChassisSpeeds speeds = translationSupplier.get();
                speeds = new ChassisSpeeds(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    -vision.getTargetTx() * Constants.OperatorConstants.AIM_P_GAIN
                );
                drivebase.getSwerveDrive().driveFieldOriented(speeds);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        turret.setTargetAngle(0);
        pivot.setTargetAngle(0);
        flywheel.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
