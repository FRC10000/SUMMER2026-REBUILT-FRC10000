package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Intake retract + reverse roller command.
 * While held: retracts intake to 50% deploy, spins roller at -0.2.
 * On release: stops roller, resets intake to full deploy.
 */
public class IntakeRetractCommand extends Command {

    private static final double RETRACT_FRACTION = 0.5;
    private static final double ROLLER_REVERSE_POWER = -0.2;

    private final IntakeSubsystem m_intake;

    public IntakeRetractCommand(IntakeSubsystem intake) {
        m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        m_intake.setDeployAngle(IntakeSubsystem.FULL_DEPLOY_DEGREES * RETRACT_FRACTION);
        m_intake.setRollerSpeed(ROLLER_REVERSE_POWER);
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.setRollerSpeed(0);
        m_intake.setDeployAngle(IntakeSubsystem.FULL_DEPLOY_DEGREES);
    }
}
