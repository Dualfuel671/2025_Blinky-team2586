package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;


public class ShootCoral extends Command {
    private Shooter m_shooter;

    public ShootCoral(Shooter shooter) {
        m_shooter = shooter;

        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        m_shooter.setShooterSpeed(.8);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return !m_shooter.isCoralPresent();
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stopShooter();
    }
}
