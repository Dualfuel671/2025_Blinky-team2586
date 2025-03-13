package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.RampSubSystem;

public class AdvToShooter extends Command {
    private Shooter m_shooter;

    public AdvToShooter(Shooter shooter) {
        m_shooter = shooter;

        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_shooter.setShooterSpeed(.2);
    }

    @Override
    public boolean isFinished() {
        return m_shooter.isCoralPresent();
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stopShooter();
    }


}