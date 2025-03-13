package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class IntakeAlgae extends Command {
    private Shooter m_shooter;

    public IntakeAlgae(Shooter shooter) {
        m_shooter = shooter;

        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        m_shooter.setShooterSpeed(.4);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return m_shooter.isAlgaePresent();
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.setShooterSpeed(0);
        
    }


}