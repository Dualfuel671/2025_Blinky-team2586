package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RampSubSystem;

public class StartRampWheel extends Command {
    private RampSubSystem m_ramp;

    public StartRampWheel(RampSubSystem ramp) {
        m_ramp = ramp;

        addRequirements(m_ramp);
    }

    @Override
    public void initialize() {
        m_ramp.setRampSpeed(0.35);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}