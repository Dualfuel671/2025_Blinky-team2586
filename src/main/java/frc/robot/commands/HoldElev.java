package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class HoldElev extends Command {
    private ElevatorSubsystem m_elevator;

    public HoldElev(ElevatorSubsystem elevator) {
        m_elevator = elevator;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_elevator.setMotorSpeed(-0.050);
        System.out.println("Executing HoldElev");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.setMotorSpeed(0);
    }
    
}
