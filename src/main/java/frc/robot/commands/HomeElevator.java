package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

public class HomeElevator extends Command {
    private ElevatorSubsystem m_elevator;
     
     public HomeElevator(ElevatorSubsystem elevator) {
        m_elevator = elevator;

        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        m_elevator.moveDown();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return m_elevator.elevatorHome();
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.setCurrentPosition(ElevatorPosition.Home);
        m_elevator.setMotorSpeed(0);
    }
}
