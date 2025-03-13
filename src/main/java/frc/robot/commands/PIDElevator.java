package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

public class PIDElevator extends Command {

    private ElevatorSubsystem m_elevator;
    private ElevatorPosition targetPosition;

    public PIDElevator(ElevatorPosition targetPosition, ElevatorSubsystem m_elevator) {
        this.targetPosition = targetPosition;
        this.m_elevator = m_elevator;

        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        m_elevator.resetPID();
    }

    @Override
    public void execute() {
        m_elevator.placeholder(m_elevator.TranslateEnum(targetPosition));
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(m_elevator.getEncoder1Position() - m_elevator.TranslateEnum(targetPosition)) < 0.5) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.setMotorSpeed(-0.0350);
    }

}
