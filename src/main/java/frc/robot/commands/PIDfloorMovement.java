package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.floorIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.floorIntakeSubsystem.FloorPickupPosition;

public class PIDfloorMovement extends Command {

    private floorIntakeSubsystem m_extensionFloorIntakeSubsystem;
    private FloorPickupPosition targetPosition;

    public PIDfloorMovement(FloorPickupPosition targetPosition, floorIntakeSubsystem m_extensiFloorIntakeSubsystem) {
        this.targetPosition = targetPosition;
        this.m_extensionFloorIntakeSubsystem = m_extensiFloorIntakeSubsystem;

        addRequirements(m_extensionFloorIntakeSubsystem);
    }

    @Override
    public void initialize() {
        m_extensionFloorIntakeSubsystem.resetPID();
    }

    @Override
    public void execute() {
        m_extensionFloorIntakeSubsystem.placeholder(m_extensionFloorIntakeSubsystem.TranslateEnum(targetPosition));
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(m_extensionFloorIntakeSubsystem.TranslateEnum(targetPosition)
                - m_extensionFloorIntakeSubsystem.getCurrentPosition()) < 0.1) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_extensionFloorIntakeSubsystem.setCurrentPosition(FloorPickupPosition.out);
        m_extensionFloorIntakeSubsystem.setMotorSpeed(0);
    }

}
