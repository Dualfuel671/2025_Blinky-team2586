package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.floorIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.floorIntakeSubsystem.FloorPickupPosition;

public class PIDfloorMovement extends Command {

    private floorIntakeSubsystem m_extensioFloorIntakeSubsystem;
    private FloorPickupPosition targetPosition;

    public PIDfloorMovement(FloorPickupPosition targetPosition, floorIntakeSubsystem m_extensiFloorIntakeSubsystem) {
        this.targetPosition = targetPosition;
        this.m_extensioFloorIntakeSubsystem = m_extensioFloorIntakeSubsystem;

        addRequirements(m_extensioFloorIntakeSubsystem);
    }

    @Override
    public void initialize() {
        m_extensioFloorIntakeSubsystem.resetPID();
    }

    @Override
    public void execute() {
        m_extensioFloorIntakeSubsystem.placeholder(m_extensioFloorIntakeSubsystem.TranslateEnum(targetPosition));
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(m_extensioFloorIntakeSubsystem.getExtensionPosition()
                - m_extensioFloorIntakeSubsystem.TranslateEnum(targetPosition)) < 0.5) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_extensioFloorIntakeSubsystem.setCurrentPosition(FloorPickupPosition.out);
    }

}
