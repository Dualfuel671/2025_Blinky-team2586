package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubSystem;
import frc.robot.subsystems.WristSubSystem.WristPosition;

public class PIDWrist extends Command {

    private WristSubSystem m_wrist;
    private WristPosition targetPosition;

    public PIDWrist(WristPosition targetPosition, WristSubSystem m_wrist) {
        this.targetPosition = targetPosition;
        this.m_wrist = m_wrist;

        addRequirements(m_wrist);
    }

    @Override
    public void initialize() {
        m_wrist.resetPID();
    }

    @Override
    public void execute() {
        m_wrist.setWristGoal(m_wrist.TranslateEnum(targetPosition));
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(m_wrist.getWristEncoderPosition() - m_wrist.TranslateEnum(targetPosition)) < 0.03) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_wrist.setMotorSpeed(0);
    }

}