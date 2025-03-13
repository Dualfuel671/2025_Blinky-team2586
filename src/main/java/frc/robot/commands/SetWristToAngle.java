package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubSystem;

public class SetWristToAngle extends Command {

    // private final WristSubSystem m_wrist;
    // private double m_angle;
    // private Constraints topConstraints = new Constraints(3000, 1200);
    // private ProfiledPIDController WristPID = new ProfiledPIDController(.012, 0, 0, topConstraints);
    
    // public SetWristToAngle(WristSubSystem wrist, double angle) {
    //     this.m_wrist = wrist;
    //     this.m_angle = angle;
    //     addRequirements(m_wrist);
    // }

    // @Override
    // public void initialize() {
    //     WristPID.reset(m_wrist.getEncoderPosition());
    // }

    // @Override
    // public void execute() {
    //     m_wrist.setPivot(WristPID.calculate(m_wrist.getEncoderPosition(), m_angle));
    // }

    // @Override
    // public boolean isFinished() {
    //     if (Math.abs(m_angle - m_wrist.getEncoderPosition()) < 2 || !m_wrist.isinRange()) {
    //         return true;
    //     } else {
    //         return false;
    //     }
    // }

    // @Override
    // public void end(boolean interrupted) {
    //     m_wrist.setWristPosition(0);
    // }

}
