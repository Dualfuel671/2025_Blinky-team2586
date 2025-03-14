package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.PhotonCam;

public class SetLeds extends Command {
    PhotonCam m_cam;
    CANdleSubsystem m_candle;
    double yaw_target = 30;//TODO: measure in degrees
    double yaw_tolerance = 1;//TODO: tune the tolerance
    public SetLeds(PhotonCam cam, CANdleSubsystem candle) {
        m_cam = cam;
        m_candle = candle;
        addRequirements(m_cam, m_candle);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        var yaw = m_cam.getCameraYaw();
        if(Math.abs(yaw - yaw_target) < yaw_tolerance){
            m_candle.setLEDSTate(CANdleSubsystem.LEDState.GREEN);
        } else {
            m_candle.setLEDSTate(CANdleSubsystem.LEDState.BLACK);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
