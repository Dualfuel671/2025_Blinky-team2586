package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.PhotonCam;

public class SetLeds extends Command {
    PhotonCam m_cam;
    CANdleSubsystem m_candle;
    double april_tag1_yaw_target = -23;//TODO: determine the correct values
    double april_tag3_yaw_target = 25;//TODO: determine the correct values
    double yaw_tolerance = 5;//TODO: tune the tolerance
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
        var yaw1 = m_cam.getCamera1Yaw();
        var yaw3 = m_cam.getCamera3Yaw();

        //handle april tag1
        if(Math.abs(yaw1 - april_tag1_yaw_target) < yaw_tolerance){
            m_candle.setLEDSTate(CANdleSubsystem.LEDState.GREEN);
        } else {
            m_candle.setLEDSTate(CANdleSubsystem.LEDState.BLACK);
        }

        //handle april tag3
        if(Math.abs(yaw3 - april_tag3_yaw_target) < yaw_tolerance){
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
