package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Candle;

public class SetLeds extends Command {
    private Candle m_candle;
    private Candle.LEDState m_ledState;
    private boolean isFinished = false;

    public SetLeds(Candle candle, Candle.LEDState ledState) {
        m_candle = candle;
        m_ledState = ledState;
        addRequirements(m_candle);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_candle.setLEDSTate(m_ledState);
        isFinished = true;
    }

    @Override
    public boolean isFinished() {

        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
