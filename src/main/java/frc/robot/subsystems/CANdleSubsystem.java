package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANdleSubsystem extends SubsystemBase {
    CANdle m_candle = new CANdle(0);
    public LEDState ledstate;

    public void Candle() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = .7;

        m_candle.configAllSettings(configAll, 100);
        m_candle.clearAnimation(0);
        ledstate = LEDState.GREEN;
        setLEDSTate(ledstate);
    }

    public void setLEDSTate(LEDState state) {
        ledstate = state;

        m_candle.setLEDs(state.r, state.g, state.b);
    }

    public LEDState getLEDState() {

        return ledstate;

    }

    public enum LEDState {

        ORANGE(255, 128, 0),
        GREEN(0, 128, 0),
        PURPLE(128, 0, 128),
        YELLOW(255, 128, 0),
        RED(255, 0, 0),
        BLACK(0, 0, 0),
        WHITE(255, 255, 255),
        COPPER(184, 115, 51),
        PINK(255, 50, 193),
        BLUE(0, 0, 225)

        ;

        public final int r;
        public final int g;
        public final int b;

        private LEDState(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("LED_R", getLEDState().r);
        // SmartDashboard.putNumber("LED_G", getLEDState().g);
        // SmartDashboard.putNumber("LED_B", getLEDState().b);
        // SmartDashboard.putNumber("CANdle TEMP", m_candle.getTemperature());

    }
}
