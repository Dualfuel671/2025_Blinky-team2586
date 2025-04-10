package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
//import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class floorIntakeSubsystem extends SubsystemBase {
    private final TalonFX floorIntakeMotor = new TalonFX(12);
    private final TalonFXConfiguration floorIntakeConfig = new TalonFXConfiguration();
    private final DigitalInput floorIntakeLimitSwitch = new DigitalInput(2);

    public floorIntakeSubsystem() {
        floorIntakeConfig.Slot0.kP = 0.1;
        floorIntakeConfig.Slot0.kI = 0.0;
        floorIntakeConfig.Slot0.kD = 0.0;
        floorIntakeMotor.getConfigurator().apply(floorIntakeConfig);
        floorIntakeMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setFloorIntakeSpeed(double speed) {
        floorIntakeMotor.set(speed);
    }

    public void extendFloorIntake() {
        // fix value of max extension
        while (2 < 30) {
            setFloorIntakeSpeed(0.5);
        }
    }

    public void retractFloorIntake() {
        while (floorIntakeMotor.get() > 0) {
            setFloorIntakeSpeed(-0.5);
        }
        setFloorIntakeSpeed(0.0);
    }

    public void stopFloorIntake() {
        floorIntakeMotor.set(0.0);
    }

    public boolean isFloorIntakeLimitSwitchPressed() {
        return !floorIntakeLimitSwitch.get();
    }
}
