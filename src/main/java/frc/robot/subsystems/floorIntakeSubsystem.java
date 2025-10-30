package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class floorIntakeSubsystem extends SubsystemBase {
    private TalonFX moveMotor;
    private TalonFX intakeMotor;
    private TalonFXConfiguration moveMotorConfig;
    private TalonFXConfiguration intakeMotorConfig;
    // private DigitalInput fullyInLimitSwitch;
    private ProfiledPIDController PIDie;
    private Constraints PIDConstraints;

    public enum FloorPickupPosition {
        out(1),
        in(2);

        private double value;

        private FloorPickupPosition(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }
    }

    public double TranslateEnum(FloorPickupPosition FloorPickupPosition) {
        if (FloorPickupPosition == FloorPickupPosition.out) {
            return 5.7;
        } else if (FloorPickupPosition == FloorPickupPosition.in) {
            return .5;
        } else {
            return 3;
        }

    }

    public FloorPickupPosition currentPosition = FloorPickupPosition.in;
    public FloorPickupPosition targetPosition = FloorPickupPosition.out;

    public floorIntakeSubsystem() {
        // Initialize the falcones and get encored for later use.
        moveMotor = new TalonFX(15);
        intakeMotor = new TalonFX(13);
        moveMotorConfig = new TalonFXConfiguration();
        intakeMotorConfig = new TalonFXConfiguration();
        // fullyInLimitSwitch = new DigitalInput(4);

        PIDConstraints = new Constraints(300, 400);
        // PIDie = new ProfiledPIDController(.040, 0.03, 0, PIDConstraints);
        PIDie = new ProfiledPIDController(.05, 0.05, 0, PIDConstraints);

        // Set the neutral mode to brake to stop immediately when reaching a limit
        moveMotor.setNeutralMode(NeutralModeValue.Brake);
        intakeMotor.setNeutralMode(NeutralModeValue.Brake);

        // Apply configurations to the motors
        moveMotor.getConfigurator().apply(moveMotorConfig);
        intakeMotor.getConfigurator().apply(intakeMotorConfig);

        // Initialize dashboard values
        SmartDashboard.setDefaultBoolean("Direction", true);
    }

    public void setIntakeSpeed(double speed) {
        SmartDashboard.putNumber("Intake Speed", speed);
        intakeMotor.set(speed);
    }

    public void setExtensionSpeed(double speed) {
        SmartDashboard.putNumber("Extension Speed", speed);
        moveMotor.set(speed);
    }

    // Create a public method to get the position of the encoder in a different
    // class
    public double getExtensionPosition() {
        return moveMotor.getPosition().getValueAsDouble();
    }

    public void resetPID() {
        PIDie.reset(getExtensionPosition());
    }

    public void setMotorSpeed(double speed) {
        moveMotor.set(speed);
    }

    public void setCurrentPosition(FloorPickupPosition position) {
        currentPosition = position;
    }

    public double getCurrentPosition() {
        return moveMotor.getPosition().getValueAsDouble();
    }

    public void setTargetPosition(FloorPickupPosition position) {
        targetPosition = position;
    }

    public FloorPickupPosition getTargetPosition() {
        return targetPosition;
    }

    // tells us when the elevator is Home
    // public boolean extensionHome() {
    // return !fullyInLimitSwitch.get();
    // }

    // public void resetEncoders() {
    // if (extensionHome()) {
    // moveMotor.setPosition(0);
    // }
    // }

    public void placeholder(double goal) {

        PIDie.setGoal(goal);
        setExtensionSpeed(PIDie.calculate(getExtensionPosition()));

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("current position", getCurrentPosition());
        SmartDashboard.putNumber("target position", getTargetPosition().getValue());
        // SmartDashboard.putBoolean("is lower Limit switch triggered",
        // extensionHome());
    }
}

// package frc.robot.subsystems;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// //import com.revrobotics.RelativeEncoder;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class floorIntakeSubsystem extends SubsystemBase {
// private final TalonFX floorIntakeMotor = new TalonFX(20);
// private final TalonFXConfiguration floorIntakeConfig = new
// TalonFXConfiguration().withDifferentialSensors(null);
// private final DigitalInput floorIntakeLimitSwitch = new DigitalInput(2);

// public floorIntakeSubsystem() {
// floorIntakeConfig.Slot0.kP = 0.1;
// floorIntakeConfig.Slot0.kI = 0.0;
// floorIntakeConfig.Slot0.kD = 0.0;
// floorIntakeMotor.getConfigurator().apply(floorIntakeConfig);
// floorIntakeMotor.setNeutralMode(NeutralModeValue.Brake);
// }

// public void setFloorIntakeSpeed(double speed) {
// floorIntakeMotor.set(speed);
// }

// public void extendFloorIntake() {
// // fix value of max extension
// while ( < 30) {
// setFloorIntakeSpeed(0.5);
// }
// }

// public void retractFloorIntake() {
// if (floorIntakeMotor.getPosition().getValueAsDouble() < 0) {
// setFloorIntakeSpeed(-0.5);
// }
// setFloorIntakeSpeed(0.0);
// }

// public void stopFloorIntake() {
// floorIntakeMotor.set(0.0);
// }

// public boolean isFloorIntakeLimitSwitchPressed() {
// return !floorIntakeLimitSwitch.get();
// }
// }
