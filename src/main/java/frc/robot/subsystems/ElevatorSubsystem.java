package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem extends SubsystemBase {
    private SparkMax motor1;
    private SparkMax motor2;
    private SparkMaxConfig motorConfig1;
    private SparkMaxConfig motorConfig2;
    private SparkLimitSwitch forwardLimitSwitch1;
    private SparkLimitSwitch reverseLimitSwitch1;
    private SparkLimitSwitch forwardLimitSwitch2;
    private SparkLimitSwitch reverseLimitSwitch2;
    private RelativeEncoder encoder1;
    private RelativeEncoder encoder2;
    private DigitalInput upperLimitSwitch;
    private DigitalInput lowerLimitSwitch;
    private ProfiledPIDController PIDie;
    private Constraints PIDConstraints;

    public enum ElevatorPosition {
        Home(1),
        L1(2),
        L2(3),
        L3(4),
        L4(5),
        A1(6),
        A2(7),
        INTAKE(8);

        private double value;

        private ElevatorPosition(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }
    }

    public ElevatorPosition currentPosition = ElevatorPosition.Home;
    public ElevatorPosition targetPosition = ElevatorPosition.L2;

    public ElevatorSubsystem() {
        // Initialize the SPARK MAX motors and get their limit switch and encoder
        // objects for later use.
        motor1 = new SparkMax(1, MotorType.kBrushless);
        motor2 = new SparkMax(2, MotorType.kBrushless);
        forwardLimitSwitch1 = motor1.getForwardLimitSwitch();
        reverseLimitSwitch1 = motor1.getReverseLimitSwitch();
        forwardLimitSwitch2 = motor2.getForwardLimitSwitch();
        reverseLimitSwitch2 = motor2.getReverseLimitSwitch();
        encoder1 = motor1.getEncoder();
        encoder2 = motor2.getEncoder();

        PIDConstraints = new Constraints(300, 400);
        PIDie = new ProfiledPIDController(.040, 0.03, 0, PIDConstraints);

        upperLimitSwitch = new DigitalInput(5);
        lowerLimitSwitch = new DigitalInput(3);

        // Create new SPARK MAX configuration objects.
        motorConfig1 = new SparkMaxConfig();
        motorConfig2 = new SparkMaxConfig();

        // Set the idle mode to brake to stop immediately when reaching a limit
        motorConfig1.idleMode(IdleMode.kBrake);
        motorConfig2.idleMode(IdleMode.kBrake);

        // Enable limit switches to stop the motors when they are closed
        motorConfig1.limitSwitch
                .forwardLimitSwitchType(Type.kNormallyOpen)
                .forwardLimitSwitchEnabled(true)
                .reverseLimitSwitchType(Type.kNormallyOpen)
                .reverseLimitSwitchEnabled(true);
        motorConfig2.limitSwitch
                .forwardLimitSwitchType(Type.kNormallyOpen)
                .forwardLimitSwitchEnabled(true)
                .reverseLimitSwitchType(Type.kNormallyOpen)
                .reverseLimitSwitchEnabled(true);

        // Set the soft limits to stop the motors at -50 and 50 rotations
        /*
         * motorConfig1.softLimit
         * .forwardSoftLimit(50)
         * .forwardSoftLimitEnabled(true)
         * .reverseSoftLimit(-50)
         * .reverseSoftLimitEnabled(true);
         * motorConfig2.softLimit
         * .forwardSoftLimit(50)
         * .forwardSoftLimitEnabled(true)
         * .reverseSoftLimit(-50)
         * .reverseSoftLimitEnabled(true);
         */
        // Set Motor 2 to follow Motor 1
        motorConfig2.follow(motor1);

        // Apply the configuration to the SPARK MAX motors.
        motor1.configure(motorConfig1, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        motor2.configure(motorConfig2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // Reset the positions to 0 to start within the range of the soft limits
        encoder1.setPosition(0);
        encoder2.setPosition(0);

        // Initialize dashboard values
        SmartDashboard.setDefaultBoolean("Direction", true);
    }

    public void moveUp() {
        // If the upper limit switch is not pressed, move the elevator up
        if (upperLimitSwitch.get()) {
            setMotorSpeed(-0.2);
        } else {
            setMotorSpeed(0);
        }
    }

    public void moveDown() {
        // If the lower limit switch is not pressed, move the elevator down
        if (lowerLimitSwitch.get()) {
            setMotorSpeed(0.2);
        } else {
            setMotorSpeed(0);
        }
    }

    public void setMotorSpeed(double speed) {
        SmartDashboard.putNumber("Elevator Speed", speed);
        motor1.set(speed);
    }

    // Create a public method to get the position of the encoder in a different
    // class
    public double getEncoder1Position() {
        return encoder1.getPosition();
    }

    public double TranslateEnum(ElevatorPosition elevatorPosition) {
        if (elevatorPosition == ElevatorPosition.L1) {
            return -6;
        } else if (elevatorPosition == ElevatorPosition.L2) {
            return -11.417;
        } else if (elevatorPosition == ElevatorPosition.L3) {
            return -22;
        } else if (elevatorPosition == ElevatorPosition.L4) {
            return -38;
        } else if (elevatorPosition == ElevatorPosition.A1) {
            return -8.5;
        } else if (elevatorPosition == ElevatorPosition.A2) {
            return -19;
        } else {
            return 0;
        }

    }

    public void resetPID() {
        PIDie.reset(getEncoder1Position());
    }

    public void setCurrentPosition(ElevatorPosition position) {
        currentPosition = position;
    }

    public ElevatorPosition getCurrentPosition() {
        return currentPosition;
    }

    public void setTargetPosition(ElevatorPosition position) {
        targetPosition = position;
    }

    public ElevatorPosition getTargetPosition() {
        return targetPosition;
    }

    // tells us when the elevator is Home
    public boolean elevatorHome() {
        return !lowerLimitSwitch.get();
    }

    public boolean maxHeight() {
        return !upperLimitSwitch.get();
    }

    public void resetEncoders() {
        if (elevatorHome()) {
            encoder1.setPosition(0);
            encoder2.setPosition(0);
        }
    }

    public void placeholder(double goal) {

        PIDie.setGoal(goal);
        setMotorSpeed(PIDie.calculate(getEncoder1Position()) - .035);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("current position", getCurrentPosition().getValue());
        SmartDashboard.putNumber("target position", getTargetPosition().getValue());
        SmartDashboard.putBoolean("is upper Limit switch triggered", maxHeight());
        SmartDashboard.putBoolean("is lower Limit switch triggered", elevatorHome());
        // Display data from SPARK onto the dashboard
        SmartDashboard.putBoolean("Forward Limit Reached Motor 1", forwardLimitSwitch1.isPressed());
        SmartDashboard.putBoolean("Reverse Limit Reached Motor 1", reverseLimitSwitch1.isPressed());
        SmartDashboard.putBoolean("Forward Limit Reached Motor 2", forwardLimitSwitch2.isPressed());
        SmartDashboard.putBoolean("Reverse Limit Reached Motor 2", reverseLimitSwitch2.isPressed());
        SmartDashboard.putNumber("Applied Output Motor 1", motor1.getAppliedOutput());
        SmartDashboard.putNumber("Applied Output Motor 2", motor2.getAppliedOutput());
        SmartDashboard.putNumber("Position Motor 1", encoder1.getPosition());
        SmartDashboard.putNumber("Position Motor 2", encoder2.getPosition());

    }
}