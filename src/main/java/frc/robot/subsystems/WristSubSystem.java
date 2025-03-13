package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

public class WristSubSystem extends SubsystemBase {
  private final TalonFX wristMotor = new TalonFX(9);// TODO: update with correct value
  private final TalonFXConfiguration wristConfig = new TalonFXConfiguration();
  final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
  private CANcoder wristCaNcoder = new CANcoder(14);// TODO: update with correct value
  private ProfiledPIDController wristPID;
  private Constraints wristPIDConstraints;

  public enum WristPosition {// TODO: set these values correctly
    HOME(1),
    CORALL4(2),
    ALGAEPICKUP(3),
    ALGAESHOOT(4),
    PROCESSOR(5);

    private double value;

    private WristPosition(double value) {
      this.value = value;
    }

    public double getValue() {
      return value;
    }
  }

  public WristPosition currentPosition = WristPosition.HOME;
  public WristPosition targetPosition = WristPosition.HOME;

  public WristSubSystem() {

    wristPIDConstraints = new Constraints(300, 400);
    wristPID = new ProfiledPIDController(4, 0, 0, wristPIDConstraints);

  }

  // What in the heck is a StatusSignal!?
  public double getWristEncoderPosition() {
    return wristCaNcoder.getPosition().getValueAsDouble();
  }

  public double TranslateEnum(WristPosition wristPosition) {
    if (wristPosition == WristPosition.CORALL4) {
      return .335;
    } else if (wristPosition == WristPosition.ALGAEPICKUP) {
      return .436; // TODO: change to correct
    } else if (wristPosition == WristPosition.ALGAESHOOT) {
      return .2; // TODO: change to correct
    } else if (wristPosition == WristPosition.PROCESSOR) {
      return .38;
    }

    else {
      return 0.353; // home
    }
  }

  public void resetPID() {
    wristPID.reset(getWristEncoderPosition());
  }

  public void setCurrentPosition(WristPosition position) {
    currentPosition = position;
  }

  public WristPosition getCurrentPosition() {
    return currentPosition;
  }

  public void setTargetPosition(WristPosition position) {
    targetPosition = position;
  }

  public WristPosition getTargetPosition() {
    return targetPosition;
  }

  public void setWristGoal(double goal) {

    wristPID.setGoal(goal);
    setMotorSpeed(wristPID.calculate(getWristEncoderPosition()));
  }

  public void setMotorSpeed(double speed) {
    SmartDashboard.putNumber("Elevator Speed", speed);
    wristMotor.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist Position", getCurrentPosition().getValue());
    SmartDashboard.putNumber("Wrist Cancoder", getWristEncoderPosition());
    setWristGoal(TranslateEnum(currentPosition));

  }
}
