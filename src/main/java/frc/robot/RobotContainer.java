// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.util.Units;

import java.lang.management.OperatingSystemMXBean;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.SequenceWriter;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix.ButtonMonitor;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AdvToShooter;
import frc.robot.commands.HomeElevator;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.OutakeAlgae;
import frc.robot.commands.PIDElevator;
import frc.robot.commands.PIDWrist;
import frc.robot.commands.PIDfloorMovement;
import frc.robot.commands.SetLeds;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;
import frc.robot.subsystems.PhotonCam;
//import frc.robot.commands.LiftElevToHallEffect;
import frc.robot.commands.ShootCoral;
import frc.robot.commands.SlowShootCoral;
import frc.robot.commands.StartRampWheel;
import frc.robot.commands.StartShooterWheel;
import frc.robot.commands.StopRampWheel;
import frc.robot.commands.StopShooterWheel;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RampSubSystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.WristSubSystem;
import frc.robot.subsystems.floorIntakeSubsystem;
import frc.robot.subsystems.WristSubSystem.WristPosition;
import frc.robot.subsystems.floorIntakeSubsystem.FloorPickupPosition;

public class RobotContainer {

        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.RobotCentric rcDrive = new SwerveRequest.RobotCentric().withDeadband(MaxSpeed * 0.1)
                        .withRotationalDeadband(MaxAngularRate * 0.1);
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        // private final CommandXboxController driverJoystick = new
        // CommandXboxController(0);
        private final PS4Controller driverJoystick = new PS4Controller(0);
        private final PS4Controller operatorJoystick = new PS4Controller(1);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
        private final floorIntakeSubsystem floorIntakeSubsystem = new floorIntakeSubsystem();
        private final WristSubSystem wristSubsystem = new WristSubSystem();
        private final RampSubSystem rampSubsystem = new RampSubSystem();
        private final Shooter shooter = new Shooter();
        public PhotonCam cam = new PhotonCam();
        private final CANdleSubsystem m_candle = new CANdleSubsystem();

        private final SendableChooser<Command> autoChooser;

        public RobotContainer() {

                configureAutos();

                autoChooser = AutoBuilder.buildAutoChooser("Blank");
                SmartDashboard.putData("Auto Mode", autoChooser);

                configureBindings();
        }

        private double squareInput(double input) {
                boolean negative = input < 0;
                if (negative) {
                        return -Math.pow(input, 2);
                } else {
                        return Math.pow(input, 2);
                }
        }

        private void configureAutos() {

                NamedCommands.registerCommand("l4", new SequentialCommandGroup(
                                new RunCommand(() -> wristSubsystem
                                                .setCurrentPosition(WristPosition.HOME),
                                                wristSubsystem).withTimeout(0.1),
                                new PIDElevator(ElevatorPosition.L4, elevatorSubsystem),
                                new WaitCommand(.3),
                                new ShootCoral(shooter)));

                NamedCommands.registerCommand("Home", new SequentialCommandGroup(
                                new StopShooterWheel(shooter),
                                new RunCommand(() -> wristSubsystem
                                                .setCurrentPosition(WristPosition.HOME),
                                                wristSubsystem).withTimeout(0.1),
                                new PIDElevator(ElevatorPosition.Home, elevatorSubsystem),
                                new RunCommand(() -> elevatorSubsystem.setMotorSpeed(0),
                                                elevatorSubsystem).withTimeout(0.1)));

                NamedCommands.registerCommand("Intake", new SequentialCommandGroup(
                                new PIDElevator(ElevatorPosition.Home, elevatorSubsystem),
                                new RunCommand(() -> elevatorSubsystem.setMotorSpeed(0),
                                                elevatorSubsystem).withTimeout(0.1),
                                new ParallelCommandGroup(
                                                new AdvToShooter(shooter),
                                                new StartRampWheel(rampSubsystem)),
                                new ParallelCommandGroup(
                                                new StopRampWheel(rampSubsystem),
                                                new StartShooterWheel(shooter)),
                                new WaitCommand(.1),
                                new StopShooterWheel(shooter)));

                NamedCommands.registerCommand("Low Algie Pickup", new SequentialCommandGroup(
                                new PIDElevator(ElevatorPosition.A1, elevatorSubsystem),
                                new RunCommand(() -> wristSubsystem
                                                .setCurrentPosition(WristPosition.ALGAEPICKUP),
                                                wristSubsystem).withTimeout(0.1),
                                new IntakeAlgae(shooter).withTimeout(1),
                                new RunCommand(() -> shooter.setShooterSpeed(.2), shooter)
                                                .withTimeout(.3),
                                new RunCommand(() -> wristSubsystem
                                                .setCurrentPosition(WristPosition.HOME),
                                                wristSubsystem).withTimeout(0.1)));

                NamedCommands.registerCommand("High Algie Pickup", new SequentialCommandGroup(
                                new PIDElevator(ElevatorPosition.A2, elevatorSubsystem),
                                new RunCommand(() -> wristSubsystem
                                                .setCurrentPosition(WristPosition.ALGAEPICKUP),
                                                wristSubsystem).withTimeout(0.1),
                                new IntakeAlgae(shooter).withTimeout(1),
                                new RunCommand(() -> shooter.setShooterSpeed(.2), shooter)
                                                .withTimeout(.3),
                                new RunCommand(() -> wristSubsystem
                                                .setCurrentPosition(WristPosition.HOME),
                                                wristSubsystem).withTimeout(0.1)));

                NamedCommands.registerCommand("Barge", new SequentialCommandGroup(
                                new PIDElevator(ElevatorPosition.L4, elevatorSubsystem),
                                new RunCommand(() -> wristSubsystem
                                                .setCurrentPosition(WristPosition.ALGAESHOOT),
                                                wristSubsystem).withTimeout(0.3),
                                new OutakeAlgae(shooter).withTimeout(.3),
                                new RunCommand(() -> wristSubsystem
                                                .setCurrentPosition(WristPosition.HOME),
                                                wristSubsystem).withTimeout(0.1),
                                new PIDElevator(ElevatorPosition.Home, elevatorSubsystem),
                                new RunCommand(() -> elevatorSubsystem.setMotorSpeed(0),
                                                elevatorSubsystem).withTimeout(0.1)));

        }

        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain
                                                .applyRequest(() -> drive.withVelocityX(
                                                                -.7 * squareInput(driverJoystick.getLeftY()) * MaxSpeed) // Drive
                                                                                                                         // forward
                                                                                                                         // with
                                                                                                                         // negative
                                                                                                                         // Y
                                                                                                                         // (forward)
                                                                .withVelocityY(-.7
                                                                                * squareInput(driverJoystick.getLeftX())
                                                                                * MaxSpeed) // Drive left
                                                                                            // with negative
                                                                                            // X (left)
                                                                .withRotationalRate(-.7
                                                                                * squareInput(driverJoystick
                                                                                                .getRightX())
                                                                                * MaxAngularRate) // Drive
                                                                                                  // counterclockwise
                                                                                                  // with
                                                                                                  // negative
                                                                                                  // X
                                                                                                  // (left)
                                                ));

                m_candle.setDefaultCommand(new SetLeds(cam, m_candle));

                /*
                 * DRIVER CONTROLLER BINDINGS
                 */

                new JoystickButton(driverJoystick, 2).whileTrue(drivetrain.applyRequest(() -> brake));
                new JoystickButton(driverJoystick, 3).whileTrue(drivetrain.applyRequest(() -> point
                                .withModuleDirection(new Rotation2d(-driverJoystick.getLeftY(),
                                                -driverJoystick.getLeftX()))));

                // reset the field-centric heading on share button press
                new JoystickButton(driverJoystick, 9).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                // Driver Left Bumper bind to strafe left t
                new JoystickButton(driverJoystick, 5).whileTrue(
                                drivetrain
                                                .applyRequest(() -> rcDrive.withVelocityX(0) // Drive
                                                                                             // forward
                                                                                             // with
                                                                                             // negative
                                                                                             // Y
                                                                                             // (forward)
                                                                .withVelocityY(.2 * MaxSpeed) // Drive left
                                                                // with negative
                                                                // X (left)
                                                                .withRotationalRate(0) // Drive
                                                                                       // counterclockwise
                                                                                       // with
                                                                                       // negative
                                                                                       // X
                                                                                       // (left)
                                                ));

                // Driver Right Bumper bind to strafe right
                new JoystickButton(driverJoystick, 6).whileTrue(
                                drivetrain
                                                .applyRequest(() -> rcDrive.withVelocityX(0) // Drive
                                                                                             // forward
                                                                                             // with
                                                                                             // negative
                                                                                             // Y
                                                                                             // (forward)
                                                                .withVelocityY(-.2 * MaxSpeed) // Drive left
                                                                // with negative
                                                                // X (left)
                                                                .withRotationalRate(0) // Drive
                                                                                       // counterclockwise
                                                                                       // with
                                                                                       // negative
                                                                                       // X
                                                                                       // (left)
                                                ));
                // Driver right Trigger bind to strafe forward
                new JoystickButton(driverJoystick, 8).whileTrue(
                                drivetrain
                                                .applyRequest(() -> rcDrive.withVelocityX(.1 * MaxSpeed) // Drive
                                                                // forward
                                                                // with
                                                                // negative
                                                                // Y
                                                                // (forward)
                                                                .withVelocityY(0) // Drive left
                                                                // with negative
                                                                // X (left)
                                                                .withRotationalRate(0) // Drive
                                                                                       // counterclockwise
                                                                                       // with
                                                                                       // negative
                                                                                       // X
                                                                                       // (left)
                                                ));
                // Driver left Trigger bind to strafe backward
                new JoystickButton(driverJoystick, 7).whileTrue(
                                drivetrain
                                                .applyRequest(() -> rcDrive.withVelocityX(-.1 * MaxSpeed) // Drive
                                                                // forward
                                                                // with
                                                                // negative
                                                                // Y
                                                                // (forward)
                                                                .withVelocityY(0) // Drive left
                                                                // with negative
                                                                // X (left)
                                                                .withRotationalRate(0) // Drive
                                                                                       // counterclockwise
                                                                                       // with
                                                                                       // negative
                                                                                       // X
                                                                                       // (left)
                                                ));

                drivetrain.registerTelemetry(logger::telemeterize);

                /*
                 * SYSID
                 */

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                // new JoystickButton(driverJoystick, 9).and( new JoystickButton(driverJoystick,
                // 4)).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                // new JoystickButton(driverJoystick, 9).and( new JoystickButton(driverJoystick,
                // 1)).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                // new JoystickButton(driverJoystick, 10).and( new
                // JoystickButton(driverJoystick,
                // 4)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                // new JoystickButton(driverJoystick, 10).and( new
                // JoystickButton(driverJoystick,
                // 1)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                /*
                 * OPERATOR CONTROLLER BINDINGS
                 */

                // Bind Operator left POV button intake Coral directly to shooter
                new POVButton(operatorJoystick, 270).onTrue(
                                new SequentialCommandGroup(
                                                new RunCommand(() -> wristSubsystem
                                                                .setCurrentPosition(WristPosition.HOME),
                                                                wristSubsystem).withTimeout(0.3),
                                                new PIDElevator(ElevatorPosition.Home, elevatorSubsystem)
                                                                .withTimeout(.3),
                                                new RunCommand(() -> elevatorSubsystem.setMotorSpeed(.035),
                                                                elevatorSubsystem).withTimeout(.3),
                                                new RunCommand(() -> elevatorSubsystem.setMotorSpeed(0),
                                                                elevatorSubsystem).withTimeout(0.1),
                                                new ParallelCommandGroup(
                                                                new AdvToShooter(shooter),
                                                                new StartRampWheel(rampSubsystem)),
                                                new ParallelCommandGroup(
                                                                new StopRampWheel(rampSubsystem),
                                                                new StartShooterWheel(shooter)),
                                                new WaitCommand(.1),
                                                new StopShooterWheel(shooter)));

                // Binds Operator right bumper button to stop intaking
                new POVButton(operatorJoystick, 90).onTrue(
                                new ParallelCommandGroup(
                                                new StopRampWheel(rampSubsystem),
                                                new StopShooterWheel(shooter)));

                // Bind Operator circle button to shoot L1
                new JoystickButton(operatorJoystick, 3).onTrue(
                                new SequentialCommandGroup(
                                                new RunCommand(() -> wristSubsystem
                                                                .setCurrentPosition(WristPosition.HOME),
                                                                wristSubsystem).withTimeout(0.3),
                                                new PIDElevator(ElevatorPosition.L1, elevatorSubsystem)));
                // setLEDSTate(LEDState.YELLOW);

                // Bind Operator cross button to shoot L2
                new JoystickButton(operatorJoystick, 2).onTrue(
                                new SequentialCommandGroup(
                                                new RunCommand(() -> wristSubsystem
                                                                .setCurrentPosition(WristPosition.HOME),
                                                                wristSubsystem).withTimeout(0.3),
                                                new PIDElevator(ElevatorPosition.L2, elevatorSubsystem)));

                // Bind Operator square button to shoot L3
                new JoystickButton(operatorJoystick, 1).onTrue(
                                new SequentialCommandGroup(
                                                new RunCommand(() -> wristSubsystem
                                                                .setCurrentPosition(WristPosition.HOME),
                                                                wristSubsystem).withTimeout(0.3),
                                                new PIDElevator(ElevatorPosition.L3, elevatorSubsystem)));

                // Bind Operator triange button to shoot L4
                new JoystickButton(operatorJoystick, 4).onTrue(
                                new SequentialCommandGroup(
                                                new RunCommand(() -> wristSubsystem
                                                                .setCurrentPosition(WristPosition.HOME),
                                                                wristSubsystem).withTimeout(0.3),
                                                new PIDElevator(ElevatorPosition.L4, elevatorSubsystem),
                                                new WaitCommand(.3),
                                                new ShootCoral(shooter)));

                // Bind Operator touchpad button to Home
                new JoystickButton(operatorJoystick, 14).onTrue(
                                new SequentialCommandGroup(
                                                new StopShooterWheel(shooter),
                                                new RunCommand(() -> wristSubsystem
                                                                .setCurrentPosition(WristPosition.HOME),
                                                                wristSubsystem).withTimeout(0.3),
                                                new PIDElevator(ElevatorPosition.Home, elevatorSubsystem),
                                                new RunCommand(() -> elevatorSubsystem.setMotorSpeed(0.035),
                                                                elevatorSubsystem).withTimeout(.3),
                                                new RunCommand(() -> elevatorSubsystem.setMotorSpeed(0),
                                                                elevatorSubsystem).withTimeout(0.1),
                                                new RunCommand(() -> elevatorSubsystem.resetEncoders(),
                                                                elevatorSubsystem).withTimeout(.5)));

                // Bind Operator left trigger to shoot coral
                new JoystickButton(operatorJoystick, 7).onTrue(
                                new SlowShootCoral(shooter));

                // Bind intake Algae A1 button to down POV
                new POVButton(operatorJoystick, 180).onTrue(
                                new SequentialCommandGroup(
                                                new RunCommand(() -> wristSubsystem
                                                                .setCurrentPosition(WristPosition.HOME),
                                                                wristSubsystem).withTimeout(0.3),
                                                new PIDElevator(ElevatorPosition.A1, elevatorSubsystem),
                                                new RunCommand(() -> wristSubsystem
                                                                .setCurrentPosition(WristPosition.ALGAEPICKUP),
                                                                wristSubsystem).withTimeout(.1),
                                                new IntakeAlgae(shooter),
                                                new RunCommand(() -> shooter.setShooterSpeed(.2), shooter)
                                                                .withTimeout(.3),

                                                new RunCommand(() -> wristSubsystem
                                                                .setCurrentPosition(WristPosition.HOME),
                                                                wristSubsystem).withTimeout(0.1)));
                // Bind floor pickup to R2
                new JoystickButton(operatorJoystick, 8).onTrue(
                                new SequentialCommandGroup(
                                                new RunCommand(() -> wristSubsystem
                                                                .setCurrentPosition(WristPosition.HOME),
                                                                wristSubsystem).withTimeout(0.3),
                                                new PIDElevator(ElevatorPosition.Home, elevatorSubsystem),
                                                new RunCommand(() -> wristSubsystem
                                                                .setCurrentPosition(WristPosition.FLOORPICKUP),
                                                                wristSubsystem).withTimeout(.1),
                                                new IntakeAlgae(shooter),
                                                new RunCommand(() -> shooter.setShooterSpeed(.2), shooter)
                                                                .withTimeout(.3),

                                                new RunCommand(() -> wristSubsystem
                                                                .setCurrentPosition(WristPosition.HOME),
                                                                wristSubsystem).withTimeout(0.1)));
                // Bind intake Algae A2 button to Up POV
                new POVButton(operatorJoystick, 0).onTrue(
                                new SequentialCommandGroup(
                                                new RunCommand(() -> wristSubsystem
                                                                .setCurrentPosition(WristPosition.HOME),
                                                                wristSubsystem).withTimeout(0.3),
                                                new PIDElevator(ElevatorPosition.A2, elevatorSubsystem),
                                                new RunCommand(() -> wristSubsystem
                                                                .setCurrentPosition(WristPosition.ALGAEPICKUP),
                                                                wristSubsystem).withTimeout(.1),
                                                new IntakeAlgae(shooter),
                                                new RunCommand(() -> shooter.setShooterSpeed(.2), shooter)
                                                                .withTimeout(.3),

                                                new RunCommand(() -> wristSubsystem
                                                                .setCurrentPosition(WristPosition.HOME),
                                                                wristSubsystem).withTimeout(0.1)));
                // Bind Outake Algae in Barge to left Bumper
                new JoystickButton(operatorJoystick, 5).onTrue(
                                new SequentialCommandGroup(
                                                new RunCommand(() -> wristSubsystem
                                                                .setCurrentPosition(WristPosition.HOME),
                                                                wristSubsystem).withTimeout(0.3),
                                                new PIDElevator(ElevatorPosition.L4, elevatorSubsystem),
                                                new RunCommand(() -> wristSubsystem
                                                                .setCurrentPosition(WristPosition.ALGAESHOOT),
                                                                wristSubsystem).withTimeout(0.3)));
                // new OutakeAlgae(shooter).withTimeout(.5),
                // new RunCommand(() -> wristSubsystem
                // .setCurrentPosition(WristPosition.HOME),
                // wristSubsystem).withTimeout(1),
                // new PIDElevator(ElevatorPosition.Home, elevatorSubsystem)));

                // Bind Outake Algae to Right Bumper
                new JoystickButton(operatorJoystick, 6).onTrue(
                                new SequentialCommandGroup(
                                                // new RunCommand(() -> wristSubsystem
                                                // .setCurrentPosition(WristPosition.HOME),
                                                // wristSubsystem).withTimeout(0.3),
                                                new OutakeAlgae(shooter).withTimeout(.3)));
                // bind options to floor out
                new JoystickButton(operatorJoystick, 10).onTrue(
                                new SequentialCommandGroup(
                                                new PIDfloorMovement(FloorPickupPosition.out, floorIntakeSubsystem)));
                // new RunCommand(() -> floorIntakeSubsystem.setIntakeSpeed(.2),
                // floorIntakeSubsystem),
                // new RunCommand(() -> wristSubsystem
                // .setCurrentPosition(WristPosition.ALGAESHOOT),
                // wristSubsystem).withTimeout(0.3)));
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

}
