// package frc.robot.commands;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.ElevatorSubsystem.ElevatorPosition;

// public class LiftElevToHallEffect extends Command {
//     private ElevatorSubsystem m_elevator;
//     private ElevatorPosition targetPosition;
//     private boolean startingAtHome;
//     private boolean isPastHallEffect;
//     private boolean startingAthallEffect;

//     public LiftElevToHallEffect(ElevatorSubsystem elevator, ElevatorPosition targetPosition) {
//         this.targetPosition = targetPosition;
//         m_elevator = elevator;
//         addRequirements(m_elevator);
//     }

//     @Override
//     public void initialize() {
//         m_elevator.setTargetPosition(targetPosition);
//         System.out.println("Target Position changed");
//         startingAtHome = m_elevator.elevatorHome();
//         startingAthallEffect = m_elevator.isHallEffectTripped();
//         currentPosition = 1;
//         isPastHallEffect = false;
//         System.out.println("Starting LiftElevToHallEffect");
//     }

//     @Override
//     public void execute() {
//         m_elevator.currentisTripped(m_elevator.isHallEffectTripped());
//         m_elevator.moveUp();
//         System.out.println("Executing LiftElevToHallEffect");

//         if(m_elevator.becameTrue()) {
//             System.out.println("Hall effect triggered");
//             currentPosition = currentPosition + 1 ;
//             m_elevator.previousisTripped(true);
//             System.out.println("Current Position: " + currentPosition); 
//         }
//         if(m_elevator.becameFalse()) {
//             System.out.println("Hall effect untriggered");
//             m_elevator.previousisTripped(false);
//         }
//         if(m_elevator.maxHeight()) {
//             currentPosition = 5;
//             System.out.println("Current Position: " + currentPosition); 
//         }
//     }

//     @Override
//     public boolean isFinished() {

//         if(!startingAtHome) {
//             System.out.println("Finished because not starting at home");
//             return true;
//         }

//         if(m_elevator.getCurrentPosition() == targetPosition) {
//             System.out.println("Finished because current position is target position");
//             return true;
//         }
//         return false;
//     }


//         // if(startingAthallEffect){
//         //     System.out.println("Starting at hall effect");
//         //     //if elevator is not past hall effect and hall effect becomes false, then it is now past hall effect
//         //     if (!isPastHallEffect && m_elevator.becameFalse()){
//         //         System.out.println("Past starting hall effect now");
//         //         isPastHallEffect = true;
//         //     }

//         //     if(isPastHallEffect && m_elevator.becameTrue()){
//         //         System.out.println("Finished because halleffect triggered bacame true");
//         //         m_elevator.previousisTripped(m_elevator.isHallEffectTripped());
//         //         return true;
//         //     }
//         //     m_elevator.previousisTripped(m_elevator.isHallEffectTripped());
//         //     return false;
//         // }
//         // else{
//         //     System.out.println("Starting elsewhere");
//         //     // if you don't start at a hall effect, finish once you hit one
//         //     if(m_elevator.becameTrue()){
//         //         System.out.println("Finished because halleffect triggered became true");
//         //         return true;
//         //     }
//         //     return false;
//         //}

//     @Override
//     public void end(boolean interrupted) {
//         System.out.println("Ending LiftElevToHallEffect");
//         m_elevator.setMotorSpeed(0);
//     }

// }