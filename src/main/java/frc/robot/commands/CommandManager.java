package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.ClimberPuller;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.FunnelPivot;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.pathplanner.lib.path.PathPlannerPath;



public class CommandManager {
    //  work in progress (desperately needs some un-.andThen'ining)
  /*   public static Command coralL4(Arm arm, Elevator elevator,Intake intake){

            return setPositions(arm, elevator, 0.23, 5.2) //All hail the wall of .andThen
            .andThen(new WaitCommand(3)) //adjust the wait time accordingly
            .andThen(intake.runIntake(-0.6))
            .andThen(new WaitCommand(2.5))
            .andThen(defaultElevator(elevator))
            .andThen(defaultArm(arm));

    }
    */
    public static Command intakeCoral(Funnel funnel, Intake intake){

            //auto intake coral
            return funnel.runFunnel(0.4).alongWith(intake.runIntake(0.2))
                    .until(intake::hasCoral)
                    .andThen(new WaitCommand(0.10))
                    .andThen(funnel.stopFunnel().alongWith(intake.stopIntake()));
        }

    public static Command intakeCoral2(Funnel funnel, Intake intake){

            //auto intake coral
            return funnel.runFunnel(0.4).alongWith(intake.runIntake(0.2));
                //   until(intake::hasCoral)
                //     .andThen(new WaitCommand(0.10))
                //     .andThen(funnel.stopFunnel().alongWith(intake.stopIntake())); 
        }
    public static Command setPositions(Arm arm, Elevator elevator, double armPosition, double elevatorPosition){
             return arm.setPosition(0.25)
             .until(()-> arm.armPosition() < 0.27 && arm.armPosition() > 0.23)
             .andThen(
                elevator.setPosition(elevatorPosition)
             .alongWith(arm.setPosition(armPosition))
             
             );   
    }

    public static Command intakePositions(Arm arm, Elevator elevator) {
    // return arm.setPosition(0.36).alongWith(elevator.runElevator(0));
    // return arm.runArm(0.1).alongWith(elevator.runElevator(0)).withTimeout(0.5);
        return arm.runArm(0.1).withTimeout(0.5).andThen(elevator.runElevator(0)).andThen(arm.runArm(0));
    }

    public static Command defaultPositions(Arm arm, Elevator elevator, Intake intake) {
        return setPositions(arm, elevator, 0.30, 0.15)
        .alongWith(intake.runIntake(0));
    }

    public static Command defaultArm(Arm arm, Elevator elevator){
        return arm.runArm(0.2)
        .until(()-> arm.armPosition() < 0.37 && arm.armPosition() > 0.36)
        .andThen(arm.runArm(0)
        .alongWith(elevator.setPosition(0.01)));
        // .andThen(arm.setPosition(0.37));

        
       // return arm.setPosition(0.34); //0.365
    }

public static Command defaultElevator(Elevator elevator){
    return elevator.setPosition(0.01);
  //  .until(()-> elevator.elevatorPosition() <0.2)
    //.andThen(elevator.runElevator(0).withTimeout(0.5));
}

    public static Command netPosition(Elevator elevator, Arm arm){
        return elevator.setPosition(5.8)
        .alongWith(arm.setPosition(0.16))
        .until(()-> elevator.elevatorPosition() > 5.8)
        .andThen(
            elevator.setPosition(6)
        .alongWith(arm.setPosition(0.225))
        
        );  

}

public static Command climbPose(FunnelPivot funnelPivot, Arm arm){
    // return funnelPivot.runPivot(-0.34)
    return funnelPivot.runPivot(-0.2)
    .withTimeout(0.85)
    .andThen(funnelPivot.runPivot(0));
    // .alongWith(arm.setPosition(-0.125)); //0.30 try comm this out for keeping arm in default position should be okay FOR BEACH BLITZ
}  

//  autoCommands

    public static Command L4Pose(Elevator elevator, Arm arm){
        return setPositions(arm, elevator, 0.31, 5.3);
    }

    public static Command score(Intake intake){
        return (intake.runIntake(1))
        .withTimeout(4);

    }

    public static Command scoreL4(Elevator elevator, Arm arm, Intake intake){
        return setPositions(arm, elevator, 0.213, 5.25)
        .until(()-> elevator.elevatorPosition() > 5.25)
        .andThen(intake.runIntake(1).withTimeout(2))
        .andThen(new WaitCommand(1)) //if need to shorten, shorten
        .andThen(defaultPoses(elevator, arm)); //should retract after scoring
   }



   public static Command defaultPoses(Elevator elevator, Arm arm){
    return 
    (setPositions(arm, elevator, 0.25,0.035));
   // .andThen(defaultArm(arm));

   }
   public static Command climberExtend(ClimberPuller climber){
        return climber.runClimber(1)
        .until(()-> climber.climberPosition() > 3.5);

   }

   public static Command climberRetract(ClimberPuller climber, FunnelPivot funnelPivot){
    return climber.runClimber(-1)
    .until(()-> climber.climberPosition() < 0.8)
    .andThen(funnelPivot.runPivot(0.1)
    .withTimeout(0.1));




}

}
