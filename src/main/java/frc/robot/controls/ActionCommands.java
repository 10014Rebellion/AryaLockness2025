package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Claw.ClawSubsystem;
import frc.robot.subsystems.Climb.ClimbSubsystem;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Elevator.ElevatorConstants.ElevatorSetpoints;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Wrist.WristConstants;
import frc.robot.subsystems.Wrist.WristSubsystem;
import frc.robot.subsystems.Wrist.WristConstants.WristSetpoints;

public class ActionCommands {
    private final WristSubsystem mWrist;
	private final ElevatorSubsystem mElevator;
	private final IntakeSubsystem mIntake;
	private final ClawSubsystem mClaw;
	private final ClimbSubsystem mClimb;

    public ActionCommands(){
        mWrist = new WristSubsystem();
        mElevator = new ElevatorSubsystem();
        mIntake = new IntakeSubsystem();
        mClaw = new ClawSubsystem();
        mClimb = new ClimbSubsystem();
    }

    public SequentialCommandGroup getIntakeCoralCmd(){
        return new SequentialCommandGroup(
            new ParallelDeadlineGroup(
                new ParallelCommandGroup(
                    mIntake.intakeCmd(),
                    new SequentialCommandGroup(
                        mElevator.setElevatorPIDCmd(ElevatorConstants.ElevatorSetpoints.PREINTAKE),
                        mWrist.setWristPIDCmd(WristConstants.WristSetpoints.INTAKE)   
                    )
                )
            ),
            new WaitCommand(0.35),
            new ParallelCommandGroup(
                mElevator.setElevatorPIDCmd(ElevatorConstants.ElevatorSetpoints.POSTINTAKE),
                mWrist.setWristPIDCmd(WristConstants.WristSetpoints.INTAKE),
                mClaw.clawIntakeCoralCmd()   
            ),

            mElevator.setElevatorPIDCmd(ElevatorConstants.ElevatorSetpoints.PREINTAKE)

        );
    }

    public SequentialCommandGroup scoreCoralL1(){
        return new SequentialCommandGroup(
                mElevator.setElevatorPIDCmd(ElevatorConstants.ElevatorSetpoints.L1),
                mWrist.setWristPIDCmd(WristConstants.WristSetpoints.L1),
                mClaw.clawOuttakeCoralCmd()
            
        );
    }


}

