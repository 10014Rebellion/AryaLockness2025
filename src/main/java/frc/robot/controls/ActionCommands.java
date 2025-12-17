package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Claw.ClawConstants;
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

    public ActionCommands() {
        mWrist = new WristSubsystem();
        mElevator = new ElevatorSubsystem();
        mIntake = new IntakeSubsystem();
        mClaw = new ClawSubsystem();
        mClimb = new ClimbSubsystem();
    }

    public SequentialCommandGroup getIntakeCoralCmd() {
        return new SequentialCommandGroup(
                mWrist.setWristPIDCmd(WristSetpoints.TEST_90),
                new ParallelCommandGroup(
                        mIntake.intakeCmd(),
                        new SequentialCommandGroup(
                                mElevator.setElevatorPIDCmd(ElevatorConstants.ElevatorSetpoints.PREINTAKE),
                                mWrist.setWristPIDCmd(WristConstants.WristSetpoints.INTAKE))
                                ),

                new ParallelCommandGroup(
                    mClaw.clawIntakeCoralCmd(),
                    new SequentialCommandGroup(
                        mWrist.setWristPIDCmd(WristConstants.WristSetpoints.INTAKE),
                        mElevator.setElevatorPIDCmd(ElevatorConstants.ElevatorSetpoints.POSTINTAKE)
                    ) 
                ),

                mElevator.setElevatorPIDCmd(ElevatorConstants.ElevatorSetpoints.PREINTAKE)

        );
    }

    //L1 Commands
        public SequentialCommandGroup prepCoralL1() {
            return new SequentialCommandGroup(
                    mElevator.setElevatorPIDCmd(ElevatorConstants.ElevatorSetpoints.L1),
                    mWrist.setWristPIDCmd(WristSetpoints.L1));
        }

        public ParallelCommandGroup scoreCoralL1() {
            return new ParallelCommandGroup(
                    mWrist.setWristPIDCmd(WristSetpoints.SCORE),
                    new WaitCommand(0.1).andThen(new InstantCommand(() -> mClaw.setVolts(ClawConstants.ClawVolts.EJECT_CORAL.getClawVolts()))));
        }

    //L2 Commands
        public SequentialCommandGroup prepCoralL2() {
            return new SequentialCommandGroup(
                    mElevator.setElevatorPIDCmd(ElevatorConstants.ElevatorSetpoints.L2),
                    mWrist.setWristPIDCmd(WristSetpoints.L2));
        }

        public ParallelCommandGroup scoreCoralL2() {
            return new ParallelCommandGroup(
                    mWrist.setWristPIDCmd(WristSetpoints.SCORE),
                    new WaitCommand(0.1).andThen(new InstantCommand(() -> mClaw.setVolts(ClawConstants.ClawVolts.EJECT_CORAL.getClawVolts()))));
        }
    
    //L3 Commands
        public SequentialCommandGroup prepCoralL3() {
            return new SequentialCommandGroup(
                    mElevator.setElevatorPIDCmd(ElevatorConstants.ElevatorSetpoints.L3),
                    mWrist.setWristPIDCmd(WristSetpoints.L3));
        }

        public ParallelCommandGroup scoreCoralL3() {
            return new ParallelCommandGroup(
                    mWrist.setWristPIDCmd(WristSetpoints.SCORE),
                    new WaitCommand(0.1).andThen(new InstantCommand(() -> mClaw.setVolts(ClawConstants.ClawVolts.EJECT_CORAL.getClawVolts()))));
        }
    
    //L4 Commands
        public SequentialCommandGroup prepCoralL4() {
            return new SequentialCommandGroup(
                    mElevator.setElevatorPIDCmd(ElevatorConstants.ElevatorSetpoints.L4),
                    mWrist.setWristPIDCmd(WristSetpoints.L4));
        }

        public ParallelCommandGroup scoreCoralL4() {
            return new ParallelCommandGroup(
                    mWrist.setWristPIDCmd(WristSetpoints.SCORE),
                    new WaitCommand(0.1).andThen(new InstantCommand(() -> mClaw.setVolts(ClawConstants.ClawVolts.EJECT_CORAL.getClawVolts()))));
        }



}
