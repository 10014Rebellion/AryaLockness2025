package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Claw.ClawSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Wrist.WristSubsystem;

public class Bindings {
        private final IntakeSubsystem mIntake;
        private final ElevatorSubsystem mElevator;
        private final WristSubsystem mWrist;
        private final ClawSubsystem mClaw;
        private final ActionCommands mActionCmd;
    public class initDriverControls{
        
        // Controller
        private final CommandXboxController controller = new CommandXboxController(0);

        public initDriverControls(){
            mIntake = new IntakeSubsystem();
            mElevator = new ElevatorSubsystem();
            mWrist = new WristSubsystem();
            mClaw = new ClawSubsystem();
        }


    
    }
    
}
