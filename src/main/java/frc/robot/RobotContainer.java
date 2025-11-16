// REBELLION 10014

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Claw.ClawSubsystem;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Wrist.WristConstants;
import frc.robot.subsystems.Wrist.WristSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final IntakeSubsystem mIntake;
    private final ElevatorSubsystem mElevator;
    private final WristSubsystem mWrist;
    private final ClawSubsystem mClaw;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        mWrist = new WristSubsystem();
        mIntake = new IntakeSubsystem();
        mElevator = new ElevatorSubsystem();
        mClaw = new ClawSubsystem();

        configureButtonBindings();
        configureIntakeBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {}

    private void configureIntakeBindings() {
        controller.rightTrigger().whileTrue(mElevator.setElevatorPIDCmd(ElevatorConstants.ElevatorSetpoints.L1));
        controller.leftTrigger().whileTrue(mElevator.setElevatorPIDCmd(ElevatorConstants.ElevatorSetpoints.BOTTOM));
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
