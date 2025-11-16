// REBELLION 10014

package frc.robot.subsystems.Claw;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
    private SparkFlex mClawMotor;
    private DigitalInput mBeamBrake;
    private Debouncer mBeamBreakIntakeDebounce = new Debouncer(0.2, DebounceType.kRising);
    private Debouncer mBeamBreakOuttakeDebounce = new Debouncer(0.5, DebounceType.kFalling);

    public ClawSubsystem() {
        mClawMotor = new SparkFlex(ClawConstants.kClawID, ClawConstants.kMotorType);
        mClawMotor.configure(ClawConstants.kClawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        mBeamBrake = new DigitalInput(ClawConstants.kBeamBrakeID);
    }

    private boolean beamBrakeHasPiece() {
        return !mBeamBrake.get();
    }

    private boolean beamBrakeHasPieceIntake() {
        return mBeamBreakIntakeDebounce.calculate(beamBrakeHasPiece());
    }

    private boolean beamBrakeHasPieceOuttake() {
        return mBeamBreakOuttakeDebounce.calculate(beamBrakeHasPiece());
    }

    private void setVolts(double pVolts) {
        mClawMotor.setVoltage(MathUtil.clamp(pVolts, -Constants.robotVoltage, Constants.robotVoltage));
    }

    public Command clawIntakeCoralCmd() {
        return new FunctionalCommand(
                () -> {
                    setVolts(ClawConstants.ClawIntakeVolts.INTAKE_CORAL.getIntakeVolts());
                },
                () -> {},
                (interrupted) -> {
                    setVolts(ClawConstants.ClawIntakeVolts.HOLD_CORAL.getIntakeVolts());
                },
                () -> beamBrakeHasPieceIntake(),
                this);
    }

    public Command clawOuttakeCoralCmd() {
        return new FunctionalCommand(
                () -> {
                    setVolts(ClawConstants.ClawIntakeVolts.OUTTAKE_REEF.getIntakeVolts());
                },
                () -> {},
                (interrupted) -> {
                    setVolts(0);
                },
                () -> !beamBrakeHasPieceOuttake(),
                this);
    }
}
