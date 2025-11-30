// REBELLION 10014

package frc.robot.subsystems.Claw;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
    private SparkFlex mClawMotor;
    private CANrange mCANrange;
    private DigitalInput mBeamBrake;
    private Debouncer mBeamBreakIntakeDebounce = new Debouncer(0.2, DebounceType.kRising);
    private Debouncer mBeamBreakOuttakeDebounce = new Debouncer(0.5, DebounceType.kFalling);
    private CANrangeConfiguration canRangeConfig = new CANrangeConfiguration();
 

    public ClawSubsystem() {
        mCANrange = new CANrange(ClawConstants.kCANRangeID, "drivetrain");
        mClawMotor = new SparkFlex(ClawConstants.kClawID, ClawConstants.kMotorType);
        mClawMotor.configure(ClawConstants.kClawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        mBeamBrake = new DigitalInput(ClawConstants.kBeamBrakeID);

        canRangeConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 3000;
        canRangeConfig.ProximityParams.ProximityHysteresis = 0.01;
        canRangeConfig.ProximityParams.ProximityThreshold = 0.5;

        mCANrange.getConfigurator().apply(canRangeConfig);
    }

    private boolean beamBrakeHasPiece() {
        return !mBeamBrake.get();
    }
    private boolean canRangeHasPiece(){
        return (mCANrange.getDistance().getValueAsDouble() < 0.06) && (mCANrange.getSignalStrength().getValueAsDouble() > 8000) && (mCANrange.getAmbientSignal().getValueAsDouble() < 20);
    }

    private boolean beamBrakeHasPieceIntake() {
        return mBeamBreakIntakeDebounce.calculate(beamBrakeHasPiece());
    }

    private boolean beamBrakeHasPieceOuttake() {
        return mBeamBreakOuttakeDebounce.calculate(beamBrakeHasPiece());
    }

    public void setVolts(double pVolts) {
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
                () -> canRangeHasPiece(),
                this);
    }

    public Command clawOuttakeCoralCmd() {
        return new FunctionalCommand(
                () -> {
                    setVolts(ClawConstants.ClawIntakeVolts.OUTTAKE_REEF.getIntakeVolts());
                },
                () -> {
                    setVolts(ClawConstants.ClawIntakeVolts.OUTTAKE_REEF.getIntakeVolts());
                },
                (interrupted) -> {
                    setVolts(0);
                },
                () -> !canRangeHasPiece(),
                this);
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Claw/hasCoral", canRangeHasPiece());
        SmartDashboard.putNumber("Claw/distance", mCANrange.getDistance().getValueAsDouble());
        SmartDashboard.putNumber("Claw/signalStrength", mCANrange.getSignalStrength().getValueAsDouble());
        SmartDashboard.putNumber("Claw/ambience", mCANrange.getAmbientSignal().getValueAsDouble());
    }

}
