// REBELLION 10014

package frc.robot.subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    private SparkMax mElevatorMotor;
    private ProfiledPIDController mPidController;
    private ElevatorFeedforward mElevatorFeedforward;
    private RelativeEncoder mEncoder;

    public ElevatorSubsystem() {
        mPidController = new ProfiledPIDController(
                ElevatorConstants.kP,
                0,
                ElevatorConstants.kD,
                new Constraints(ElevatorConstants.kMaxAcceleration, ElevatorConstants.kMaxVelocity));
        mPidController.setTolerance(ElevatorConstants.kTolerance);
        mElevatorFeedforward =
                new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);

        mElevatorMotor = new SparkMax(ElevatorConstants.kElevatorID, ElevatorConstants.kMotorType);
        mElevatorMotor.configure(
                ElevatorConstants.kElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        mEncoder = mElevatorMotor.getEncoder();
    }

    public void setVolts(double pVolts) {
        pVolts = MathUtil.clamp(pVolts, -Constants.robotVoltage, Constants.robotVoltage);
        if (isOutOfBounds(pVolts)) mElevatorMotor.setVoltage(0);
        else mElevatorMotor.setVoltage(pVolts);
    }

    private double getEncoderInches() {
        return mEncoder.getPosition() * ElevatorConstants.kPositionConversionFactor;
    }

    public boolean isPIDAtGoal() {
        return mPidController.atGoal();
    }

    public Command elevatorVoltCmd(double pVolts){
        return new FunctionalCommand(
            ()->{
                setVolts(pVolts);
            },
            ()->{},
            (interrupted)->{
                setVolts(mElevatorFeedforward.calculate(mPidController.getSetpoint().velocity));
            },
            ()->false,
            this);
    }

    public Command setElevatorPIDCmd(ElevatorConstants.ElevatorSetpoints pSetPoint) {
        return new FunctionalCommand(
                () -> {
                    mPidController.setGoal(pSetPoint.getInches());
                    mPidController.reset(getEncoderInches());
                },
                () -> {
                    double pidCalculation = mPidController.calculate(getEncoderInches());
                    double ffCalc = mElevatorFeedforward.calculate(mPidController.getSetpoint().velocity);
                    setVolts(pidCalculation + ffCalc);
                    SmartDashboard.putNumber("Elevator/ControllerOutput", pidCalculation + ffCalc);
                },
                (interrupted) -> {
                    setVolts(mElevatorFeedforward.calculate(mPidController.getSetpoint().velocity));
                },
                () -> isPIDAtGoal(),
                this);
    }

    private boolean isOutOfBounds(double pInput) {
        return (pInput > 0 && getEncoderInches() >= ElevatorConstants.kForwardSoftLimit)
                || (pInput < 0 && getEncoderInches() <= ElevatorConstants.kReverseSoftLimit);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator/EncoderDeg", getEncoderInches());
        SmartDashboard.putNumber("Elevator/MotorOutput", mElevatorMotor.getAppliedOutput());
        SmartDashboard.putBoolean("Elevator/isGoal", isPIDAtGoal());

        if (isOutOfBounds(mElevatorMotor.getAppliedOutput())) {
            setVolts(0);
        }
    }
}
