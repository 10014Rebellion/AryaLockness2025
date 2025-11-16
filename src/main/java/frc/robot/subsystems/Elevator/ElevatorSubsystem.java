// REBELLION 10014

package frc.robot.subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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

    private void setVolts(double pVolts) {
        pVolts = MathUtil.clamp(pVolts, -Constants.robotVoltage, Constants.robotVoltage);
        if (isOutOfBounds(pVolts)) mElevatorMotor.setVoltage(0);
        else mElevatorMotor.setVoltage(pVolts);
    }

    private Rotation2d getEncoderReading() {
        return new Rotation2d(mEncoder.getPosition() * 2 * Math.PI);
    }

    public boolean isPIDAtGoal() {
        return mPidController.atGoal();
    }

    public Command setElevatorPIDCmd(ElevatorConstants.ElevatorSetpoints pSetPoint) {
        return new FunctionalCommand(
                () -> {
                    mPidController.setGoal(pSetPoint.getDeg());
                    mPidController.reset(getEncoderReading().getDegrees());
                },
                () -> {
                    double pidCalculation =
                            mPidController.calculate(getEncoderReading().getDegrees());
                    double ffCalc = mElevatorFeedforward.calculate(mPidController.getSetpoint().velocity);
                    setVolts(pidCalculation + ffCalc);
                    SmartDashboard.putNumber("Elevator/ControllerOutput", pidCalculation + ffCalc);
                },
                (interrupted) -> {
                    setVolts(0);
                },
                () -> isPIDAtGoal(),
                this);
    }

    private boolean isOutOfBounds(double pInput) {
        return (pInput > 0 && getEncoderReading().getDegrees() >= ElevatorConstants.kForwardSoftLimit)
                || (pInput < 0 && getEncoderReading().getDegrees() <= ElevatorConstants.kReverseSoftLimit);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator/EncoderDeg", getEncoderReading().getDegrees());
        SmartDashboard.putNumber("Elevator/MotorOutput", mElevatorMotor.getAppliedOutput());

        if (isOutOfBounds(mElevatorMotor.getAppliedOutput())) {
            setVolts(0);
        }
    }
}
