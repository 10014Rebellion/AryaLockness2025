// REBELLION 10014

package frc.robot.subsystems.Wrist;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
    private final SparkFlex mWristMotor;
    private final ProfiledPIDController mPidController;
    private final ArmFeedforward mArmFeedforward;
    private final DutyCycleEncoder mEncoder;

    public WristSubsystem() {
        mWristMotor = new SparkFlex(WristConstants.kWristID, WristConstants.kMotorType);
        mWristMotor.configure(
                WristConstants.kWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        mPidController = new ProfiledPIDController(
                WristConstants.kP,
                0,
                WristConstants.kD,
                new Constraints(WristConstants.kMaxVelocity, WristConstants.kMaxAcceleration));
        mPidController.setTolerance(WristConstants.kTolerance);

        mArmFeedforward =
                new ArmFeedforward(WristConstants.kS, WristConstants.kG, WristConstants.kV, WristConstants.kA);

        mEncoder = new DutyCycleEncoder(1);
    }

    public void setVolts(double pVolts) {
        pVolts = MathUtil.clamp(pVolts, -Constants.robotVoltage, Constants.robotVoltage);

        if (isOutOfBounds(pVolts)) mWristMotor.setVoltage(0);
        else mWristMotor.setVoltage(pVolts);
    }

    private Rotation2d getEncoderReading() {
        return Rotation2d.fromRotations(mEncoder.get())
                .minus(WristConstants.kArmEncoderOffset)
                .times(WristConstants.kInvertedEncoder ? -1 : 1);
    }

    private boolean isPIDAtGoal() {
        return mPidController.atGoal();
    }

    private boolean isOutOfBounds(double pInput) {
        return (pInput > 0 && getEncoderReading().getDegrees() >= WristConstants.kForwardSoftLimit.getDegrees())
                || (pInput < 0 && getEncoderReading().getDegrees() <= WristConstants.kReverseSoftLimit.getDegrees());
    }

    public Command setWristPIDCmd(WristConstants.WristSetpoints pSetpointDeg) {
        return new FunctionalCommand(
                () -> {
                    mPidController.setGoal(pSetpointDeg.getDeg());
                    mPidController.reset(getEncoderReading().getDegrees());
                },
                () -> {
                    double calculation =
                            mPidController.calculate(getEncoderReading().getDegrees());
                    double ffCalc = mArmFeedforward.calculate(
                            Units.degreesToRadians(mPidController.getSetpoint().position),
                            Units.degreesToRadians(mPidController.getSetpoint().velocity));
                    setVolts(calculation + ffCalc);
                },
                (interrupted) -> {
                    setVolts(0);
                },
                () -> false,
                this);
    }

    @Override
    public void periodic() {
        if (isOutOfBounds(mWristMotor.getAppliedOutput())) {
            setVolts(0);
        }
        SmartDashboard.putNumber("Wrist/EncoderReadingDeg", getEncoderReading().getDegrees());
    }
}
