// REBELLION 10014

package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.IntakeConstants.IntakePivot;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX mIntakeRollerMotor;
    private final SparkFlex mIndexerMotor;
    private final SparkMax mPivotMotor;
    private final DigitalInput mBackBeamBrake;
    private final DigitalInput mFrontBeamBrake;
    private final AbsoluteEncoder mEncoder;
    private ProfiledPIDController mPIDController;
    private ArmFeedforward mFeedforward;

    public IntakeSubsystem() {
        // Pivot Motor
        mPivotMotor = new SparkMax(IntakePivot.kPivotID, IntakePivot.kMotorType);
        mPivotMotor.configure(IntakePivot.kPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.mEncoder = mPivotMotor.getAbsoluteEncoder();

        // Roller Motor
        this.mIntakeRollerMotor = new TalonFX(IntakeConstants.kRollerID, "drivetrain");
        mIntakeRollerMotor.getConfigurator().apply(new TalonFXConfiguration());
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        
        // Apply configurations
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.kRollerCurrentLimit;
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.kRollerCurrentLimit;
        rollerConfig.Voltage.PeakForwardVoltage = 12;
        rollerConfig.Voltage.PeakReverseVoltage = -12;

        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rollerConfig.MotorOutput.Inverted =
                IntakeConstants.kInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;

        // Indexer
        mIndexerMotor = new SparkFlex(IntakeConstants.kIndexerID, IntakeConstants.kMotorType);
        mIndexerMotor.configure(
                IntakeConstants.kIndexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Beam Brakes
        this.mBackBeamBrake = new DigitalInput(IntakeConstants.kBackSensorDIOPort);
        this.mFrontBeamBrake = new DigitalInput(IntakeConstants.kFrontSensorDIOPort);

        //PID
        mPIDController = new ProfiledPIDController(IntakeConstants.IntakePivot.kP, 0, IntakeConstants.IntakePivot.kD, new Constraints(IntakeConstants.IntakePivot.kMaxVelocity, IntakeConstants.IntakePivot.kMaxAcceleration));
        mFeedforward = new ArmFeedforward(IntakeConstants.IntakePivot.kS, IntakeConstants.IntakePivot.kG, IntakeConstants.IntakePivot.kV, IntakeConstants.IntakePivot.kA);
    }

    public void setPivotVolts(double pVolts) {
        mPivotMotor.setVoltage(pVolts);
    }

    public void setIntakeVolts(double pVolts) {
        mIntakeRollerMotor.setVoltage(pVolts);
    }

    public void setIndexerVolts(double pVolts) {
        mIndexerMotor.setVoltage(pVolts);
    }

    public boolean hasPieceInitial() {
        return !mBackBeamBrake.get();
    }

    public boolean hasPieceCompletely() {
        return !mFrontBeamBrake.get();
    }

    public Command setIntakePivotUpDownCmd(double pVolts) { //for test purposes

        return new FunctionalCommand(
                () -> {
                    setPivotVolts(pVolts);
                },
                () -> {},
                (interrupted) -> {
                    setPivotVolts(pVolts);
                },
                () -> false,
                this);
    }


    public Command intakePIDCmd(double pSetPoint){
        return new FunctionalCommand(
            () -> {
                mPIDController.setGoal(pSetPoint);
                mPIDController.reset(getEncoderReading().getDegrees());
            }, 

            () -> {
                double calculation = mPIDController.calculate(getEncoderReading().getDegrees());
                double ffCalc = mFeedforward.calculate(
                    Units.degreesToRadians(mPIDController.getSetpoint().position),
                    Units.degreesToRadians(mPIDController.getSetpoint().velocity));
                setPivotVolts(ffCalc+calculation);
            }, 

            (interrupted)-> {
                setPivotVolts(0);
            },

            () -> false,

            this);
    }

    public Command intakeCmd() {

        return new FunctionalCommand(
                () -> {
                    setIntakeVolts(8);
                    setIndexerVolts(4);
                },
                () -> {
                    if (hasPieceInitial()) {
                        setIntakeVolts(4);
                        setIndexerVolts(2);
                    }
                },
                (interrupted) -> {
                    setIntakeVolts(0);
                    setIndexerVolts(0);
                },
                () -> hasPieceCompletely(),
                this);
    }

    public Rotation2d getEncoderReading() {
        return Rotation2d.fromRotations(mEncoder.getPosition())
        .minus(IntakeConstants.IntakePivot.kEncoderOffset)
        .times(IntakeConstants.IntakePivot.kEncoderInvert ? -1 : 1);
    }

    private boolean isOutOfBoundsIntakePivot(double pInput) {
        return ((pInput > 0 && getEncoderReading().getDegrees() >= IntakePivot.kForwardSoftLimit.getDegrees())
                || (pInput < 0 && getEncoderReading().getDegrees() <= IntakePivot.kReverseSoftLimit.getDegrees()));
    }

    public void stopIfLimitIntakePivot() {
        if (isOutOfBoundsIntakePivot(getMotorOutput())) {
            setPivotVolts(0);
        }
    }

    public double getMotorOutput() {
        return mPivotMotor.getAppliedOutput();
    }

    @Override
    public void periodic() {
        stopIfLimitIntakePivot();
        SmartDashboard.putNumber("Intake/Encoder Reading Degrees", getEncoderReading().getDegrees());
    }
}
