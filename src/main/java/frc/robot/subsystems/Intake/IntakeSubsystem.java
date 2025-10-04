package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    private final TalonFX mIntakeRollerMotor;
    private final SparkFlex mIndexerMotor;
    private final DigitalInput mBackBeamBrake;
    private final DigitalInput mFrontBeamBrake;

    public IntakeSubsystem(){
        //Roller Motor
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
            rollerConfig.MotorOutput.Inverted =  IntakeConstants.kInverted 
                ? InvertedValue.CounterClockwise_Positive 
                : InvertedValue.Clockwise_Positive;

        //Indexer
            mIndexerMotor = new SparkFlex(IntakeConstants.kIndexerID, IntakeConstants.kMotorType);
            mIndexerMotor.configure(IntakeConstants.kIndexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        this.mBackBeamBrake = new DigitalInput(IntakeConstants.kBackSensorDIOPort);
        this.mFrontBeamBrake = new DigitalInput(IntakeConstants.kFrontSensorDIOPort);
        
    }

    public void setIntakeVolts(double pVolts){
        mIntakeRollerMotor.setVoltage(pVolts);
    }
    public void setIndexerVolts(double pVolts){
        mIndexerMotor.setVoltage(pVolts);
    }

    public boolean hasPieceInitial(){
        return !mBackBeamBrake.get();
    }
    public boolean hasPieceCompletely(){
        return !mFrontBeamBrake.get();
    }
    
    public Command intakeCmd(){
        return new FunctionalCommand(
            () -> {
                setIntakeVolts(8);
                setIndexerVolts(4);
            },

            ()->{
                if(hasPieceInitial()){
                    setIntakeVolts(4);
                    setIndexerVolts(2);
                }
            },

            (interrupted)->{
                setIntakeVolts(0);
                setIndexerVolts(0);
            },

            () -> hasPieceCompletely(),

            this);
    }
}
