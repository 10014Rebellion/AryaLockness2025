package frc.robot.subsystems.Elevator;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{
    private SparkMax mElevatorMotor;
    private ProfiledPIDController mPidController;
    private ElevatorFeedforward mElevatorFeedforward;
    private AbsoluteEncoder mEncoder;

    public ElevatorSubsystem(){
        mPidController = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, new Constraints(ElevatorConstants.kMaxAcceleration, ElevatorConstants.kMaxVelocity));
        mPidController.setTolerance(ElevatorConstants.kTolerance);
        mElevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);

        mElevatorMotor = new SparkMax(ElevatorConstants.kElevatorID, ElevatorConstants.kMotorType);
        mElevatorMotor.configure(ElevatorConstants.kElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);    
        
        mEncoder = mElevatorMotor.getAbsoluteEncoder();
    }

    private void setVolts(double pVolts){
        if(isOutOfBounds(pVolts))
            mElevatorMotor.setVoltage(0);
        else    
            mElevatorMotor.setVoltage(pVolts);
    }

    private double getEncoderReading(){
        return mEncoder.getPosition() * ElevatorConstants.kPositionConversionFactor;
    }

    public boolean isPIDAtGoal() {
        return mPidController.atGoal();
      }

    public Command upOrDown(double pVolts){
        return new FunctionalCommand(()->{setVolts(pVolts);}, ()->{}, (interrupted)->{setVolts(0);}, ()->false, this);
    }


    public Command setPIDCmd(ElevatorConstants.setPoints pSetPoint){
        return new FunctionalCommand(
            () -> {
                mPidController.setGoal(pSetPoint.getPos());
                mPidController.reset(mEncoder.getPosition());
            }, 
            () -> {
                double pidCalculation = mPidController.calculate(getEncoderReading());
                double ffCalc = mElevatorFeedforward.calculate(mPidController.getSetpoint().velocity);
                setVolts(pidCalculation + ffCalc);
                SmartDashboard.putNumber("Elevator/ControllerOutput", pidCalculation+ffCalc);
            }, 
           (interrupted) -> {
                setVolts(0);
           }, 
            () -> isPIDAtGoal(), 
            this);
    }


    private boolean isOutOfBounds(double pInput) {
        return (pInput > 0 && getEncoderReading() >= ElevatorConstants.kForwardSoftLimit)
            || (pInput < 0 && getEncoderReading() <= ElevatorConstants.kReverseSoftLimit);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Elevator/Encoder", getEncoderReading());
        SmartDashboard.putNumber("Elevator/MotorOutput", mElevatorMotor.getAppliedOutput());
    }

    
}
