package frc.robot.subsystems.Wrist;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;

public class WristSubsystem extends SubsystemBase{
    private SparkFlex mWristMotor;
    private ProfiledPIDController mPidController;
    private ArmFeedforward mArmFeedforward;
    private AbsoluteEncoder mEncoder;

    public WristSubsystem(){
        mWristMotor = new SparkFlex(WristConstants.kWristID, WristConstants.kMotorType);
        mWristMotor.configure(WristConstants.kWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
        mPidController = new ProfiledPIDController(WristConstants.kP, WristConstants.kI, WristConstants.kD, new Constraints(WristConstants.kMaxVelocity, WristConstants.kMaxAcceleration));
        mPidController.setTolerance(WristConstants.kTolerance);
        mArmFeedforward = new ArmFeedforward(WristConstants.kS, WristConstants.kG, WristConstants.kV, WristConstants.kA);

        mEncoder = mWristMotor.getAbsoluteEncoder();
        

    }

    private void setVolts(double pVolts){
        if(isOutOfBounds(pVolts))
            mWristMotor.setVoltage(0);
        else
            mWristMotor.setVoltage(pVolts);
    }

    private double getEncoderReading(){
        return mEncoder.getPosition() * WristConstants.kPositionConversionFactor;
    }

    private boolean isPIDAtGoal() {
        return mPidController.atGoal();
    }

    private boolean isOutOfBounds(double pInput) {
        return (pInput > 0 && getEncoderReading() >= WristConstants.kForwardSoftLimit)
            || (pInput < 0 && getEncoderReading() <= WristConstants.kReverseSoftLimit);
    }

    public Command setWristPIDCmd(WristConstants.WristSetPoints pSetpoint){
        return new FunctionalCommand(
            () -> {
                mPidController.setGoal(pSetpoint.getPos());
                mPidController.reset(mEncoder.getPosition());
            }, 

            () -> {
                double calculation = mPidController.calculate(getEncoderReading());
                double ffCalc = mArmFeedforward.calculate(
                    Units.degreesToRadians(mPidController.getSetpoint().position),
                    Units.degreesToRadians(mPidController.getSetpoint().velocity)
                    );
                setVolts(calculation + ffCalc);
            }, 

            (interrupted) -> {
                setVolts(0);
            }, 

            () -> isPIDAtGoal(), 

            this);
    }

    


}
