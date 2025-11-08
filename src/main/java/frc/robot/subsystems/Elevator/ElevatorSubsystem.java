package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class ElevatorSubsystem {
    private SparkMax mElevatorMotor;

    public ElevatorSubsystem(){
        mElevatorMotor = new SparkMax(ElevatorConstants.kElevatorID, ElevatorConstants.kMotorType);
        mElevatorMotor.configure(ElevatorConstants.kElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);    
    }

    private void setVolts(double pVolts){
        mElevatorMotor.setVoltage(pVolts);
    }
}
