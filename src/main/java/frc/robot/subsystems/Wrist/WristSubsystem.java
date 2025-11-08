package frc.robot.subsystems.Wrist;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;

public class WristSubsystem {
    private SparkFlex mWristMotor;

    public WristSubsystem(){
        mWristMotor = new SparkFlex(WristConstants.kWristID, WristConstants.kMotorType);
        mWristMotor.configure(WristConstants.kWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void setVolts(double pVolts){
        mWristMotor.setVoltage(pVolts);
    }
}
