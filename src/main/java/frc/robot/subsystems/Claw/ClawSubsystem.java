package frc.robot.subsystems.Claw;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;

public class ClawSubsystem {
    private SparkFlex mClawMotor;

    public ClawSubsystem(){
        mClawMotor = new SparkFlex(ClawConstants.kClawID, ClawConstants.kMotorType);
        mClawMotor.configure(ClawConstants.kClawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void setVolts(double pVolts){
        mClawMotor.setVoltage(pVolts);
    }
}
