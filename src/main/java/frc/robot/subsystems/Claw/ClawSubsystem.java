package frc.robot.subsystems.Claw;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkFlex;

public class ClawSubsystem extends SubsystemBase{
    private SparkFlex mClawMotor;
    private DigitalInput mBeamBrake;

    public ClawSubsystem(){
        mClawMotor = new SparkFlex(ClawConstants.kClawID, ClawConstants.kMotorType);
        mClawMotor.configure(ClawConstants.kClawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        mBeamBrake = new DigitalInput(ClawConstants.kBeamBrakeID);

    }

    private boolean getBeamBrake(){
        return !mBeamBrake.get();
    }

    private void setVolts(double pVolts){
        mClawMotor.setVoltage(pVolts);
    }

    public Command clawIntakeCoralCmd(){
        return new FunctionalCommand(
            ()->{
                setVolts(6);
            }, 
            ()->{}, 
            (interrupted) -> {
                setVolts(0.25);
            }, 
            ()-> getBeamBrake(), 
            this);
    }

    public Command clawOuttakeCoralCmd(){
        return new FunctionalCommand(
            ()->{
                setVolts(-0.6);
            }, 
            ()->{}, 
            (interrupted) -> {
                setVolts(0);
            }, 
            ()-> false, 
            this);
    }


}
