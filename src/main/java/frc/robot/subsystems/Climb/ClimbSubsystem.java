package frc.robot.subsystems.Climb;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Climb.ClimbConstants.PulleyConstants;
import frc.robot.subsystems.Climb.ClimbConstants.PulleyConstants.PulleySetPoint;

public class ClimbSubsystem extends SubsystemBase{
    private SparkMax mPulleyMotor;
    private SparkFlex mGrabberMotor;
    private AbsoluteEncoder mEncoder;

    public ClimbSubsystem(){
        mPulleyMotor = new SparkMax(ClimbConstants.PulleyConstants.kMotorID, ClimbConstants.PulleyConstants.kMotorType);
        mGrabberMotor = new SparkFlex(ClimbConstants.GrabberConstants.kMotorID, ClimbConstants.GrabberConstants.kMotorType);
        mEncoder = mPulleyMotor.getAbsoluteEncoder();
    }

    public void setPulleyVolts(ClimbConstants.PulleyConstants.PulleyVolts pulleyVolts){
        mPulleyMotor.setVoltage(pulleyVolts.getVolts());
    }

    public void setGrabberVolts(double pVolts){
        mGrabberMotor.setVoltage(pVolts);
    }


    public Rotation2d getEncoderReading(){
        return Rotation2d.fromRotations(mEncoder.getPosition()).minus(ClimbConstants.PulleyConstants.encoderZeroOffset);
    }

    public Command initialClimb(){
        return new FunctionalCommand(
            () -> {}, 
            
            () -> {
                if(getEncoderReading().getDegrees() < ClimbConstants.PulleyConstants.PulleySetPoint.STARTROLLING.getSetpoint()){
                    setGrabberVolts(2);
                }
                
                if(getEncoderReading().getDegrees() <= ClimbConstants.PulleyConstants.PulleySetPoint.SLOW_DOWN.getSetpoint()){
                    setPulleyVolts(ClimbConstants.PulleyConstants.PulleyVolts.SLOW);
                }

                if(getEncoderReading().getDegrees() > ClimbConstants.PulleyConstants.PulleySetPoint.SLOW_DOWN.getSetpoint()){
                    setPulleyVolts(ClimbConstants.PulleyConstants.PulleyVolts.GO);
                }

                if(getEncoderReading().getDegrees() <= ClimbConstants.PulleyConstants.PulleySetPoint.SLOW_DOWN.getSetpoint()){
                    setPulleyVolts(ClimbConstants.PulleyConstants.PulleyVolts.STOP);
                }

            }, 
            
            (interrupted) -> {
                setPulleyVolts(ClimbConstants.PulleyConstants.PulleyVolts.STOP);
                setGrabberVolts(2);
            }, 

            () -> getEncoderReading().getDegrees() <= ClimbConstants.PulleyConstants.PulleySetPoint.SLOW_DOWN.getSetpoint(), 
            
            this);
    }

    public boolean withinTolerance(){
        return Math.abs(getEncoderReading().getDegrees() - PulleyConstants.PulleySetPoint.CLIMBED.getSetpoint()) < ClimbConstants.PulleyConstants.kTolerance;
    }

    public Command pullClimb(){
        return new FunctionalCommand(
        ()->{},

        () -> {
            if(getEncoderReading().getDegrees() < PulleyConstants.PulleySetPoint.CLIMBED.getSetpoint()){
                setPulleyVolts(ClimbConstants.PulleyConstants.PulleyVolts.FAST);
            }
        },

        (interrupted) -> {
            setPulleyVolts(ClimbConstants.PulleyConstants.PulleyVolts.STOP);
        },

        () -> withinTolerance(),

        this);
    }
    
}
