package frc.robot.subsystems.Climb;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;

public class ClimbConstants {
    public static class PulleyConstants{
        public static int kMotorID = 61;
    
        public static MotorType kMotorType = MotorType.kBrushless;
        public static IdleMode kIdleMode = IdleMode.kBrake;
        public static boolean kInverted = true;
        public static SparkMaxConfig kConfig = new SparkMaxConfig();
    
        public static Rotation2d encoderZeroOffset = Rotation2d.fromRotations(0.6088358);
        public static double kTolerance = 3;


        static{
            kConfig.idleMode(kIdleMode).smartCurrentLimit(60).inverted(kInverted);
        }

        public enum PulleyVolts{
            GO(4),
            FAST(5.5),
            SLOW(1.1),
            STOP(0);

            public final double volts;
            private PulleyVolts(double pVolts){
                volts = pVolts;
            }

            public double getVolts(){
                return volts;
            }
            
        }

        public enum PulleySetPoint{
            EXTENDED(0.0),
            EXTENDED_PRE(10),
            STARTROLLING(28.0), // angle of the climb where it start intaking the cage
            SLOW_DOWN(70.0), 
            CLIMBED(95.0),
            STOWED(157.0);
            
            public final double setPoint;
            private PulleySetPoint(double pSetPoint){
                setPoint = pSetPoint;
            }

            public double getSetpoint(){
                return setPoint;
            }
            
        }


    }

    public static class GrabberConstants{  
        public static int kMotorID = 62;
        public static int kClimbBeamBreakID = 6;
        public static MotorType kMotorType = MotorType.kBrushless;

        public static IdleMode kIdleMode = IdleMode.kBrake;
        public static boolean kMotorInverted = true;
        public static SparkFlexConfig kFlexConfig = new SparkFlexConfig();

        static{
            kFlexConfig.idleMode(kIdleMode).smartCurrentLimit(60).inverted(kMotorInverted);
        }

    }

}
