// REBELLION 10014

package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeConstants {

    public static int kFrontSensorDIOPort = 3;
    public static int kBackSensorDIOPort = 4;

    public static int kRollerID = 54;
    public static int kIndexerID = 55;

    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kCoast;

    public static SparkFlexConfig kIndexerConfig = new SparkFlexConfig();
    public static double kRollerCurrentLimit = 60;
    public static boolean kInverted = true;

   

    static {
        kIndexerConfig.idleMode(kIdleMode).smartCurrentLimit(60).inverted(kInverted);
    }

    public static class IntakePivot {

        public static int kPivotID = 53;
        public static MotorType kMotorType = MotorType.kBrushless;
        public static IdleMode kIdleMode = IdleMode.kBrake;
        public static SparkMaxConfig kPivotConfig = new SparkMaxConfig();
        public static boolean kEncoderInvert = true;

        public static double kPositionConversionFactor = 360.0;
        public static double kTolerance = 3;
        public static double kP = 0.2;
        public static double kI = 0.0;
        public static double kD = 0.001;
        public static double kMaxVelocity = 300; // Theoretical max: 1555
        public static double kMaxAcceleration = 500; // Theoretical max: 16179 deg/s^2
    
        public static Rotation2d kEncoderOffset = Rotation2d.fromRotations(0.3345553);
        public static Rotation2d kForwardSoftLimit = Rotation2d.fromDegrees(90.0); // TO DO: CONFIGURE ME!
        public static Rotation2d kReverseSoftLimit = Rotation2d.fromDegrees(2.0); // TO DO: CONFIGURE ME!
    
    
        public static double kS = 0.0; // 0.34
        public static double kG = 0.19; // 0.3
        public static double kV = 2.4; // 0.43
        public static double kA = 0.08; // 0.06

        static {
            kPivotConfig.idleMode(kIdleMode).smartCurrentLimit(60).inverted(false);
        }
    }
}
