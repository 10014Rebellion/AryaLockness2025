// REBELLION 10014

package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

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

    public static double kPositionConversionFactor = 360.0;

    static {
        kIndexerConfig.idleMode(kIdleMode).smartCurrentLimit(60).inverted(kInverted);
    }

    public static class IntakePivot {

        public static int kPivotID = 53;
        public static MotorType kMotorType = MotorType.kBrushless;
        public static IdleMode kIdleMode = IdleMode.kBrake;
        public static boolean kEncoderInverted = true;
        public static double kPositionConversionFactor = 360.0;
        public static double kEncoderOffsetRev = 0.3345553;
        public static SparkMaxConfig kPivotConfig = new SparkMaxConfig();

        public static double kForwardSoftLimit = 90.0;
        public static double kReverseSoftLimit = 2.0;

        static {
            kPivotConfig.idleMode(kIdleMode).smartCurrentLimit(60).inverted(false);
        }
    }
}
