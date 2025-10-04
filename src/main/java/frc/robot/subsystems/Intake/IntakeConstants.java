package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

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
}
