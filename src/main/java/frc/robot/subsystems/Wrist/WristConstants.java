package frc.robot.subsystems.Wrist;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class WristConstants {
    public static int kWristID = 42;
    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kCoast;
    public static boolean kInverted = true;
    public static int kSmartCurrentLimit = 60;
    public static SparkFlexConfig kWristConfig = new SparkFlexConfig();

    static {
        kWristConfig.idleMode(kIdleMode).smartCurrentLimit(60).inverted(kInverted);
    }
}
