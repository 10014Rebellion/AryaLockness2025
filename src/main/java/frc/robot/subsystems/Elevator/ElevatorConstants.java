package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class ElevatorConstants {
    public static final int kElevatorID = 41;

    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kCoast;
    public static boolean kInverted = true;
    public static int kSmartCurrentLimit = 60;
    public static SparkMaxConfig kElevatorConfig = new SparkMaxConfig();
    
    static {
        kElevatorConfig.idleMode(kIdleMode).smartCurrentLimit(60).inverted(kInverted);
    }

}

