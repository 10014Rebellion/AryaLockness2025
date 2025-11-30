// REBELLION 10014

package frc.robot.subsystems.Elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorConstants {
    public static final int kElevatorID = 41;

    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kBrake;
    public static boolean kInverted = true;
    public static int kSmartCurrentLimit = 50;
    public static SparkMaxConfig kElevatorConfig = new SparkMaxConfig();

    public static double kP = 1.0;
    public static double kD = 0.0;
    public static double kMaxAcceleration = 950; // 250;
    public static double kMaxVelocity = 2400; // 500;
    public static double kTolerance = 1;
    public static double kS = 0.0;
    public static double kG = 0.6;
    public static double kV = 0.0; // 4.49;
    public static double kA = 0.0; // 0.2;
    public static double kForwardSoftLimit = 55;
    public static double kReverseSoftLimit = 0;
    public static double kPositionConversionFactor = 1.21875; // 1.0 / kDrumCircumference

    static {
        kElevatorConfig
                .idleMode(kIdleMode)
                .smartCurrentLimit(kSmartCurrentLimit)
                .inverted(kInverted);
    }

    public enum ElevatorSetpoints {
        BOTTOM(0),
        L1(27),
        L2(7),
        L3(25.0),
        L4(48),
        PREINTAKE(25.0),
        POSTINTAKE(15.6);

        private final double mSetpointInches;

        private ElevatorSetpoints(double pSetpointInches) {
            this.mSetpointInches = pSetpointInches;
        }

        public double getInches() {
            return this.mSetpointInches;
        }
    }
}
