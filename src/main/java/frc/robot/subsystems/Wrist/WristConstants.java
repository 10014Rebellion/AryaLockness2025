// REBELLION 10014

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

    public static double kForwardSoftLimit = 90;
    public static double kReverseSoftLimit = -89;
    public static double kPositionConversionFactor = 360.0;

    public static double kP = 0.22;
    public static double kD = 0.01;

    public static double kMaxAcceleration = 1000; // was 500. k0inda slow
    public static double kMaxVelocity = 700;
    public static double kS = 0.02;
    public static double kG = 0.4;
    public static double kV = 0.003;
    public static double kA = 0.0;
    public static double kTolerance = 2.3;

    static {
        kWristConfig
                .idleMode(kIdleMode)
                .smartCurrentLimit(WristConstants.kSmartCurrentLimit)
                .inverted(kInverted);
    }

    public enum WristSetpoints {
        BOTTOM(0),
        L1(-30),
        L2(46 + 6),
        L3(46 + 6),
        L4(65),
        SCORE(11);

        private final double mSetpoint;

        private WristSetpoints(double pSetPoint) {
            this.mSetpoint = pSetPoint;
        }

        public double getDeg() {
            return this.mSetpoint;
        }
    };
}
