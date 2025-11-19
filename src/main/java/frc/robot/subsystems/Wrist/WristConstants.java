// REBELLION 10014

package frc.robot.subsystems.Wrist;

import static edu.wpi.first.units.Units.Rotation;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;

import com.revrobotics.spark.config.SparkFlexConfig;

public class WristConstants {
    public static int kWristID = 42;

    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kCoast;

    public static boolean kInverted = true;
    public static boolean kInvertedEncoder = true;
    public static int kSmartCurrentLimit = 60;

    public static SparkFlexConfig kWristConfig = new SparkFlexConfig();

    public static Rotation2d kForwardSoftLimit = Rotation2d.fromDegrees(90);
    public static Rotation2d kReverseSoftLimit = Rotation2d.fromDegrees(-90);
    public static Rotation2d kArmEncoderOffset = Rotation2d.fromRotations(0.545);

    public static double kP = 0.18;
    public static double kD = 0.0;

    public static double kMaxAcceleration = 500; // was 500. k0inda slow
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
        TEST_90(80),
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
