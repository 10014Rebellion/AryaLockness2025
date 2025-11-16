// REBELLION 10014

package frc.robot.subsystems.Claw;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class ClawConstants {
    public static int kClawID = 42;
    public static MotorType kMotorType = MotorType.kBrushless;
    public static IdleMode kIdleMode = IdleMode.kCoast;
    public static boolean kInverted = true;
    public static int kSmartCurrentLimit = 60;
    public static SparkFlexConfig kClawConfig = new SparkFlexConfig();
    public static int kBeamBrakeID = 55;

    static {
        kClawConfig.idleMode(kIdleMode).smartCurrentLimit(60).inverted(kInverted);
    }

    public enum ClawIntakeVolts {
        INTAKE_CORAL(6.0),
        HOLD_CORAL(0.25),
        OUTTAKE_REEF(-0.6),
        OUTTAKE_L1(-3),
        EJECT_CORAL(-3);

        private double intakeVolts;

        private ClawIntakeVolts(double pVolts) {
            this.intakeVolts = pVolts;
        }

        public double getIntakeVolts() {
            return intakeVolts;
        }
    };
}
