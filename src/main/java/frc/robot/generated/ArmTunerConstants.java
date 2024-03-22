package frc.robot.generated;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class ArmTunerConstants {

    // TODO: Tune motionmagic and pid vals later
    public static final MotionMagicConfigs pivotMotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicAcceleration(2)
            .withMotionMagicCruiseVelocity(1);

    public static final Slot0Configs pivotPIDConfigs = new Slot0Configs()
            .withKP(64)
            .withKD(0)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withKG(0.3);

    // tune later for shooter motors to reach target speeds
    public static final Slot0Configs shooterPIDConfigs = new Slot0Configs()
            .withKS(0.3)
            .withKV(0.125);
}
