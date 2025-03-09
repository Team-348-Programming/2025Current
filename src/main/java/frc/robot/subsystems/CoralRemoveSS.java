package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralRemoveSS extends SubsystemBase {

    public static final SparkMax coral = new SparkMax(11, MotorType.kBrushless);

    public void CoralSpin(double speed) {
        coral.set(speed);
    }

    public void CoralStop() {
        coral.set(0);
    }
}
