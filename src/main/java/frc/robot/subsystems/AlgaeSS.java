package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSS extends SubsystemBase{
    public final SparkMax Algae = new SparkMax(12, MotorType.kBrushless);

    public void AlgaeForward(){
        Algae.set(0.5);
    }

    public void AlgaeReverse() {
        Algae.set(-0.5);
    }

    public void AlgaeStop() {
        Algae.set(0);
    }
}
