package frc.robot.subsystems.Components;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class Belt {
    SparkMax motor1;

    public Belt(){
        motor1 = new SparkMax(8, MotorType.kBrushless);
    }

    public void start(){
        motor1.set(.3);
        //idk if speed is correct
    }

    public void stop(){
        motor1.set(0);
    }
}
