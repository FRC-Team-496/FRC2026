package frc.robot.subsystems.Components;

import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.spark.SparkMax;

public class Belt {
    SparkMax motor1;

    public Belt(){
        motor1= new SparkMax(67,null);
    //idk if the motor type is correct and the number needs to be changed for both
    }

    public void start(){
        motor1.set(.3);
        //idk if speed is correct
    }

    public void stop(){
        motor1.set(0);
    }
}
