package frc.robot.subsystems.Components;

import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.spark.SparkMax;

public class shooter {
    SparkMax shooters;
    SparkMax elevator;
    double fixedSpeed;
    public shooter(){
        shooters= new SparkMax(67,null);
        elevator= new SparkMax(67, null);
        fixedSpeed=.3;
    //idk if the motor type is correct and the number needs to be changed for both
    }

    public void start(double speed){
        shooters.set(speed);
        elevator.set(fixedSpeed);
        //idk if speed is correct
    }

    public void stop(){
        shooters.set(0);
        elevator.set(0);
    }
}
