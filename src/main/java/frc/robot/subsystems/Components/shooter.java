package frc.robot.subsystems.Components;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class shooter {
    SparkMax shooters;
    SparkMax elevator;
    double fixedSpeed;
    int mode = 0;
    double speed = .6;
    public shooter(){
        shooters= new SparkMax(2, MotorType.kBrushless);
        elevator= new SparkMax(13, MotorType.kBrushless);
        fixedSpeed=1;
    }

    public void toggle(){
        if(mode == 0){
            shooters.set(speed);
        }
        else if (mode == 1){
            elevator.set(-fixedSpeed);
        }
        else{
            stop();

        }
        mode = (mode + 1) % 3;
    }



    
    public void stop(){
        shooters.set(0);
        elevator.set(0);
    }
}
