package frc.robot.subsystems.Components;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class Belt {
    SparkMax motor1;
    int mode = 0;
    int reverseToggle = 0;
    public Belt(){
        motor1 = new SparkMax(8, MotorType.kBrushless);
    }

    public void toggle(){
        if (mode == 1){
            motor1.set(-.6);
        }
        else{
            stop();
        }
        mode = (mode + 1) % 3;
    }

    public void stop(){
        motor1.set(0);
    }

    public void setSequence(int num){
        mode = num;
    }

    public void reverse(){
        if(reverseToggle == 0){
            motor1.set(1);
            reverseToggle = 1;
        }
        else{
            stop();
            reverseToggle = 0;
        }
        
        mode = 0;
    }
}
