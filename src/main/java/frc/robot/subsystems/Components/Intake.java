package frc.robot.subsystems.Components;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class Intake {
    SparkMax IntakeWheels;
    int mode = 0;
    public Intake(){
        IntakeWheels = new SparkMax(4, MotorType.kBrushless);
    }

    public void toggle(){
        if (mode == 0){
            IntakeWheels.set(.6);
        }
        else{
            stop();
        }
        mode = (mode + 1) % 2;
    }

    public void stop(){
        IntakeWheels.set(0);
    }
}
