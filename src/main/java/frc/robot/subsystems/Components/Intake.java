package frc.robot.subsystems.Components;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {
    SparkMax IntakeWheels;
    int mode = 0;
    public Intake(){
        IntakeWheels = new SparkMax(4, MotorType.kBrushless);
    }

    public void toggle(){
        if (mode == 0){
            IntakeWheels.set(.8);
            SmartDashboard.putBoolean("Intake On", true);

        }
        else{
            stop();
            SmartDashboard.putBoolean("Intake On", false);
        }
        mode = (mode + 1) % 2;
    }

    public void stop(){
        IntakeWheels.set(0);
    }
}
