package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.studica.frc.*;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;

public class NavX extends SubsystemBase{
    AHRS navX;
    SPI port;

    public NavX(){
        navX = new AHRS(NavXComType.kMXP_SPI); 
    }

    public AHRS gyro(){
        return navX;
    }
    
    public void putGyro(){
        SmartDashboard.putNumber("Raw Roll", navX.getRoll());
        SmartDashboard.putNumber("Raw Pitch", -navX.getPitch());
        SmartDashboard.putNumber("Raw Yaw", navX.getYaw());
        SmartDashboard.putNumber("GYRO DISTANCE", Math.abs(navX.getDisplacementX() +navX.getDisplacementY() +navX.getDisplacementZ()));
        SmartDashboard.putNumber("Yaw Test", getHeading().getDegrees());
    }

    public double rawYaw(){return navX.getYaw();}
    public double rawPitch(){return navX.getPitch();}
    public double rawRoll(){return navX.getRoll();}

    //wraps the yaw in the odometry system used for most of the other methods, makes it 2d
    public Rotation2d getHeading(){
        return new Rotation2d(navX.getYaw()); //need to convert yaw to radians according to geminai
    }

    public void reset(){
        navX.reset();
    }

    //returns the current rotation rate(angular velocity) in degrees per second
    public double getRate(){
        return navX.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }
}
