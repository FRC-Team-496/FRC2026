package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

public class LimelightAlignment extends Command{

    private Camera camera;
    private ArrayList<Double> pose;
    private ArrayList<Double> cameraTargets;
    double xDistance;
    double zDistance;

    public LimelightAlignment(Camera camera){
        this.camera = camera;
        addRequirements(camera.getDriveSubsystem());
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("initialized","true");
    }

    @Override
    public void execute() {
        camera.getBestTargetArea(camera.getBestYaw());
        xDistance = camera.getAreaDistance(.3,camera.getBestYaw()).get(0);
        zDistance = camera.getAreaDistance(.3, camera.getBestYaw()).get(1);
        camera.getDriveSubsystem().drive(xDistance,zDistance,0,false,.5);
        SmartDashboard.putString("drove","true");
    }

    @Override
    public boolean isFinished() {
        return (camera.getDetected_ID() !=1) || (camera.getResultantDistance(camera.getDistXFromTag(), camera.getDistZFromTag())==.3); 
    }
       
     /*@Override
    public void initialize() {
        cameraTargets = camera.chooseTargetPosition(1,1);
        pose = camera.alignTag(1,cameraTargets.get(0),cameraTargets.get(1),cameraTargets.get(2));
        
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Tag Id",camera.getDetected_ID());
        if (camera.getDetected_ID() == 1){
            SmartDashboard.putString("move to align: ","running");
            camera.getDriveSubsystem().drive(pose.get(0),pose.get(1),pose.get(2),false,.5);
            SmartDashboard.putString("Made it past drive","true");
        }
    }

    @Override
    public boolean isFinished() {
        return camera.getDetected_ID()!=1 || (camera.getDriveSubsystem().getPose().getX() == cameraTargets.get(1)) && (camera.getDriveSubsystem().getPose().getY()==cameraTargets.get(2)) ;  
    }*/



    
}
