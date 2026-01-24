package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;

public class Camera extends SubsystemBase {
    private static double statX;
    private static double statYaw;
    private static double statY;
    private static int detected_ID;

    private static int cameraPipelineID = 0;
    private static int neuralNetworkpipelineId;

    private static double[] tagToCamera;

    public void startCamera() {
        NetworkTable armTable = NetworkTableInstance.getDefault().getTable("limelight-algae");
        NetworkTableEntry tx = armTable.getEntry("tx");
        NetworkTableEntry ty = armTable.getEntry("ty");
        NetworkTableEntry ta = armTable.getEntry("ta");
        NetworkTableEntry botpose = armTable.getEntry("botpose");
        NetworkTableEntry pipeline = armTable.getEntry("pipeline");
        int aprilTagID =  (int) armTable.getEntry("tid").getDouble(-1);
        detected_ID = aprilTagID;

        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
        
        SmartDashboard.putNumber("tag ID", aprilTagID);
        //read values periodically
        double x = tx.getDouble(0.0);
        statX = x;
        double y = ty.getDouble(0.0);
        statY = y;
        double area = ta.getDouble(0.0);
        double[] bot = botpose.getDoubleArray(new double[6]);
        statYaw = bot[5];



        double[] tagToCamera = armTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);

        SmartDashboard.putNumber("tx Tag", tagToCamera[0]);
        SmartDashboard.putNumber("ty Tag", tagToCamera[1]);
        SmartDashboard.putNumber("tz Tag", tagToCamera[2]);
        SmartDashboard.putNumber("pitch Tag", tagToCamera[3]);
        SmartDashboard.putNumber("yaw Tag", tagToCamera[4]);
        SmartDashboard.putNumber("roll Tag", tagToCamera[5]);
        
        SmartDashboard.putNumber("Tag ID", aprilTagID);

    }


    public static int getDetected_ID(){
        //System.out.println(detected_ID);
        return detected_ID;
    }
    
    public static double getX(){
        return statX;
    }

    public static double getYaw(){
        return statYaw;
    }

    public static double getY(){
        return statY;
    }

    public static double getDistX(){
        return NetworkTableInstance.getDefault().getTable("limelight-algae").getEntry("targetpose_cameraspace").getDoubleArray(new double[6])[0];
    }

    public static double getDistYaw(){
        return NetworkTableInstance.getDefault().getTable("limelight-algae").getEntry("targetpose_cameraspace").getDoubleArray(new double[6])[4];
    }

    public static double getDistZ(){
        return NetworkTableInstance.getDefault().getTable("limelight-algae").getEntry("targetpose_cameraspace").getDoubleArray(new double[6])[2];
    }


    public void alignTag(double tagID, double distance){
        //Pose2d current_position = 
        //use functions from limelight helpers as examples or just copy?
    }

    
}
