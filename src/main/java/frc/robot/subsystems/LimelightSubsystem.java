package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.libs.LimelightHelpers;
import frc.robot.libs.LimelightHelpers.RawDetection;
import frc.robot.libs.LimelightHelpers.RawFiducial;

public class LimelightSubsystem {

    // // Basic targeting data
    // double tx = LimelightHelpers.getTX(""); // Horizontal offset from crosshair to target in degrees
    // double ty = LimelightHelpers.getTY(""); // Vertical offset from crosshair to target in degrees
    // double ta = LimelightHelpers.getTA(""); // Target area (0% to 100% of image)
    // boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?

    // double txnc = LimelightHelpers.getTXNC(""); // Horizontal offset from principal pixel/point to target in degrees
    // double tync = LimelightHelpers.getTYNC(""); // Vertical offset from principal pixel/point to target in degrees

    public LimelightSubsystem() {

        // Switch to pipeline 0
        LimelightHelpers.setPipelineIndex("", 0);
        // Let the current pipeline control the LEDs
        LimelightHelpers.setLEDMode_PipelineControl("");

        // // Force LEDs on/off/blink
        // LimelightHelpers.setLEDMode_ForceOn("");
        // LimelightHelpers.setLEDMode_ForceOff("");
        // LimelightHelpers.setLEDMode_ForceBlink("");

        // Set a custom crop window for improved performance (-1 to 1 for each value)
        LimelightHelpers.setCropWindow("", -0.5, 0.5, -0.5, 0.5);

        // Change the camera pose relative to robot center (x forward, y left, z up,
        // degrees)
        LimelightHelpers.setCameraPose_RobotSpace("",
                0.5, // Forward offset (meters)
                0.0, // Side offset (meters)
                0.5, // Height offset (meters)
                0.0, // Roll (degrees)
                30.0, // Pitch (degrees)
                0.0 // Yaw (degrees)
        );

        // // Set AprilTag offset tracking point (meters)
        // LimelightHelpers.setFiducial3DOffset("",
        //         0.0, // Forward offset
        //         0.0, // Side offset
        //         0.5 // Height offset
        // );

        // // Configure AprilTag detection
        // LimelightHelpers.SetFiducialIDFiltersOverride("", new int[] { 1, 2, 3, 4 }); // Only track these tag IDs
        // LimelightHelpers.SetFiducialDownscalingOverride("", 2.0f); // Process at half resolution for improved framerate
        //                                                            // and reduced range

        // // Get raw AprilTag/Fiducial data
        // RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
        // for (RawFiducial fiducial : fiducials) {
        //     int id = fiducial.id; // Tag ID
        //     double txnc = fiducial.txnc; // X offset (no crosshair)
        //     double tync = fiducial.tync; // Y offset (no crosshair)
        //     double ta = fiducial.ta; // Target area
        //     double distToCamera = fiducial.distToCamera; // Distance to camera
        //     double distToRobot = fiducial.distToRobot; // Distance to robot
        //     double ambiguity = fiducial.ambiguity; // Tag pose ambiguity
        // }

        // // Get raw neural detector results
        // RawDetection[] detections = LimelightHelpers.getRawDetections("");
        // for (RawDetection detection : detections) {
        //     int classID = detection.classId;
        //     double txnc = detection.txnc;
        //     double tync = detection.tync;
        //     double ta = detection.ta;
        //     // Access corner coordinates if needed
        //     double corner0X = detection.corner0_X;
        //     double corner0Y = detection.corner0_Y;
        //     // ... corners 1-3 available similarly
        // }
    }

    
    public double limelight_range_proportional()
    {    
      double kP = .1;
      double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
      targetingForwardSpeed *= Constants.maxSpeed;
      targetingForwardSpeed *= -1.0;
      return targetingForwardSpeed;
    }

    public double limelight_aim_proportional()
    {    
      // kP (constant of proportionality)
      // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
      // if it is too high, the robot will oscillate.
      // if it is too low, the robot will never reach its target
      // if the robot never turns in the correct direction, kP should be inverted.
      double kP = .035;
  
      // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
      // your limelight 3 feed, tx should return roughly 31 degrees.
      double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;
  
      // convert to radians per second for our drive method
      targetingAngularVelocity *= Constants.maxAngularVelocity;
  
      //invert since tx is positive when the target is to the right of the crosshair
      targetingAngularVelocity *= -1.0;
  
      return targetingAngularVelocity;
    }
}
