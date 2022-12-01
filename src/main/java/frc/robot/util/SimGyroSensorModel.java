package frc.robot.util;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import frc.robot.Robot;

public class SimGyroSensorModel{

    SimDeviceSim gyroSim;
    SimDouble rateSimDouble;
    SimDouble yawSimDouble;
    double gyroPosReading_deg;

    public SimGyroSensorModel(){
        gyroSim = new SimDeviceSim("navX-Sensor[0]");
        rateSimDouble = gyroSim.getDouble("Rate");
        yawSimDouble = gyroSim.getDouble("Yaw");

    }

    public void resetToPose(Pose2d resetPose){
        yawSimDouble.set(resetPose.getRotation().getDegrees() * -1.0);
    }
    /**
     * 
     * @param curRobotPose
     * @param prevRobotPose
     * @param deltaTime the
     */
    public void update(Pose2d curRobotPose, Pose2d prevRobotPose, Double deltaTime){
        double curGyroAngle  = curRobotPose.getRotation().getDegrees();
        double prevGyroAngle = prevRobotPose.getRotation().getDegrees();
        double gyroRate = (curGyroAngle - prevGyroAngle)/deltaTime; //Gyro reads backward from sim reference frames.
        // Pass our model of what the sensor would be measuring back into the simGyro object
        // for the embedded code to interact with.
        rateSimDouble.set(gyroRate);
        yawSimDouble.set(yawSimDouble.get() + (gyroRate*deltaTime));
       
    }
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(yawSimDouble.get());
    }
}