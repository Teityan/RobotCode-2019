/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;


public class EncoderGroup implements PIDSource{

    private Encoder e_driveA, e_driveB;
    
    EncoderGroup(Encoder e_drive) {
        this(e_drive, e_drive);
        }

    EncoderGroup(Encoder e_driveA, Encoder e_driveB) {
        this.e_driveA = e_driveA;
        this.e_driveB = e_driveB;
    }
    
    public void setDistancePerPulse(double distancePerPulse) {
        e_driveA.setDistancePerPulse(distancePerPulse);
        e_driveB.setDistancePerPulse(distancePerPulse);
    }
    
    public double getDistance() {
        return (e_driveA.getDistance() + e_driveB.getDistance()) / 2;
    }
    
    public void reset() {
        e_driveA.reset();
        e_driveB.reset();
    }
    
        @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        // TODO 自動生成されたメソ�?ド�?�スタ�?

    }
    
        @Override
    public PIDSourceType getPIDSourceType() {
        // TODO 自動生成されたメソ�?ド�?�スタ�?
        return PIDSourceType.kRate;
    }
    
        @Override
    public double pidGet() {
        // TODO 自動生成されたメソ�?ド�?�スタ�?
        return getDistance();
    }
    
    
}