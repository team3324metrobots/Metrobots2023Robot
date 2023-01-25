package frc.robot.vision;

import org.photonvision.PhotonCamera;

import edu.wpi.first.net.PortForwarder;

public class PhotonVision {

    public static void main(String[] args){

        PortForwarder.add(5800, "photonvision.local", 5800);

        PhotonCamera camera = new PhotonCamera("Metrobots3324");

        camera.setDriverMode(false);  

        var result = camera.getLatestResult();{
            
            //boolean hasTargets = result.hasTargets();{}

                if((result.hasTargets()))
                {
                    camera.setDriverMode(true); 
                }
        }

    }    
}



