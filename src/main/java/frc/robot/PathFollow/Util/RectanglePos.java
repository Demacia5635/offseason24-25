
package frc.robot.PathFollow.Util;

import edu.wpi.first.math.geometry.Translation2d;

public class RectanglePos {
    private Translation2d topRight;
    private Translation2d bottomLeft;
    public RectanglePos(Translation2d topRight, Translation2d bottomLeft){
        if(bottomLeft.getY() >= topRight.getY() || bottomLeft.getX() >= topRight.getX()){}
            // System.out.println("WRONG INPUT IN RECTANGLE");
            
        
        else{
            this.bottomLeft = bottomLeft;
            this.topRight = topRight;
        }
        
    }

    public boolean isInside(Translation2d pos){
        return pos.getX() >= bottomLeft.getX() && pos.getX() <= topRight.getX() &&
        pos.getY() >= bottomLeft.getY() && pos.getY() <= topRight.getY();
    }
}
