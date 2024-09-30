
package frc.robot.PathFollow.Util;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class RoundedPoint{
    double smoothnes = 10;
    double radius;
    Translation2d aPoint;
    Translation2d bPoint;
    Translation2d cPoint;
    Translation2d vectorAtoB;
    Translation2d vectorBtoC;
    Rotation2d cornerDir;
    Rotation2d cornerAngle;

    boolean aprilTagMode;

    public RoundedPoint(pathPoint aPoint, pathPoint bPoint, pathPoint cPoint, boolean aprilTagMode){
        this.radius = bPoint.getRadius();
        this.aPoint = aPoint.getTranslation();
        this.bPoint = bPoint.getTranslation();
        this.cPoint = cPoint.getTranslation();
        this.aprilTagMode = aprilTagMode;
        
        vectorAtoB = this.bPoint.minus(this.aPoint);
        vectorBtoC = this.cPoint.minus(this.bPoint);



        this.cornerAngle = vectorAtoB.times(-1).getAngle().minus(vectorBtoC.getAngle());
        //case for angle of the corner is close to 0 so only leg
        if(Math.abs(cornerAngle.getDegrees()) >= 175 && Math.abs(cornerAngle.getDegrees()) <= 180)
            this.radius = 0;
        else{
            //case for radius bigger then max possible radius
            if(radius > getMaxRadius()){
                radius = getMaxRadius();
                // System.out.println("radius is bigger then possible, new radius is: " + radius);
            }
    
        }
        this.cornerDir = vectorAtoB.times(-1).getAngle().minus(this.cornerAngle.div(2));

        
        

    }
    
    
    public double getMaxRadius()
    {
        return Math.sin(Math.abs(this.cornerAngle.getRadians()) / 2) * Math.min(vectorAtoB.getNorm(), vectorBtoC.getNorm());
    }

    

    /**
     * 
     * @return The position of the corner's circle center
     */
    //Calculate Cross Angle vector here too
    public Translation2d getCenterCircle(){
        double length;

        if(this.cornerAngle.div(2).getSin() != 0 && Math.abs(this.cornerAngle.getDegrees()) < 177)
            length =  radius / Math.abs(this.cornerAngle.div(2).getSin());
        else
            length = 0;
        
        Translation2d dirVector = new Translation2d(length, this.cornerDir);
        return dirVector.plus(bPoint);
    }

    /**
     * 
     * @return The starting position of the corner's curve (relative to the corner's circle's center)
     */
    //Calculate cornerDegree beforehand.
    public Translation2d startRange()
    {
        return new Translation2d(radius, vectorAtoB.times(-1).getAngle().plus(new Rotation2d((Math.PI/2) * Math.signum(cornerAngle.getDegrees()))));
    }
    /**
     * 
     * @return The ending position of the corner's curve (relative to the corner's circle's center) 
     */
    public Translation2d endRange()
    {
        return new Translation2d(radius, vectorBtoC.getAngle().minus(new Rotation2d((Math.PI/2) * Math.signum(cornerAngle.getDegrees()))));
    }



    /**
     * 
     * @return An array of points that represent the corner's curve's structure
     */

    public Translation2d[] getPoints(){
        int place = 0;
        Translation2d[] points = new Translation2d[(int)smoothnes];
        double diffAngle = endRange().getAngle().getDegrees() - startRange().getAngle().getDegrees();
        for (double i = 0; i < diffAngle; i = i + (diffAngle / smoothnes)) {
            points[place] = startRange().rotateBy((new Rotation2d(Math.toRadians(i)))).plus(getCenterCircle());
            place++;
        }
        return points;
    } 

    /**
     * 
     * @return the length of the corner's curve
     */
    public double getCurveLength()
    {
        double diffAngle = Math.abs(endRange().getAngle().minus(startRange().getAngle()).getRadians());
        return this.radius * diffAngle;
    }

    /**
     * 
     * @return The distance between point A and the start of the corner's curve
     */
    public double getAtoCurvelength()
    {
        return getCenterCircle().plus(startRange()).minus(aPoint).getNorm();
    }

    public Translation2d getCurveStart()
    {
        return getCenterCircle().plus(startRange());
    }

    public Translation2d getCurveEnd()
    {
        return getCenterCircle().plus(endRange());
    }

    /**
     * 
     * @return The distance between point C and the end of the corner's curve
     */
    public double getCtoCurvelength()
    {
        return getCenterCircle().plus(endRange()).minus(cPoint).getNorm();
    }

    public double getTotalLength()
    {
        return getCtoCurvelength() + getCurveLength() + getAtoCurvelength();
    }


    public Leg getAtoCurveLeg()
    {
        return new Leg(aPoint, startRange().plus(getCenterCircle()), aprilTagMode);
    }
    public Leg getCtoCurveLeg()
    {
        return new Leg(endRange().plus(getCenterCircle()), cPoint, aprilTagMode);
    }

    public Arc getArc()
    {
        Rotation2d diffAngle = endRange().getAngle().minus(startRange().getAngle());
        return new Arc(startRange().plus(getCenterCircle()), getCenterCircle(), diffAngle, aprilTagMode);
    }
}
