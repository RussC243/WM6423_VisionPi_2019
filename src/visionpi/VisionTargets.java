/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 *//*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/**
 *
 * @author RussS9
 */
package visionpi;

import com.sun.org.apache.bcel.internal.generic.TABLESWITCH;
import java.awt.List;
import java.util.ArrayList;
import org.opencv.core.Point;
import java.util.Comparator;
import java.util.Collections;

public class VisionTargets{
    
    private ArrayList<TargetRectangle>   targetRectangles;
    public ArrayList<TargetLine>   targetLines;
        
    private boolean lineProcessingReady = false;
    double tiltAngle = 14.0;
    double tiltThreshold = 5.0;
    Point centerPoint;   
    boolean leftTilt;
    private TargetLine bestTargetLine;
    
    public VisionTargets()
    {
        targetRectangles = new ArrayList<>();   //holds target object for each found target half
        targetLines = new ArrayList<>();        //holds target object for line segment
        bestTargetLine = new TargetLine(new Point(-1,-1), new Point (-1,-1));
    }
    
    class stringLengthComparitor implements Comparator<String>
    {
        @Override
        public int compare(String s1, String s2)
        {
            int len1 = s1.length();
            int len2 = s2.length();
            if(len1 == len2)
            {
                return 0;
            }
            if(len1 < len2)
            {
                return 1;
            }
            return -1;
        }
    }
    public void addLineSegment(Point endPoint1, Point endPoint2)
    {
        TargetLine target = new TargetLine(endPoint1, endPoint2);
        targetLines.add(target);
        lineProcessingReady = false;
    }
   
    public void addRectangle(Point centerPoint, double tiltAngle)
    {
        TargetRectangle target = new TargetRectangle(centerPoint, tiltAngle);
        targetRectangles.add(target);
    }
    public int getLineCount()
    {
        return targetLines.size();
    }
 
    public int getRectangleCount()
    {
        return targetRectangles.size();
    }
    
    public void reset()
    {
        targetRectangles.clear();
        targetLines.clear();
        lineProcessingReady = false;
    }
    public Boolean foundValidTargetRectanglePair()
    {
        if(targetRectangles.size() >= 2)
        {
//            Collections.sort(targets, (a, b) -> b.compareTo(a));
  
//            targets.sort(centerPoint.x);
            return true;
        }
        return false;
    }
    public Point getBestTargetPoint()
    {
        Point pRet = new Point(-1,-1);
        if(foundValidTargetRectanglePair())
        {
            pRet.x = (targetRectangles.get(0).centerPoint.x + targetRectangles.get(1).centerPoint.x)/2;
            pRet.y = (targetRectangles.get(0).centerPoint.y + targetRectangles.get(1).centerPoint.y)/2;
        }
        return pRet;
    }
   
    public boolean foundLineTarget()
    {
        if(!lineProcessingReady)
        {
            repairSegments(); 
            groupSegmentPairs();
            lineProcessingReady = true;
        }
        return false;
    }
    public Point getBestTargetLine()
    {
        if(!lineProcessingReady)
        {
            repairSegments(); 
            groupSegmentPairs();
            lineProcessingReady = true;
        }
        return getBestTargetLineInfo();
    }
    
    public void repairSegments()//if segment endpoints are close and angles are close, replace segments with one long segment
    {
       
        for(int i = 0; i < targetLines.size()-1;i++)
        {
            for(int j = i+1; j<targetLines.size();j++)
            {
                TargetLine line1 = targetLines.get(i);    
                TargetLine line2 = targetLines.get(j);    
               
                double endPointDistanceTolerance = 5.0;//how far can endpoints be before we combine segments ? distance in pixels
                double lineAngleTolerance = 40.0;//how close does angle have to be before we combine segments ? angle in degrees
                if(Math.abs(line1.angleContinuous_0_TO_180()- line2.angleContinuous_0_TO_180()) < lineAngleTolerance)
                {
                    //With 2 lines, there are 4 cases to consider 
                    if(distanceBetweenPoints(line1.endPoint1, line2.endPoint1) < endPointDistanceTolerance  )
                    {
                        line1.endPoint1 = line2.endPoint2;
                        targetLines.remove(j);
                    }
                    else
                    {
                        if(distanceBetweenPoints(line1.endPoint1, line2.endPoint2) < endPointDistanceTolerance )
                        {
                            line1.endPoint1 = line2.endPoint1;
                            targetLines.remove(j);
                        }
                        else
                        {
                            if(distanceBetweenPoints(line1.endPoint2, line2.endPoint1) < endPointDistanceTolerance  )
                            {
                                line1.endPoint2 = line2.endPoint2;
                                targetLines.remove(j);
                            }
                            else
                            {
                                if(distanceBetweenPoints(line1.endPoint2, line2.endPoint2) < endPointDistanceTolerance  )
                                {
                                    line1.endPoint2 = line2.endPoint1;
                                    targetLines.remove(j);
                                }
                            }
                        }
                    }
                }
            }           
        }
    }
    private void groupSegmentPairs()//find pairs of segments that could be sides of 2" x 18" tape
    {
        
    }
    public Point getTargetLineEndPoint(int index, boolean firstPointPlease)
    {
        Point pRet = new Point(-1,-1);
        if(targetLines.size() > index)
        {
            TargetLine line = targetLines.get(index);
            if(firstPointPlease)
            {
                return line.endPoint1;
            }
            return line.endPoint2;
        }
        return pRet;
    }
    public double getTargetLineLength(int index)
    {
        return Math.abs(distanceBetweenPoints(targetLines.get(index).endPoint1, targetLines.get(index).endPoint2));
    }
    private double distanceBetweenPoints(Point p1, Point p2)
    {
        double deltaX = p2.x - p1.x;
        double deltaY = p2.y - p1.y;
        return Math.sqrt(deltaX*deltaX + deltaY*deltaY);
    }
    private Point getBestTargetLineInfo()//return the head and tail points 
    {
        Point pRet = new Point(-1,-1);
        return pRet;
    }
    public boolean isLinePairTarget(int lineIndex1, int lineIndex2)
    {
        //line pair is a target when
        //  adding a line connecting center points forms an H shape
        //  ratio of average line length and distance between lines is withing range
        //  vanishing point is in a reasonable place
        //make sure both lines exist 
        if(targetLines.size() <= lineIndex1 || targetLines.size() < lineIndex2)
        {         
            return false;
        }

        //copy the 4 x and 4 y values to make things easier to read
        double line1P1x = targetLines.get(lineIndex1).endPoint1.x;
        double line1P2x = targetLines.get(lineIndex1).endPoint2.x;
        double line1P1y = targetLines.get(lineIndex1).endPoint1.y;
        double line1P2y = targetLines.get(lineIndex1).endPoint2.y;
        
        double line2P1x = targetLines.get(lineIndex2).endPoint1.x;
        double line2P2x = targetLines.get(lineIndex2).endPoint2.x;
        double line2P1y = targetLines.get(lineIndex2).endPoint1.y;
        double line2P2y = targetLines.get(lineIndex2).endPoint2.y;

        //determine center point of each line
        double xAvg1stLine = (line1P1x + line1P2x) / 2;
        double yAvg1stLine = (line1P1y + line1P2y) / 2;
        double xAvg2ndLine = (line2P1x + line2P2x) / 2;
        double yAvg2ndLine = (line2P1y + line2P2y) / 2;

        TargetLine lineH = new TargetLine(new Point(xAvg1stLine, yAvg1stLine), new Point(xAvg2ndLine, yAvg2ndLine));

        //Now that we have the three lines, check to see if it forms an H
        double angle1 = targetLines.get(lineIndex1).angleContinuous_0_TO_180();
        double angle2 = targetLines.get(lineIndex2).angleContinuous_0_TO_180();
        double angleH = lineH.angleContinuous_0_TO_180();
        double angleH_tolerance = 35;
        
        
        //Filter each angle made by the crossing line and the other two lines
        // Angles are always positive (0 to 180)
        // There are many ways to do this. One way is to subtract smaller from the larger and compare to 90
        // This is not the same as ABS function
        double angleDiff = angle1 - angleH;
        if(angle1 < angleH)
        {
            angleDiff = angleH - angle1;
        }
        
        if(Math.abs(90 - angleDiff) > angleH_tolerance)
        {
            return false; //the crossing line of the H is not close enough to 90 degrees
        }
            
        //Filter the aspect ratio of the H         
        //Target line is a 2" x 18" peice of tape 
        //Different distances and angles cause the ratio to be all over the map so this filter has to be very tolerant
        //We could account for the camera pitch and figure out the projection but that will need some time to figure out
        //A quick hack that should work well is to filter when the H is to wide and allow all narrow ones
        double minAspectRatio = 0.35;//Line aspect rarion get pretty small when far away and lined up with camera axis
        if(     ((targetLines.get(lineIndex1).length() /  lineH.length()) < minAspectRatio) ||
                ((targetLines.get(lineIndex2).length() /  lineH.length()) < minAspectRatio))
        {
            return false;
        }
        
        //filter narrow H's
        if(lineH.length() > 40)
        {
          //  return false;
        }
        
        //Only accept line pairs that have a vanishing point (intersection) at a reasonable spot (behind target)
        Point intersectionPoint = lineLineIntersection( targetLines.get(lineIndex1).endPoint1,
                                                        targetLines.get(lineIndex1).endPoint2,
                                                        targetLines.get(lineIndex2).endPoint1,
                                                        targetLines.get(lineIndex2).endPoint2);
        if(intersectionPoint.y > 50)
        {
            return false;
        }
        
        
        return true; // valid target
    }
    
    public Point lineLineIntersection(Point A, Point B, Point C, Point D) 
    { 
        // Line AB represented as a1x + b1y = c1 
        double a1 = B.y - A.y; 
        double b1 = A.x - B.x; 
        double c1 = a1*(A.x) + b1*(A.y); 
       
        // Line CD represented as a2x + b2y = c2 
        double a2 = D.y - C.y; 
        double b2 = C.x - D.x; 
        double c2 = a2*(C.x)+ b2*(C.y); 
       
        double determinant = a1*b2 - a2*b1 +0.000001;//avoid div by zero, works for whole doubles 
       
        if (determinant == 0) 
        { 
            // The lines are parallel. This is simplified 
            // by returning a pair of FLT_MAX 
            return new Point(620, 460); 
        } 
        else
        { 
            double x = (b2*c1 - b1*c2)/determinant; 
            double y = (a1*c2 - a2*c1)/determinant; 
            return new Point(x, y); 
        } 
    } 
 
    public double getHLineAngle(int lineIndex1, int lineIndex2)
    {
        //line pair is a target when
        //  adding a line connecting center points forms an H shape
        //  ratio of average line length and distance between lines is withing range
        //  vanishing point is in a reasonable place
        //make sure both lines exist 
        if(targetLines.size() <= lineIndex1 || targetLines.size() < lineIndex2)
        {         
            return -1;
        }

        //copy the 4 x and 4 y values to make things easier to read
        double line1P1x = targetLines.get(lineIndex1).endPoint1.x;
        double line1P2x = targetLines.get(lineIndex1).endPoint2.x;
        double line1P1y = targetLines.get(lineIndex1).endPoint1.y;
        double line1P2y = targetLines.get(lineIndex1).endPoint2.y;
        
        double line2P1x = targetLines.get(lineIndex2).endPoint1.x;
        double line2P2x = targetLines.get(lineIndex2).endPoint2.x;
        double line2P1y = targetLines.get(lineIndex2).endPoint1.y;
        double line2P2y = targetLines.get(lineIndex2).endPoint2.y;

        //determine center point of each line
        double xAvg1stLine = (line1P1x + line1P2x) / 2;
        double yAvg1stLine = (line1P1y + line1P2y) / 2;
        double xAvg2ndLine = (line2P1x + line2P2x) / 2;
        double yAvg2ndLine = (line2P1y + line2P2y) / 2;

        TargetLine lineH = new TargetLine(new Point(xAvg1stLine, yAvg1stLine), new Point(xAvg2ndLine, yAvg2ndLine));
        return lineH.angleContinuous_0_TO_180();
    }

    public TargetLine getHLine(int lineIndex1, int lineIndex2)
    {
        //line pair is a target when
        //  adding a line connecting center points forms an H shape
        //  ratio of average line length and distance between lines is withing range
        //  vanishing point is in a reasonable place
        //make sure both lines exist 
        if(targetLines.size() <= lineIndex1 || targetLines.size() < lineIndex2)
        {         
            return new TargetLine(new Point(-1,-1), new Point(-1,-1));
        }

        //copy the 4 x and 4 y values to make things easier to read
        double line1P1x = targetLines.get(lineIndex1).endPoint1.x;
        double line1P2x = targetLines.get(lineIndex1).endPoint2.x;
        double line1P1y = targetLines.get(lineIndex1).endPoint1.y;
        double line1P2y = targetLines.get(lineIndex1).endPoint2.y;
        
        double line2P1x = targetLines.get(lineIndex2).endPoint1.x;
        double line2P2x = targetLines.get(lineIndex2).endPoint2.x;
        double line2P1y = targetLines.get(lineIndex2).endPoint1.y;
        double line2P2y = targetLines.get(lineIndex2).endPoint2.y;

        //determine center point of each line
        double xAvg1stLine = (line1P1x + line1P2x) / 2;
        double yAvg1stLine = (line1P1y + line1P2y) / 2;
        double xAvg2ndLine = (line2P1x + line2P2x) / 2;
        double yAvg2ndLine = (line2P1y + line2P2y) / 2;

        TargetLine lineH = new TargetLine(new Point(xAvg1stLine, yAvg1stLine), new Point(xAvg2ndLine, yAvg2ndLine));
        return lineH;
    }
    
    public Point getStartingPointOfLinePair(int lineIndex1, int lineIndex2)
    {
        //This function returns the point between the lowest two ends of the lines 
        Point pRet = new Point(-1,-1);
        //make sure both lines exist 
        if(targetLines.size() > lineIndex1 && targetLines.size() > lineIndex2)
        {
           //copy the 4 x and 4 y values to make things easier to read
           double x1_1 = targetLines.get(lineIndex1).endPoint1.x;
           double x1_2 = targetLines.get(lineIndex1).endPoint2.x;
           double x2_1 = targetLines.get(lineIndex2).endPoint1.x;
           double x2_2 = targetLines.get(lineIndex2).endPoint2.x;
           double y1_1 = targetLines.get(lineIndex1).endPoint1.y;
           double y1_2 = targetLines.get(lineIndex1).endPoint2.y;
           double y2_1 = targetLines.get(lineIndex2).endPoint1.y;
           double y2_2 = targetLines.get(lineIndex2).endPoint2.y;

           //identify the highest y value for each line and save the x as well
           //positive y is down
           double lowestX1 = x1_1;
           double lowestY1 = y1_1;
           if(y1_2 > y1_1)
           {
                lowestX1 = x1_2;
                lowestY1 = y1_2;
           }

           double lowestX2 = x2_1;
           double lowestY2 = y2_1;
           if(y2_2 > y2_1)
           {
                lowestX2 = x2_2;
                lowestY2 = y2_2;
           }
           //the average is the center point
           pRet.x = (lowestX1 + lowestX2)/2;
           pRet.y = (lowestY1 + lowestY2)/2;
        }
        return pRet;
    }

    public Point getEndingPointOfLinePair(int lineIndex1, int lineIndex2)
    {
        //This function returns the point between the highest two ends of the lines 
        Point pRet = new Point(-1,-1);
        //make sure both lines exist 
        if(targetLines.size() > lineIndex1 && targetLines.size() > lineIndex2)
        {
           //copy the 4 x and 4 y values to make things easier to read
           double x1_1 = targetLines.get(lineIndex1).endPoint1.x;
           double x1_2 = targetLines.get(lineIndex1).endPoint2.x;
           double x2_1 = targetLines.get(lineIndex2).endPoint1.x;
           double x2_2 = targetLines.get(lineIndex2).endPoint2.x;
           double y1_1 = targetLines.get(lineIndex1).endPoint1.y;
           double y1_2 = targetLines.get(lineIndex1).endPoint2.y;
           double y2_1 = targetLines.get(lineIndex2).endPoint1.y;
           double y2_2 = targetLines.get(lineIndex2).endPoint2.y;

           //identify the lowest y value for each line and save the x as well
           //positive y is down
           double highestX1 = x1_1;
           double highestY1 = y1_1;
           if(y1_2 < y1_1)
           {
                highestX1 = x1_2;
                highestY1 = y1_2;
           }

           double highestX2 = x2_1;
           double highestY2 = y2_1;
           if(y2_2 < y2_1)
           {
                highestX2 = x2_2;
                highestY2 = y2_2;
           }
           //the average is the center point
           pRet.x = (highestX1 + highestX2)/2;
           pRet.y = (highestY1 + highestY2)/2;
        }
        return pRet;
    }

    private class TargetRectangle
    {
        Point centerPoint;
        double tiltAngle;
        public TargetRectangle(Point centerPoint_in, double tiltAngle_in)
        {
            centerPoint = new Point(centerPoint_in.x, centerPoint_in.y);
            tiltAngle = tiltAngle_in;
        }
        public Boolean isLeft()
        {
            return tiltAngle > 0;
        }
        public Boolean isRight()
        {
            return tiltAngle <= 0;//angles are always near 14 degrees but consider = for compleatness
        }
    }
    public class TargetLine
    {
        Point endPoint1;
        Point endPoint2;
        public TargetLine(Point endPoint1_in, Point endPoint2_in)
        {
            endPoint1 = new Point(endPoint1_in.x, endPoint1_in.y);
            endPoint2 = new Point(endPoint2_in.x, endPoint2_in.y);
        }
        public double length()
        {
            return distanceBetweenPoints(endPoint1, endPoint2);
        }
        public double angle()//returns +/- 90 degrees
        {
           double m =  (endPoint1.y - endPoint2.y)/(endPoint1.x - endPoint2.x + 0.0000001);//add a bit to avoid div by 0 - can do this when dealing with whole doubles or ints
           double angle = -Math.toDegrees(Math.atan(m));    //calculate angle of line
           return angle;
        }
        public double angleContinuous_0_TO_180()//returns 0 to 180 degrees
        {
           double m =  (endPoint1.y - endPoint2.y)/(endPoint1.x - endPoint2.x + 0.0000001);//add a bit to avoid div by 0 - can do this when dealing with whole doubles or ints
           double angle = -Math.toDegrees(Math.atan(m));    //calculate angle of line
           if(angle<0)
           {
               angle += 180;
           }
           return angle;
        }
        public Boolean isLongSegment()
        {
            return false;
        }
        public Boolean isShortSegment()
        {
            return false;
        }
    }
}
