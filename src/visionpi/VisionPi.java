package visionpi;
import com.sun.jndi.url.rmi.rmiURLContext;
import com.sun.org.apache.bcel.internal.generic.AALOAD;
import java.awt.Color;
import java.util.AbstractList;
import java.util.ArrayList;
import java.util.List;
import java.util.Vector;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Size;
import org.opencv.core.Point;
import org.opencv.imgcodecs.Imgcodecs;  
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.LineSegmentDetector;


import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;
import org.opencv.highgui.HighGui;
import org.opencv.core.CvType;
import org.opencv.core.Scalar;
import org.opencv.core.Core;
import org.opencv.core.Rect;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import java.util.Collections;
import org.graalvm.compiler.hotspot.nodes.FastAcquireBiasedLockNode;
public class VisionPi 
{
    static final boolean SEND_TO_SOCKET_SERVER = true;      //set to true if socket server is running
    static final boolean ENABLE_GUI = true;                //set to false to disable gui
    static final boolean ENABLE_PRINT_SLIDER_VALUES = true; //set to false to disable print of slider values
    
    static int state = 2;   //initial State 3 uses hardcoded cal values. Set to -1 and enable gui to cal
                            // set to state 1 to develop deep space tracking
    static final String serverAddr = "10.64.23.2"; //roboRio IP address
    static final int    serverPort = 1234;       //roboRio server port   
    static final double H_FOV = 62.2;//Horizontal field of view from Pi camera V2 spec, degrees
    static final double V_FOV = 48.8;//Vertical field of view from Pi camera V2 spec, degrees
    static final double RED_BALL_DIAMETER = 2.1; //inches
    static final double DISTANCE_FUDGE = 1.0/1.0; //inches
    

    static final int WIDTH = 640;
    static final int HEIGHT = 480;

    //Region of Interest (ROI) - reduces area to be processed to insure we are done before next frame is available (
    // Camera sends frames at 30fps (1/30 = 33mS frame period)
    static int ROI_Pan  = 0;    //positive pans ROI  right from center, negative pans  left
    static int ROI_Tilt = 100;  //positive tilts ROI down  from center, negative tilts up
    static int ROI_Tilt_Default = 100;  //view  just below horizon for a ball
    
    static int ROI_rowSpan = 56;    //pix height of detection area
    static int ROI_colSpan = 300;   //pix width of detection area
    static int ROI_left = WIDTH/2  + ROI_Pan  - ROI_colSpan/2;  //left edge of ROI
    static int ROI_top  = HEIGHT/2 + ROI_Tilt - ROI_rowSpan/2;  //top edge of ROI
    static boolean ROI_enablePanScan = true;//enable auto pan scan to find ball, implies pan tracking
    static boolean ROI_enableTiltTrack = true;//enable tilt tracking to keep ball centered vertically
    
    static boolean ROI_panningLeft = false; //true when auto panning towards the left, false when panning right 
    static double ROI_lastDetectedBallCenterX = 0;
    static double ROI_lastDetectedBallCenterY = 0;
    static double ROI_lastDetectedBallDistance = 0;
    static boolean ROI_ballDetectedInLastFrame = false;
    static double ROI_centerXThreshold = 3.0;//tolerance to keep ball in center of ROI, H axis, pixels
    static double ROI_centerYThreshold_TiltUp   = 0.0;//tolerance to keep ball in center of ROI, V axis, pixels, up direction
    static double ROI_centerYThreshold_TiltDown = 0.0;//tolerance to keep ball in center of ROI, V axis, pixels, down direction
   
    //Some stuff to  display how many mS each chunk of code is taking to execute 
    static int performanceTimeCounter = 0; //used to count number of times we do something for accurate time measurements
    static double performanceTimeAvg1 = 0;
    static double performanceTimeAvg2 = 0;
    static double performanceTimeAvg3 = 0;
    static double performanceTimeAvg4 = 0;
    static double performanceTimeAvg5 = 0;
    static double performanceTimeAvg6 = 0;
    static double performanceTimeAvg7 = 0;
    static double performanceTimeAvg8 = 0;
    
    
    static SocClient socClient;
    static VisionTargets vTargs;
    static Pi_GUI gui;
    static LEDs leds;
    static Mat matLive;
    static Mat matHSV;
    static Mat matTarget;
 //   static Mat matLiveOrg;
    static Mat matROI;
    static Mat matROI_processed1;
    static Mat matROI_processed2;
//    static Mat matROI_processed3;
    static Mat matSpacer;
    static int frameCount = 0;
    static double lastTimerValue = 0;    
    static final double DEGREES_PER_RADIAN = Math.toDegrees(1);
    static LineSegmentDetector lsd; //Don't create this object every frame.... java will crash after a few minutes at 30fps
    
    public VisionPi() {
    }
        public static void main(String[] args){
        gui = new Pi_GUI(ENABLE_GUI); //true enables gui; false allows access to default slider values only with no windows.
        vTargs = new VisionTargets();
        leds = new LEDs();            //LEDs class hides the details needed to access the LEDs
        socClient= new SocClient(serverAddr, serverPort);   //SocServ extends thread
        socClient.start();                      //starts the run method
        loadOpenCV();                 //load lib and print some info about it
        matLive = new Mat();
        matHSV = new Mat();
        matTarget = new Mat();
     //   matLiveOrg = new Mat();
        matROI = new Mat();
        matROI_processed1 = new Mat();
        matROI_processed2 = new Mat();
  //      matROI_processed3 = new Mat();
        matSpacer = new Mat(new Size(ROI_colSpan,10), CvType.CV_8UC(3));
        matSpacer.setTo(new Scalar(255,255,255));
        lsd = Imgproc.createLineSegmentDetector(Imgproc.LSD_REFINE_STD,0.8,0.6,5.0,22.5,0,0.3,256);
       
       //int _refine=LSD_REFINE_STD, 
       //double _scale=0.8, 
       //double _sigma_scale=0.6, 
       //double _quant=2.0,             1.0 added 30mS and more broken lines, 3.3 was 3mS faster, 5.0 was a few mS faster still
       //double _ang_th=22.5,           12.5 is a few mS faster
       //double _log_eps=0, 
       //double _density_th=0.7,        0.3 is 3mS faster and more complete lines
       //int _n_bins=1024

        if(ENABLE_GUI){
            HighGui.namedWindow("Live",1);// Create Window
            HighGui.resizeWindow("Live", WIDTH+10, HEIGHT+10);
            HighGui.moveWindow("Live", 20, 300);
            HighGui.namedWindow("Target",1);// Create Window
            HighGui.resizeWindow("Target", WIDTH+10, HEIGHT+10);
            HighGui.moveWindow("Target", 200, 400);
            
            //HighGui.namedWindow("HSV",1);// Create Window
            //HighGui.resizeWindow("HSV", width+10, height+10);
            //HighGui.namedWindow("LiveOrg",1);// Create Window
            //HighGui.resizeWindow("LiveOrg", WIDTH+10, HEIGHT+10);
            //HighGui.namedWindow("ROI",1);// Create Window
            //HighGui.resizeWindow("ROI", 300, 600);
            //HighGui.moveWindow("ROI", 800, 300);
        }
        VideoCapture camera = new VideoCapture(0); 
        if(!camera.isOpened()){
            System.out.println("Camera Error");
        }
        else
        {
            camera.set(Videoio.CAP_PROP_FRAME_WIDTH, WIDTH);
            camera.set(Videoio.CAP_PROP_FRAME_HEIGHT, HEIGHT);
            //camera.set(Videoio.CAP_PROP_BRIGHTNESS, 0.55);
            //camera.set(Videoio.CAP_PROP_CONTRAST, 0.8);
            //camera.set(Videoio.CAP_PROP_EXPOSURE, 0.9);
            
            System.out.println("frame property width  = " + camera.get(Videoio.CV_CAP_PROP_FRAME_WIDTH));
            System.out.println("frame property height = " + camera.get(Videoio.CV_CAP_PROP_FRAME_HEIGHT));
            System.out.println("frame property FPS    = " + camera.get(Videoio.CV_CAP_PROP_FPS));
            System.out.println("frame property brightness    = " + camera.get(Videoio.CAP_PROP_BRIGHTNESS));
            System.out.println("frame property contrast    = " + camera.get(Videoio.CAP_PROP_CONTRAST));
           // System.out.println("frame property Auto Exposure    = " + camera.get(Videoio.CAP_PROP_AUTO_EXPOSURE));
            System.out.println("frame property Exposure    = " + camera.get(Videoio.CAP_PROP_EXPOSURE));
            
            while(camera.read(matLive))
            {
                leds.singleOn(frameCount%2+1);//toggle LED to see frame rate on scope
                frameCount++;
                switch (state){
                    case -1:
                        stateNeg1();
                        break;
                    case 0:
                        break;
                    case 1:
                        state1();
                        break;
                    case 2:
                        stateSplitDetect();
                        //state2();
                        break;
                    case 3:
                        state3();
                        //stateScratch();
                        break;
                    case 4:
                        break;
                    default:
                        break;    
                }
                if(frameCount%100 == 0)
                {
                    double currTime =  System.currentTimeMillis();     
                    System.out.println("frames per sec : " + (100.0 * 1000.0/ (currTime - lastTimerValue)));
                    lastTimerValue = currTime;
                }
                if(ENABLE_GUI && frameCount%10==0)//sliders are more responsive dropping 4/5, really good dropping 9/10
                {
                        HighGui.imshow("Live", matLive);
                       //HighGui.imshow("HSV", matHSV);
                       //HighGui.imshow("LiveOrg", matLiveOrg);
                       HighGui.imshow("Target", matTarget);
                       HighGui.waitKey(10);//increase to slow things down if CPU util too high
                }
            }
            System.out.println("read returned false");
            camera.release();
        }//close else
    }//close main
        
        
    private static void stateSplitDetect()
    {
        long timeCodePoint1 =  System.currentTimeMillis();     
        long timeCodePoint4 =  0;     
        long timeCodePoint5 =  0;     
        vTargs.reset();
     
        //Create a region of interest (ROI) from top 1/4 of image
        //org.opencv.core.Rect roi = new Rect(0,HEIGHT - HEIGHT/4,WIDTH, HEIGHT/4);//bottom 1/4
        org.opencv.core.Rect roi = new Rect(0,0,WIDTH, HEIGHT/4);//top 1/4
        Mat matTemp = matLive.submat(roi); //share ROI with another mat
        matTemp.copyTo(matTarget); 
    
        long timeCodePoint2 =  System.currentTimeMillis();     
        
        Imgproc.cvtColor(matTarget,matTarget,Imgproc.COLOR_BGR2GRAY);
        long timeCodePoint3 =  System.currentTimeMillis();     
         
        Mat linesLSD = new Mat(); //LSD is Line Segment Detector
        try
        {
            lsd.detect(matTarget, linesLSD);                //detect lines in ROI
            timeCodePoint4 =  System.currentTimeMillis();   //update the stats    
            lsd.drawSegments(matLive, linesLSD);            //draw detected lines on live image
            drawText(matLive,new Point(5,100), "Lines in lower ROI " + linesLSD.total());   //dwaw more info on live image
     
            for(int i = 0; i < linesLSD.total(); i++) 
            { 
                Point p = new Point();
                Point q = new Point();
                double[] val = linesLSD.get(i, 0);  //get the line and put it in an array
                // Check if the line is a vertical line... avoid div by 0
                double m=0; //slope
                double b=0; //y intercept
                //check for infinite slope
                if (val[0] == val[2])       //x1 = x2 is an infinite slope 
                {
                    p.x = q.x = val[0];     //vertical line
                    p.y = 0;
                    q.y = matTarget.rows();
                }
                else
                {
                    // slope = m = (y1 - y2) / (x1 - x2)
                    // y = mx + b
                    // intercept = b = y - mx
                    p.x = 0;
                    q.x = matTarget.cols();
                    // Slope equation (y1 - y2) / (x1 - x2)
                    m = (val[1] - val[3]) / (val[0] - val[2]);
                    // Line equation:  y = mx + b
                    b = val[1] - (m * val[0]);
                    p.y = m * p.x + b;
                    q.y = m * q.x + b;
                    double angle = Math.toDegrees(Math.atan(m));    //calculate angle of line
                 //   System.out.printf("angle %.2f degrees\n", angle);
                    if(angle > 15 || angle < -15) // not interested in lines perpendicular to camera axis within some tolerance
                    {
                        Imgproc.line(matLive, new Point(val[0], val[1]), new Point(val[2], val[3]), new Scalar(255, 0, 0),2);
                        vTargs.addLineSegment(new Point(val[0], val[1]),new Point(val[2], val[3]));
                    }
                }
            }
            timeCodePoint5 =  System.currentTimeMillis();     
            drawText(matLive, new Point(5,120), "lines in range:" + vTargs.getLineCount() + " lines OOR:" + (linesLSD.total()-vTargs.getLineCount()));
            int linesOrg =  vTargs.getLineCount();
            vTargs.repairSegments();
            int linesRemaining1 =  vTargs.getLineCount();
           vTargs.repairSegments();
            int linesRemaining2 =  vTargs.getLineCount();
            
            for(int i = 0; i< vTargs.targetLines.size();i++)
            {                
                Imgproc.line(matLive, vTargs.getTargetLineEndPoint(i, true),vTargs.getTargetLineEndPoint(i, false), new Scalar(0, 255, 0),1);
            }
            
            for(int i = 0; i < vTargs.targetLines.size()-1;i++)
            {
                for(int j = i+1; j< vTargs.targetLines.size();j++)
                {
                    if(vTargs.isLinePairTarget(i, j))
                    {
                        Imgproc.line(matLive, vTargs.getHLine(i, j).endPoint1,vTargs.getHLine(i, j).endPoint2, new Scalar(0, 0, 0),1);
                        drawText(matLive, vTargs.getStartingPointOfLinePair(i, j), ".");
                        drawText(matLive, vTargs.getEndingPointOfLinePair(i, j), "|");
                        drawText(matLive, vTargs.lineLineIntersection(  vTargs.targetLines.get(i).endPoint1,
                                                                        vTargs.targetLines.get(i).endPoint2,
                                                                        vTargs.targetLines.get(j).endPoint1,
                                                                        vTargs.targetLines.get(j).endPoint2), 
                                                                        "V");
                    }
                }
            }
            drawText(matLive, new Point(5,140), "repair stats " + linesOrg + " " + linesRemaining1 + " " + linesRemaining2);
        }    
        catch(Exception ex)
        {
            System.out.println("************* exception processing lines ********************");
        }
        long timeCodePoint6 =  System.currentTimeMillis();     
        
        //Detect the left and right retro-reflective rectangles tilted 14 degrees
        //find all the contours in the image
     	List<org.opencv.core.MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(matTarget, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        org.opencv.core.MatOfPoint2f matOfPoint2f = new org.opencv.core.MatOfPoint2f();
        org.opencv.core.MatOfPoint2f approxCurve = new org.opencv.core.MatOfPoint2f();
        drawText(matLive, new Point(5,25), "Contours found: " + contours.size()); 
        long timeCodePoint7 =  System.currentTimeMillis();     
        
        vTargs.reset();
        if(contours.size()>0)
        {
            for (int idx = 0; idx >= 0; idx = (int) hierarchy.get(0, idx)[0]) {
                org.opencv.core.MatOfPoint contour = contours.get(idx);
                Rect rect = Imgproc.boundingRect(contour);
                double contourArea = Imgproc.contourArea(contour);
                matOfPoint2f.fromList(contour.toList());
                Imgproc.approxPolyDP(matOfPoint2f, approxCurve, Imgproc.arcLength(matOfPoint2f, true) * 0.02, true);
                long total = approxCurve.total();
               
                if (total == 4 && rect.area() > 500) {
                    Point center = new Point((rect.br().x + rect.tl().x)/2,(rect.br().y + rect.tl().y)/2);
                    Point[] points = approxCurve.toArray();
                    //put a dot in each of the 4 corners of the rectangle 
                    for(int i=0;i<points.length;i++)
                    {
                        drawText(matLive, points[i], ".");
                    }
                    double angle = 0;
                    if(points.length == 4) //probably always true
                    {
                        angle = rotationOfRectangle(points[0], points[1], points[2], points[3]);
                        double angleTolerance = 5.0;
                        double angleTarget = 14.0;
                        if(Math.abs(Math.abs(angle) - angleTarget) < angleTolerance )
                        {
                            vTargs.addRectangle(center, angle); 
                            if(angle > 0)
                            {
                                drawText(matLive,center, "L");
                            }
                            else
                            {
                                drawText(matLive,center, "R");
                            }
                   
                        }
                    }
                    Point drawHere1 = new Point(center.x - 25,rect.y -15);
                    Point drawHere2 = new Point(center.x - 25,rect.y );
                    drawText(matLive,drawHere1, String.format("Angle %.2f", angle));
                    drawText(matLive,drawHere2, String.format("Area  %.0f ", rect.area()));
                }
            }
        }
        long timeCodePoint8 =  System.currentTimeMillis();     
        long timeCodePoint9 =  System.currentTimeMillis();     
       
        if(vTargs.foundValidTargetRectanglePair())
        {
            drawText(matLive,vTargs.getBestTargetPoint(), "X");
        }
        //send result to server and print to output
         
        String str =    1                   +","+   //area
                        2                   +","+   //width
                        String.format("%.1f", 3.0)   +","+   //distance
                        String.format("%.1f", 4.0);                //direction
                        
        String strVerbose = "TARGET: area " + 1                        +
                            " ContourWidth " + 2                + 
                            " distW "    + String.format("%.1f", 3.0)   +
                            " VPos "     + String.format("%.1f", 4.0)           + 
                            " distVPos "    + String.format("%.1f", 5.0) +
                            " direction "   + String.format("%.1f", 6.0);
        if(SEND_TO_SOCKET_SERVER)
        {
            socClient.send(str);
        }

        if(frameCount%2 == 0)
        {
         //   System.out.println(strVerbose);
        }
        //-------- process and display code profiling -------------------------------------

        performanceTimeAvg1+=timeCodePoint2 - timeCodePoint1;
        performanceTimeAvg2+=timeCodePoint3 - timeCodePoint2;
        performanceTimeAvg3+=timeCodePoint4 - timeCodePoint3;
        performanceTimeAvg4+=timeCodePoint5 - timeCodePoint4;
        performanceTimeAvg5+=timeCodePoint6 - timeCodePoint5;
        performanceTimeAvg6+=timeCodePoint7 - timeCodePoint6;
        performanceTimeAvg7+=timeCodePoint8 - timeCodePoint7;
        performanceTimeAvg8+=timeCodePoint9 - timeCodePoint8;


        performanceTimeCounter++;
        if(performanceTimeCounter%100 == 0)
        {
            System.out.printf("Average times, mS:  %.1f %.1f %.1f %.1f    %.1f %.1f %.1f %.1f\n",performanceTimeAvg1/100,
                                                                                            performanceTimeAvg2/100,
                                                                                            performanceTimeAvg3/100,
                                                                                            performanceTimeAvg4/100,
                                                                                            performanceTimeAvg5/100,
                                                                                            performanceTimeAvg6/100,
                                                                                            performanceTimeAvg7/100,
                                                                                            performanceTimeAvg8/100);
            performanceTimeAvg1 = 0;
            performanceTimeAvg2 = 0;
            performanceTimeAvg3 = 0;
            performanceTimeAvg4 = 0;
            performanceTimeAvg5 = 0;
            performanceTimeAvg6 = 0;
            performanceTimeAvg7 = 0;
            performanceTimeAvg8 = 0;
        }   
        if(false)//frameCount%25 == 0)
        {
            System.out.println("ROI " + ROI_ballDetectedInLastFrame + " " +
                                            ROI_panningLeft + " " +
                                            ROI_lastDetectedBallCenterX + " " +
                                            ROI_lastDetectedBallCenterY + " " +                        
                                            1234 + " " +
                                            4321);
        }
        if(ENABLE_PRINT_SLIDER_VALUES && frameCount%25 == 0)
        {
            System.out.printf("Slider Values %d %d %d %d %d %d\n",  
                            gui.slide1Val, 
                            gui.slide2Val, 
                            gui.slide3Val, 
                            gui.slide4Val, 
                            gui.slide5Val,
                            gui.slide6Val);
        }
      
    }
  
        
        
    private static void stateNeg1(){
       
    }
    private static void state1(){
        long timeCodePoint1 =  System.currentTimeMillis();     
        long timeCodePoint2 =  System.currentTimeMillis();     
        long timeCodePoint3 =  System.currentTimeMillis();     
        long timeCodePoint4 =  System.currentTimeMillis();     
        long timeCodePoint5 =  System.currentTimeMillis();     
        long timeCodePoint6 =  System.currentTimeMillis();     
        long timeCodePoint7 =  System.currentTimeMillis();     
        long timeCodePoint8 =  System.currentTimeMillis();     
        long timeCodePoint9 =  System.currentTimeMillis();     
     
        Imgproc.cvtColor(matLive,matTarget,Imgproc.COLOR_BGR2HSV);
        // Imgproc.erode(matTarget,matTarget, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3)));
        // Imgproc.dilate(matTarget,matTarget, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(7,7)));
     
        /// Reduce the noise so we avoid false circle detection
    //    Imgproc.GaussianBlur( matTarget, matTarget, new Size(3, 3), 2, 2 );
    
        Scalar min = new Scalar(gui.getSlide1(),gui.getSlide3(),gui.getSlide5());
        Scalar max = new Scalar(gui.getSlide2(),gui.getSlide4(),gui.getSlide6());
      
        Scalar minLowRedOnly = new Scalar(0,gui.getSlide3(),gui.getSlide5());
        Scalar maxLowRedOnly = new Scalar(40,gui.getSlide4(),gui.getSlide6());
      
        Mat matLowRedOnly = new Mat();
        Core.inRange(matTarget, minLowRedOnly, maxLowRedOnly, matLowRedOnly);
        Core.inRange(matTarget, min, max, matTarget);
        Core.bitwise_or(matTarget, matLowRedOnly, matTarget);
        Imgproc.erode(matTarget,matTarget, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3)));
        
      
        //detect edges
        int lowThresholdCal = 20;
        int ratioCal = 3;
      //  Imgproc.Canny(matTarget, matTarget, lowThresholdCal, lowThresholdCal*ratioCal);
      //  Mat lines = new Mat();
      //  Imgproc.HoughLinesP(matTarget,  lines,  1, Math.PI/180, 5,50,5);//last 3 params are: threshold, min len, max gap

        //void solvePnPRansac(  InputArray objectPoints, 
        //                      InputArray imagePoints, 
        //                      InputArray cameraMatrix, 
        //                      InputArray distCoeffs, 
        //                      OutputArray rvec, 
        //                      OutputArray tvec, 
        //                      bool useExtrinsicGuess=false, 
        //                      int iterationsCount=100, 
        //                      float reprojectionError=8.0, 
        //                      int minInliersCount=100, 
        //                      OutputArray inliers=noArray(), 
        //                      int flags=ITERATIVE )
        org.opencv.core.MatOfPoint3f objectPoints = new org.opencv.core.MatOfPoint3f();
        org.opencv.core.MatOfPoint2f imagePoints = new org.opencv.core.MatOfPoint2f();
        Mat cameraMatrix = new Mat();
        org.opencv.core.MatOfDouble distCoeffs = new org.opencv.core.MatOfDouble();
        Mat rvec = new Mat();
        Mat tvec = new Mat();
        
        //Calib3d.solvePnPRansac(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
        
     	List<org.opencv.core.MatOfPoint> contours = new ArrayList<>();
//      	Imgproc.cvtColor(matROI_processed2, matROI_processed2, Imgproc.COLOR_BGR2GRAY);
        Imgproc.findContours(matTarget, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        int lastContourRows = 0;
        for(int i=0; i < contours.size(); i++) 
	{
             lastContourRows = contours.get(i).rows();
	}
        if(frameCount%30==0)
        {
            System.out.println("contous found " + contours.size() + " Last Size " + lastContourRows);
        }
//      
        
//        Mat circles = new Mat();
//        Imgproc.HoughCircles( matTarget, circles, Imgproc.CV_HOUGH_GRADIENT, 1, 0, 100, 50, 0, 0 );
//        // Draw the circles detected
//        Point p = new Point(); 
//        for( int i = 0; i < circles.rows(); i++ )
//        {
//            p.x = circles.get(i,0)[0];
//            p.y = circles.get(i,0)[1];
//            int radius = (int) circles.get(i,0)[2];
//            // circle center
//            Imgproc.circle( matLive, p, 3, new Scalar(0,255,0), -1, 8, 0 );
//            // circle outline
//            Imgproc.circle( matLive, p, radius, new Scalar(0,0,255), 3, 8, 0 );
//         }
        if(frameCount%25 == 0)
        {
//            System.out.printf("circle count %d\n", circles.rows());
        }
        //send result to server and print to output
         
        String str =    1                   +","+   //area
                        2                   +","+   //width
                        String.format("%.1f", 3.0)   +","+   //distance
                        String.format("%.1f", 4.0);                //direction
                        
        String strVerbose = "TARGET: area " + 1                        +
                            " ContourWidth " + 2                + 
                            " distW "    + String.format("%.1f", 3.0)   +
                            " VPos "     + String.format("%.1f", 4.0)           + 
                            " distVPos "    + String.format("%.1f", 5.0) +
                            " direction "   + String.format("%.1f", 6.0);
        if(SEND_TO_SOCKET_SERVER)
        {
            socClient.send(str);
        }

        if(frameCount%2 == 0)
        {
         //   System.out.println(strVerbose);
        }
        //-------- process and display code profiling -------------------------------------

        performanceTimeAvg1+=timeCodePoint2 - timeCodePoint1;
        performanceTimeAvg2+=timeCodePoint3 - timeCodePoint2;
        performanceTimeAvg3+=timeCodePoint4 - timeCodePoint3;
        performanceTimeAvg4+=timeCodePoint5 - timeCodePoint4;
        performanceTimeAvg5+=timeCodePoint6 - timeCodePoint5;
        performanceTimeAvg6+=timeCodePoint7 - timeCodePoint6;
        performanceTimeAvg7+=timeCodePoint8 - timeCodePoint7;
        performanceTimeAvg8+=timeCodePoint9 - timeCodePoint8;


        performanceTimeCounter++;
        if(performanceTimeCounter%100 == 0)
        {
            System.out.printf("Average times, mS:  %.1f %.1f %.1f %.1f    %.1f %.1f %.1f %.1f\n",performanceTimeAvg1/100,
                                                                                            performanceTimeAvg2/100,
                                                                                            performanceTimeAvg3/100,
                                                                                            performanceTimeAvg4/100,
                                                                                            performanceTimeAvg5/100,
                                                                                            performanceTimeAvg6/100,
                                                                                            performanceTimeAvg7/100,
                                                                                            performanceTimeAvg8/100);
            performanceTimeAvg1 = 0;
            performanceTimeAvg2 = 0;
            performanceTimeAvg3 = 0;
            performanceTimeAvg4 = 0;
            performanceTimeAvg5 = 0;
            performanceTimeAvg6 = 0;
            performanceTimeAvg7 = 0;
            performanceTimeAvg8 = 0;
        }   
        if(false)//frameCount%25 == 0)
        {
            System.out.println("ROI " + ROI_ballDetectedInLastFrame + " " +
                                            ROI_panningLeft + " " +
                                            ROI_lastDetectedBallCenterX + " " +
                                            ROI_lastDetectedBallCenterY + " " +                        
                                            1234 + " " +
                                            4321);
        }
        if(ENABLE_PRINT_SLIDER_VALUES && frameCount%25 == 0)
        {
            System.out.printf("Slider Values %d %d %d %d %d %d\n",  
                            gui.slide1Val, 
                            gui.slide2Val, 
                            gui.slide3Val, 
                            gui.slide4Val, 
                            gui.slide5Val,
                            gui.slide6Val);
        }
      
    }
    private static void state2(){
        long timeCodePoint1 =  System.currentTimeMillis();     
        long timeCodePoint2 =  System.currentTimeMillis();     
        long timeCodePoint3 =  System.currentTimeMillis();     
        long timeCodePoint4 =  System.currentTimeMillis();     
        long timeCodePoint5 =  System.currentTimeMillis();     
        long timeCodePoint6 =  System.currentTimeMillis();     
        long timeCodePoint7 =  System.currentTimeMillis();     
        long timeCodePoint8 =  System.currentTimeMillis();     
        long timeCodePoint9 =  System.currentTimeMillis();     
     
        Imgproc.cvtColor(matLive,matTarget,Imgproc.COLOR_BGR2HSV);
        // Imgproc.erode(matTarget,matTarget, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3)));
        // Imgproc.dilate(matTarget,matTarget, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(7,7)));
     
        /// Reduce the noise so we avoid false circle detection
    //    Imgproc.GaussianBlur( matTarget, matTarget, new Size(3, 3), 2, 2 );
    
        Scalar min = new Scalar(gui.getSlide1(),gui.getSlide3(),gui.getSlide5());
        Scalar max = new Scalar(gui.getSlide2(),gui.getSlide4(),gui.getSlide6());
      
        Scalar minLowRedOnly = new Scalar(0,gui.getSlide3(),gui.getSlide5());
        Scalar maxLowRedOnly = new Scalar(40,gui.getSlide4(),gui.getSlide6());
      
        Mat matLowRedOnly = new Mat();
        Core.inRange(matTarget, minLowRedOnly, maxLowRedOnly, matLowRedOnly);
        Core.inRange(matTarget, min, max, matTarget);
        Core.bitwise_or(matTarget, matLowRedOnly, matTarget);
        Imgproc.erode(matTarget,matTarget, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3)));
        
      
        //detect edges
        int lowThresholdCal = 100;
        int ratioCal = 3;
    //    Imgproc.Canny(matTarget, matTarget, lowThresholdCal, lowThresholdCal*ratioCal);
        
     	List<org.opencv.core.MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(matTarget, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        org.opencv.core.MatOfPoint2f matOfPoint2f = new org.opencv.core.MatOfPoint2f();
        org.opencv.core.MatOfPoint2f approxCurve = new org.opencv.core.MatOfPoint2f();
        drawText(matLive, new Point(5,25), "Contours found: " + contours.size()); 
     
        vTargs.reset();
        if(contours.size()>0)
        {
            for (int idx = 0; idx >= 0; idx = (int) hierarchy.get(0, idx)[0]) {
                org.opencv.core.MatOfPoint contour = contours.get(idx);
                Rect rect = Imgproc.boundingRect(contour);
                double contourArea = Imgproc.contourArea(contour);
                matOfPoint2f.fromList(contour.toList());
                Imgproc.approxPolyDP(matOfPoint2f, approxCurve, Imgproc.arcLength(matOfPoint2f, true) * 0.02, true);
                long total = approxCurve.total();
               
                if (total == 4 && rect.area() > 500) {
                    Point center = new Point((rect.br().x + rect.tl().x)/2,(rect.br().y + rect.tl().y)/2);
                    Point[] points = approxCurve.toArray();
                    //put a dot in each of the 4 corners of the rectangle 
                    for(int i=0;i<points.length;i++)
                    {
                        drawText(matLive, points[i], ".");
                    }
                    double angle = 0;
                    if(points.length == 4) //probably always true
                    {
                        angle = rotationOfRectangle(points[0], points[1], points[2], points[3]);
                        double angleTolerance = 5.0;
                        double angleTarget = 14.0;
                        if(Math.abs(Math.abs(angle) - angleTarget) < angleTolerance )
                        {
                            vTargs.addRectangle(center, angle); 
                            if(angle > 0)
                            {
                                drawText(matLive,center, "L");
                            }
                            else
                            {
                                drawText(matLive,center, "R");
                            }
                   
                        }
                    }
                    
                    Point drawHere1 = new Point(center.x - 25,rect.y -15);
                    Point drawHere2 = new Point(center.x - 25,rect.y );
                    drawText(matLive,drawHere1, String.format("Angle %.2f", angle));
                    drawText(matLive,drawHere2, String.format("Area  %.0f ", rect.area()));
                }
             }
        }
        drawText(matLive,new Point(5,40), "Valid Rectangles "+vTargs.getRectangleCount());
        if(vTargs.foundValidTargetRectanglePair())
        {
            drawText(matLive,vTargs.getBestTargetPoint(), "X");
        }
        //send result to server and print to output
         
        String str =    1                   +","+   //area
                        2                   +","+   //width
                        String.format("%.1f", 3.0)   +","+   //distance
                        String.format("%.1f", 4.0);                //direction
                        
        String strVerbose = "TARGET: area " + 1                        +
                            " ContourWidth " + 2                + 
                            " distW "    + String.format("%.1f", 3.0)   +
                            " VPos "     + String.format("%.1f", 4.0)           + 
                            " distVPos "    + String.format("%.1f", 5.0) +
                            " direction "   + String.format("%.1f", 6.0);
        if(SEND_TO_SOCKET_SERVER)
        {
            socClient.send(str);
        }

        if(frameCount%2 == 0)
        {
         //   System.out.println(strVerbose);
        }
        //-------- process and display code profiling -------------------------------------

        performanceTimeAvg1+=timeCodePoint2 - timeCodePoint1;
        performanceTimeAvg2+=timeCodePoint3 - timeCodePoint2;
        performanceTimeAvg3+=timeCodePoint4 - timeCodePoint3;
        performanceTimeAvg4+=timeCodePoint5 - timeCodePoint4;
        performanceTimeAvg5+=timeCodePoint6 - timeCodePoint5;
        performanceTimeAvg6+=timeCodePoint7 - timeCodePoint6;
        performanceTimeAvg7+=timeCodePoint8 - timeCodePoint7;
        performanceTimeAvg8+=timeCodePoint9 - timeCodePoint8;


        performanceTimeCounter++;
        if(performanceTimeCounter%100 == 0)
        {
            System.out.printf("Average times, mS:  %.1f %.1f %.1f %.1f    %.1f %.1f %.1f %.1f\n",performanceTimeAvg1/100,
                                                                                            performanceTimeAvg2/100,
                                                                                            performanceTimeAvg3/100,
                                                                                            performanceTimeAvg4/100,
                                                                                            performanceTimeAvg5/100,
                                                                                            performanceTimeAvg6/100,
                                                                                            performanceTimeAvg7/100,
                                                                                            performanceTimeAvg8/100);
            performanceTimeAvg1 = 0;
            performanceTimeAvg2 = 0;
            performanceTimeAvg3 = 0;
            performanceTimeAvg4 = 0;
            performanceTimeAvg5 = 0;
            performanceTimeAvg6 = 0;
            performanceTimeAvg7 = 0;
            performanceTimeAvg8 = 0;
        }   
        if(false)//frameCount%25 == 0)
        {
            System.out.println("ROI " + ROI_ballDetectedInLastFrame + " " +
                                            ROI_panningLeft + " " +
                                            ROI_lastDetectedBallCenterX + " " +
                                            ROI_lastDetectedBallCenterY + " " +                        
                                            1234 + " " +
                                            4321);
        }
        if(ENABLE_PRINT_SLIDER_VALUES && frameCount%25 == 0)
        {
            System.out.printf("Slider Values %d %d %d %d %d %d\n",  
                            gui.slide1Val, 
                            gui.slide2Val, 
                            gui.slide3Val, 
                            gui.slide4Val, 
                            gui.slide5Val,
                            gui.slide6Val);
        }
      
    }
  
    private static void state3(){
        long timeCodePoint1 =  System.currentTimeMillis();     
       
        //Scan ROI when no object detected; Move towards 0 pan and tilt when object is detected
        double panCenter = ROI_lastDetectedBallCenterX - ROI_colSpan/4;
        double tiltCenter = ROI_lastDetectedBallCenterY - ROI_rowSpan/4;
        

        //logic for pan scan and track ball in horizontal axis
        if((ROI_enablePanScan && (ROI_panningLeft && !ROI_ballDetectedInLastFrame) || (ROI_ballDetectedInLastFrame && panCenter < -ROI_centerXThreshold)))
        {
            if(!ROI_ballDetectedInLastFrame) //handle edge case
            {
                ROI_panningLeft = !changeROI(-5,0);
            }
            else
            {
                ROI_panningLeft = !changeROI((int)panCenter,0);  //fast track -pan left by 6 pixel each frame
                                                                 //  and change pan scan direction when limit is hit
            }
        }                       
        if((ROI_enablePanScan && (!ROI_panningLeft &&  !ROI_ballDetectedInLastFrame) || (ROI_ballDetectedInLastFrame && panCenter > ROI_centerXThreshold)))
        {
            if(!ROI_ballDetectedInLastFrame) //handle edge case
            {
                ROI_panningLeft = changeROI(5,0);
            }
            else
            {
                ROI_panningLeft = changeROI((int)panCenter,0);   //fast track, Pan right  each frame
                                                                 // and change pan scan direction when limit is hit
            }
        }
        //logic to track ball  vertically
        if(ROI_enableTiltTrack && ROI_ballDetectedInLastFrame && ROI_lastDetectedBallDistance < 12.0)//drop ROI to lowest point when ball is inside 12"
        {
                changeROI(0,123456);  //drop ROI... changeROI will limit 
        }
        else
        {
            if(ROI_enableTiltTrack && ROI_ballDetectedInLastFrame && (tiltCenter < -ROI_centerYThreshold_TiltUp))
            {
                changeROI(0,(int)tiltCenter);        //fast track
            }                       
            if(ROI_enableTiltTrack && ROI_ballDetectedInLastFrame && (tiltCenter > ROI_centerYThreshold_TiltDown))
            {
                changeROI(0,(int)tiltCenter);       //fast track               
            }
        }
        //logic to return tilt to default view, just below horizon (most likely view to find a ball)
        if(ROI_enableTiltTrack && !ROI_ballDetectedInLastFrame && ROI_Tilt < ROI_Tilt_Default)//looking up?
        {
            changeROI(0,1);                    //tilt back down by 1 pixel each frame 
                                               // Don't go too fast for some immunity to not detecting a ball in a couple frames)
        }
        if(ROI_enableTiltTrack && !ROI_ballDetectedInLastFrame && ROI_Tilt > ROI_Tilt_Default)//looking down ?
        {
            changeROI(0,-1);                     //tilt back up by 1 pixels each frame
        }
        
        // Imgproc.GaussianBlur(matHSV, matHSV, new Size(9, 9), 0);
        // src(Rect(left,top,width, height)).copyTo(dst); //copy rio to another mat
        // src.copyTo(dst(Rect(left, top, src.cols, src.rows))); //copy mat to RIO of another mat
        // matLive.copyTo(matLiveOrg);
        double[] putPix = new double[3];
        int detectedPixels = 0;
        int maxPixelX = 0;
        int minPixelX = WIDTH/2 + ROI_colSpan/2;
      
        //todo- Account for y center f blob as part of distance calc... probably more accurate
        //todo - add both red ranges
        List<Mat> list = new ArrayList<Mat>();
        org.opencv.core.Rect roi = new Rect(ROI_left,ROI_top,ROI_colSpan, ROI_rowSpan);//left, top, w,h
        Mat matTemp = matLive.submat(roi); //share ROI with another mat
        matTemp.copyTo(matROI); 
      
        // roiP1 will be used for a sub mat with 1/2 as many rows and 1/2 as many columns so we can skip every other row and col to reduce CPU load.
        // This mat will be 1/4 as large but cover the same area as the original ROI
        // matROI_processed1 and matROI_processed2 will have this reduced pixel format
        org.opencv.core.Rect roiP1 = new Rect(0,0,ROI_colSpan/2, ROI_rowSpan/2);
        Mat matTemp2 = matROI.submat(roiP1); //share ROI with another mat
      
        matTemp2.copyTo(matROI_processed1);
        matTemp2.copyTo(matROI_processed2);
        long timeCodePoint2 =  System.currentTimeMillis();     
        
       // matTemp.copyTo(matROI_processed3);
        Imgproc.cvtColor(matROI,matHSV,Imgproc.COLOR_BGR2HSV_FULL);
        // src.copyTo(dst(Rect(left, top, src.cols, src.rows))); //copy mat to RIO of another mat
        long timeCodePoint3 =  System.currentTimeMillis();     
            
        //draw green rectangle around ROI on live image
        Point p1 = new Point(ROI_left,ROI_top);
        Point p2 = new Point(ROI_left+ROI_colSpan,ROI_top+ROI_rowSpan);
        Scalar s = new Scalar(0,255,0);
        Imgproc.rectangle(matLive, p1, p2,  s);
       
        //---- filter ROI ------------------------------------------------
        //  filter on color                       (uses matROI which is still BGR format)
        //  filter on red/(green + blue)          (uses matROI which is still BGR format)
        //  filter on Lumannce values in range    (uses matHSV which is in HSV format
        //put the filtered output in matROI_processed1
        
        int slide1 = gui.slide1Val;
        int slide2 = gui.slide2Val;
        int slide3 = gui.slide3Val;
        int slide4 = gui.slide4Val;
        int slide5 = gui.slide5Val;
        int slide6 = gui.slide6Val;
        //matROI_processed1.setTo(Scalar.all(0.0));
   
        long timeCodePoint4 =  System.currentTimeMillis();     
        for(int row = 0; row < ROI_rowSpan; row+=2 )
        {
            for(int col = 0; col< ROI_colSpan; col+=2 )
            {
                double pixVal = 0;
                double[] scanPixelGBR = matROI.get(row, col);
                double[] scanPixelHSV = matHSV.get(row, col);
                if( scanPixelHSV[0] > slide1 && scanPixelHSV[0] < slide2 &&   //color range
                    scanPixelHSV[2] > slide3 && scanPixelHSV[2] < slide4)     //V range
                {
                    pixVal =  slide5 * scanPixelGBR[2]/(scanPixelGBR[1] + scanPixelGBR[0]); //scaler * red /(blue + green)
                    if(pixVal < slide6) // threshold
                    {
                        pixVal = 0;
                    }
                    else
                    {
                        detectedPixels++;
                        if(col > maxPixelX )
                        {
                            maxPixelX = col;
                        }
                        if(col < minPixelX)
                        {
                            minPixelX = col;
                        }
                    }
                }
                putPix[0] = putPix[1] = putPix[2] = pixVal;
                matROI_processed1.put(row/2, col/2, putPix);
                //matROI_processed1.put(row, col+1, putPix);
            }   
        }
        long timeCodePoint5 =  System.currentTimeMillis();     
        //matROI_processed1 is the gray scale processed red ball
        //---------------erode and dialate to reduce noise--------------------------------------
       // 3,3 erode and 7,7 dilate was found to be reliable - 1,1 and 3,3 more sensitive but not proven under different lighting conditions
        Imgproc.erode(matROI_processed1,matROI_processed2, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3)));
        Imgproc.dilate(matROI_processed2,matROI_processed2, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(7,7)));
       // Imgproc.erode(matROI_processed1,matROI_processed2, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(1,1)));
       // Imgproc.dilate(matROI_processed2,matROI_processed2, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3,3)));
       // Imgproc.dilate(matROI_processed2,matROI_processed2, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(9,9)));
        long timeCodePoint6 =  System.currentTimeMillis();     

        //---------------find the larget blob which is probably the red ball-------------------------------------------------
        	List<org.opencv.core.MatOfPoint> contours = new ArrayList<>();
        	Imgproc.cvtColor(matROI_processed2, matROI_processed2, Imgproc.COLOR_BGR2GRAY);
        Imgproc.findContours(matROI_processed2, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        int largestContourIndex = -1;
        double largestCoutourArea = -1;
        int countourIndex = -1;
        long timeCodePoint7 =  System.currentTimeMillis();     
        for(countourIndex=0; countourIndex < contours.size(); countourIndex++) 
	{
            if(Imgproc.contourArea(contours.get(countourIndex)) > largestCoutourArea)
            {
                largestCoutourArea = Imgproc.contourArea(contours.get(countourIndex));
                largestContourIndex = countourIndex;
            }
            
	}
        long timeCodePoint8 =  System.currentTimeMillis();     
        Rect rectLargetContour = new Rect(0,0,100,100);
        if(contours.size()>0)
        {
            //draw rectangle around largest countour
            rectLargetContour = Imgproc.boundingRect(contours.get(largestContourIndex));
            Imgproc.rectangle(matROI_processed2, rectLargetContour.tl(), rectLargetContour.br(), new Scalar(255, 0, 0), 1);
        }

        if(ENABLE_GUI)
        {
            Imgproc.cvtColor(matROI_processed2, matROI_processed2, Imgproc.COLOR_GRAY2BGR);//list needs to be all same format
            //list.add(matROI);
            //list.add(matSpacer);
            list.add(matROI_processed1);
            //list.add(matSpacer);
            list.add(matROI_processed2);
            Core.vconcat(list, matROI); //display 
        }
        
        if(ENABLE_PRINT_SLIDER_VALUES && frameCount%25 == 0)
        {
            System.out.printf("Slider Values %d %d %d %d %d %d\n",  
                            gui.slide1Val, 
                            gui.slide2Val, 
                            gui.slide3Val, 
                            gui.slide4Val, 
                            gui.slide5Val,
                            gui.slide6Val);
        }
        //send result to server and print to output
        double widthOfLargestContour=rectLargetContour.br().x  - rectLargetContour.tl().x;
         
        if(largestCoutourArea <25)
        {
            ROI_lastDetectedBallCenterX=0;
            ROI_lastDetectedBallCenterY=0;
            ROI_ballDetectedInLastFrame = false;
       
            String str =detectedPixels                          +","+   //area
                        widthOfLargestContour                   +","+   //width
                        -1                                      +","+   //distance
                        0;                                             //direction
            if(SEND_TO_SOCKET_SERVER)
            {
                socClient.send(str);
            }
        }
        else//only send reasonably sized detected objects... others are probably noise or far away
        { 
            double centerX =  (rectLargetContour.br().x + rectLargetContour.tl().x)/2 ;
            double centerY = (rectLargetContour.br().y  + rectLargetContour.tl().y)/2 ;
            
            //turn on LED depending on left or right of center  
            double direction = ROI_colSpan/4 - centerX - ROI_Pan/2; // extra div by 2 id because of scan every other line
            ROI_lastDetectedBallCenterX = centerX;
            ROI_lastDetectedBallCenterY = centerY;
            
            ROI_ballDetectedInLastFrame = true;
            if(direction > 0)
            {
               leds.setHigh(3);
               leds.setLow(4);
            }
            else
            {
               leds.setHigh(4);
               leds.setLow(3);
            }
            double distanceInchesW = distanceUsingWidth(widthOfLargestContour);
            double distanceInchesVPos = distanceUsingVerticalPosition(centerY);
            ROI_lastDetectedBallDistance = distanceInchesW;
            
            String str =detectedPixels                          +","+   //area
                        widthOfLargestContour                   +","+   //width
                        String.format("%.1f", distanceInchesW)   +","+   //distance
                        String.format("%.1f", direction);                //direction
                        
            String strVerbose = "TARGET: area " + detectedPixels                        +
                                " ContourWidth " + widthOfLargestContour                + 
                                " distW "    + String.format("%.1f", distanceInchesW)   +
                                " VPos "     + String.format("%.1f", centerY)           + 
                                " distVPos "    + String.format("%.1f", distanceInchesVPos) +
                                " direction "   + String.format("%.1f", direction);
            if(SEND_TO_SOCKET_SERVER)
            {
                socClient.send(str);
            }
            
            if(frameCount%2 == 0)
            {
                System.out.println(strVerbose);
            }
            long timeCodePoint9 =  System.currentTimeMillis();     
        
            
            //-------- process and display code profiling -------------------------------------
            
            performanceTimeAvg1+=timeCodePoint2 - timeCodePoint1;
            performanceTimeAvg2+=timeCodePoint3 - timeCodePoint2;
            performanceTimeAvg3+=timeCodePoint4 - timeCodePoint3;
            performanceTimeAvg4+=timeCodePoint5 - timeCodePoint4;
            performanceTimeAvg5+=timeCodePoint6 - timeCodePoint5;
            performanceTimeAvg6+=timeCodePoint7 - timeCodePoint6;
            performanceTimeAvg7+=timeCodePoint8 - timeCodePoint7;
            performanceTimeAvg8+=timeCodePoint9 - timeCodePoint8;
            
            
            performanceTimeCounter++;
            if(performanceTimeCounter%100 == 0)
            {
                System.out.printf("Average times, mS:  %.1f %.1f %.1f %.1f    %.1f %.1f %.1f %.1f\n",performanceTimeAvg1/100,
                                                                                                performanceTimeAvg2/100,
                                                                                                performanceTimeAvg3/100,
                                                                                                performanceTimeAvg4/100,
                                                                                                performanceTimeAvg5/100,
                                                                                                performanceTimeAvg6/100,
                                                                                                performanceTimeAvg7/100,
                                                                                                performanceTimeAvg8/100);
                performanceTimeAvg1 = 0;
                performanceTimeAvg2 = 0;
                performanceTimeAvg3 = 0;
                performanceTimeAvg4 = 0;
                performanceTimeAvg5 = 0;
                performanceTimeAvg6 = 0;
                performanceTimeAvg7 = 0;
                performanceTimeAvg8 = 0;
            }
        }   
        if(false)//frameCount%25 == 0)
        {
            System.out.println("ROI " + ROI_ballDetectedInLastFrame + " " +
                                            ROI_panningLeft + " " +
                                            ROI_lastDetectedBallCenterX + " " +
                                            ROI_lastDetectedBallCenterY + " " +                        
                                            panCenter + " " +
                                            tiltCenter);
        }
    }
    private static void stateScratch(){
        System.out.printf("scratch %d %d %d %d %d %d %d\n", frameCount , 
                            gui.slide1Val, 
                            gui.slide2Val, 
                            gui.slide3Val, 
                            gui.slide4Val, 
                            gui.slide5Val,
                            gui.slide6Val);
        Imgproc.cvtColor(matLive,matHSV,Imgproc.COLOR_BGR2HSV_FULL);
    //    matLive.copyTo(matLiveOrg);
        double[] putPix = new double[3];
        int detectedPixels = 0;
        int rowSpan = 200;//pix height of detection area
        int colSpan = 500;//pix width of detection area
        int maxPixelX = 0;
        int minPixelX = 640/2 + colSpan/2;
        for(int row = 480 - rowSpan; row < 480; row++ )
        {
            for(int col = 640/2-colSpan/2; col< 640/2+colSpan/2; col++ )
            {
                double[] scanPixelGBR = matLive.get(row, col);
                double[] scanPixelHSV = matHSV.get(row, col);

                if(scanPixelHSV[2] < gui.slide3Val || scanPixelHSV[2] > gui.slide4Val) //cut very high/low V areas
                {
                    putPix[0] = putPix[1] = putPix[2] = 0;
                }
                else
                {
                    double hueMult = 0; //scale for hue in range
                    if(scanPixelHSV[0] > gui.slide1Val && scanPixelHSV[0] < gui.slide2Val) //red is 200 - 255
                    {
                        hueMult = gui.slide5Val;
                    }
                    double pixVal =  hueMult * scanPixelGBR[2]/(scanPixelGBR[1] + scanPixelGBR[0]); //hueMult * red /(blue + green)
                    if(pixVal < gui.slide6Val) // threshold
                    {
                        pixVal = 0;
                    }
                    else
                    {
                        detectedPixels++;
                        //leds.toggle(1);
                        if(col > maxPixelX)
                        {
                            maxPixelX = col;
                        }
                        if(col < minPixelX)
                        {
                            minPixelX = col;
                        }

                    }
                    putPix[0] = putPix[1] = putPix[2] = pixVal;
                }
                matLive.put(row, col, putPix);
            }   
        }
        if(detectedPixels>10)
        {
           int center = 320 - (maxPixelX+minPixelX)/2;
           if(center > 0)
           {
               leds.setHigh(3);
               leds.setLow(4);
           }
           else
           {
               leds.setHigh(4);
               leds.setLow(3);
           }
           // System.out.printf("TARGET: area %d width %d direction %d\n", detectedPixels, maxPixelX-minPixelX, 320 - (maxPixelX+minPixelX)/2);
        }
    }
    
    static private double distanceUsingWidth(double pixelWidth)
    {
        //SOH CAH TOA
        //To use TOA, the ball width would be twice the oppisite side.
        //The adjacent side is our distance that we want to calculate.
        //tan(theta) = opposite/adjacent
        //adjacent = opposite / tan(theta)
        double theta = Math.toRadians(H_FOV) * pixelWidth / WIDTH;   
        
        return DISTANCE_FUDGE * (RED_BALL_DIAMETER/2.0) / Math.tan(theta);
    }
    static private double distanceUsingVerticalPosition(double verticalPosition)
    {
        //todo: figure out correct function using FOV and camera position
        //for now just fill in something 
        //vertical picxel position     distance
        //0                            3
        //10                           6
        //20                            12.5                           
        //30                            25
        //40                            50
        //50                            100
        //60                            infinity
        double distance = verticalPosition;     
        return distance;
    }
    
    static boolean changeROI(int deltaPan, int deltaTilt)
    {
         return setROI(ROI_Pan + deltaPan, ROI_Tilt + deltaTilt);
    }
    
    static boolean setROI(int pan, int tilt)
    {
        boolean limitReached = false;
        ROI_Pan = pan;
        ROI_Tilt = tilt;
                
        ROI_left = WIDTH/2  - ROI_colSpan/2 + ROI_Pan;
        ROI_top =  HEIGHT/2 - ROI_rowSpan/2 + ROI_Tilt;
        
        //limit negative pan
        if(ROI_left < 0 )
        {
            ROI_left = 0;
            ROI_Pan = ROI_colSpan/2 - WIDTH/2;
            limitReached = true;
            //System.out.println("neg pan lim");
        }
        //limit positive pan 
        if(ROI_left + ROI_colSpan >= WIDTH)
        {
            ROI_left = WIDTH - ROI_colSpan - 1;
            ROI_Pan  = ROI_left - WIDTH/2 + ROI_colSpan/2;
            limitReached = true;
            //System.out.println("pos pan lim");
        }
        //limit negative tilt
        if(ROI_top < 0 )
        {
            ROI_top= 0;
            ROI_Tilt = ROI_rowSpan/2 - HEIGHT/2;
            limitReached = true;
            //System.out.println("neg tilt lim");
        }
        //limit positive tilt
        if(ROI_top + ROI_rowSpan >= HEIGHT)
        {
            ROI_top  = HEIGHT - ROI_rowSpan - 1;
            ROI_Tilt = ROI_top - HEIGHT/2 + ROI_rowSpan/2;
            limitReached = true;
            //System.out.println("pos tilt lim");
        }
        return limitReached;
    }
    
    private static double rotationOfRectangle(Point p1, Point p2, Point p3, Point p4)
    {
        //starting with arbitrary point, p1, find its 2nd closest neighbor
        //This is the other point on the same long side of the rectangle.
        //This point will provide more resolution than its 1st closest neighbor
        Point p2ndClosest = new Point();
        double distanceP1P2 = distanceBetween(p1,p2);
        double distanceP1P3 = distanceBetween(p1,p3);
        double distanceP1P4 = distanceBetween(p1,p4);
        if((distanceP1P2 > distanceP1P3 || distanceP1P2 > distanceP1P4) &&      //p2 not closest AND
           (distanceP1P2 < distanceP1P3 || distanceP1P2 < distanceP1P4))        //p2 not farthest (diagonal point)
        {
            p2ndClosest.x = p2.x;                                                  // p2 must be 2nd closest
            p2ndClosest.y = p2.y;
        }
        else if((distanceP1P3 > distanceP1P2 || distanceP1P3 > distanceP1P4) && //p3 not closest AND
                (distanceP1P3 < distanceP1P2 || distanceP1P3 < distanceP1P4))   //p3 not farthest (diagonal point)
        {
            p2ndClosest.x = p3.x;                                                  // p3 must be 2nd closest
            p2ndClosest.y = p3.y;
        }
        else
        {
            p2ndClosest.x = p4.x;                                                  // p4 must be 2nd closest
            p2ndClosest.y = p4.y;
        }
        //toa           Pixel values are whole doubles so we can add a tiny value to insure no div by zero error
        double angle = 90 - DEGREES_PER_RADIAN * Math.atan(Math.abs(p1.y-p2ndClosest.y) / (Math.abs(p1.x-p2ndClosest.x)+0.00001)); 
        //invert angle if p1 is lower and to the left or upper and to the right of the 2nd point 
        if((p1.y > p2ndClosest.y && p1.x < p2ndClosest.x) || (p1.y < p2ndClosest.y && p1.x > p2ndClosest.x))
        {
            angle = -angle;
        }
       // angle = angle + 90; //a result of using long side rather than short side
        return angle;
    }
    private static double distanceBetween(Point p1, Point p2)
    {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return Math.sqrt(dx*dx + dy*dy);
    }
    
    private static void drawText(Mat matToDrawOn, Point ofs, String text) {
        Imgproc.putText(matToDrawOn, text, ofs, Core.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255,255,25));
    }
    
    
    private static void loadOpenCV(){
                //------ print some info about the OpenCV lib and load it ----------
        String libPath = System.getProperty("java.library.path");
        System.out.println("java lib path is: " + libPath);
        System.out.println("OpenCV core lib name is: " + Core.NATIVE_LIBRARY_NAME);
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME); //why is this set to opencv_java342 not 343 ?
        //System.loadLibrary("opencv_java343");
    }
}// close VisionPi class

//code snipits we might need some day
//
//                Imgproc.circle(matLive, new Point(width/2, height/2), 75, new Scalar(255, 255, 255), 3);
//                    Imgproc.GaussianBlur(matBall, matBall, new Size(9, 9), 0);
//                    double[] centerPixel = matBall.get(480/2, 680/2);
//                    targetHue += centerPixel[0];
//                    targetHueSamples++;
//                    Imgproc.circle(matLive, new Point(width/2, height/2), 30, new Scalar(255, 255, 255), 3); //steady circle
//                    System.out.printf("center pix %d  %d  %d\n", (int)centerPixel[0],(int)centerPixel[1],(int)centerPixel[2]);
//                    if(centerPixel[0] > highestHueVal)
//                    {
//                        highestHueVal = (int) centerPixel[0];
//                    }
//                    if(centerPixel[0] < lowestHueVal)
//                    {
//                        lowestHueVal = (int) centerPixel[0];
//                    }
//                }
//                Imgproc.circle(matLive, new Point(width/2, height/2), 30, new Scalar(255, 255, 255), 3); //steady circle
//                System.out.printf("target hue %d  samples %d  low %d   high %d\n", targetHue, targetHueSamples, lowestHueVal, highestHueVal);
//                Core.inRange(matBall, new Scalar(controls.slide1Val,controls.slide3Val,controls.slide5Val), new Scalar(controls.slide2Val,controls.slide4Val,controls.slide6Val), matBall);
//                Core.bitwise_and(matBall, matLive, matBall, matTemp);//src1,src2, dest, mask
//                Imgproc.erode(matBall, matBall, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(9,9)));
//                Imgproc.dilate(matBall, matBall, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2,2)));
                //List<Mat> lBGR = new ArrayList<Mat>(3);
                //List<Mat> lHSV = new ArrayList<Mat>(3);
                //Core.split(matLive, lHSV);
                //Core.split(matLive, lBGR);
                //Mat mB = lBGR.get(0);
                //Mat mG = lBGR.get(1);
                //Mat mR = lBGR.get(2);
                //Mat mOut = new Mat();                
