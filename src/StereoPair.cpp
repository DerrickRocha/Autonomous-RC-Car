

#include "StereoPair.h"
#include "opencv2/gpu/gpu.hpp"
#include "CommonMethods.h"
//#include "DUO3D_camera.h"
#include <string>

//————————————————————————————————————————————————————————————————————
// Default initializer
//————————————————————————————————————————————————————————————————————

StereoPair::StereoPair(){
    StereoPair(640, 480, 30, "");
}


//————————————————————————————————————————————————————————————————————
// Initializer
//————————————————————————————————————————————————————————————————————

StereoPair::StereoPair(int width, int height, int fps, string _dataDirectory){
    dataDirectory = _dataDirectory;
    imageWidth = width;
    imageHeight = height;
    maximumDepth = -1.0; // Default: not apply filter when displaying point cloud;

    // Setting some defaults
    useColorImages = false;
    flippedUpsideDown = false;
    swapped = false;
    canRectify = false;
    rectify = false;
    calibration_ParametersFileName = "stereo_calibration_parameters.xml";
    sgbm_ParametersFileName = "semiglobal_block_match_parameters.xml";
    calibration_boardSize = Size(9, 6);
    calibration_squareSize = 0.022;
    calibration_numberOfImages = 16;
    //Open and configure cameras
    webcam.left = VideoCapture();
    webcam.right = VideoCapture();
    if(!webcam.left.open(1) || !webcam.right.open(0)){
        cout << "\n*******CAMERA INITIALIZATION ERROR******" << endl;
        exit(EXIT_FAILURE);
    }
    
    // Setup rectification parameters and rectification maps
    setupRectification();
	//Setup Semi Global Block Matching object
    setupDisparityParameters();
}

//————————————————————————————————————————————————————————————————————
//  setupRectification
//————————————————————————————————————————————————————————————————————

void StereoPair::setupRectification() {
    Mat R1, R2, P1, P2;
    Mat rmap[2][2];
    Rect validRoi[2];
    
    // Open calibration file
    string calibration_parametersFile = dataDirectory + calibration_ParametersFileName;
    FileStorage fs(calibration_parametersFile, FileStorage::READ);
    
    // Read calibration file
    if(!fs.isOpened()){
        cout << "READ ERROR: Calibration file was not found" << endl;
        cout << "The provided file path is: " + calibration_parametersFile << endl;
        cout << "Do you want to calibrate now? (y/n): " << endl;
        return;
    }
    else{
        Size imageSize;
        Mat cameraMatrix0, cameraMatrix1, distCoeffs0, distCoeffs1, imgSize, RInitial, TInitial;
        fs["Size"] >> imageSize;
        fs["K1"] >> cameraMatrix0;
        fs["distCoeffs1"] >> distCoeffs0;
        fs["K2"] >> cameraMatrix1;
        fs["distCoeffs1"] >> distCoeffs1;
        fs["R"] >> RInitial;
        fs["T"] >> TInitial;
        
        // Compute rectification mappings
        stereoRectify(cameraMatrix0, distCoeffs0, cameraMatrix1, distCoeffs1, imageSize, RInitial, TInitial, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, 0, imageSize, &validRoi[0], &validRoi[1]);
        initUndistortRectifyMap(cameraMatrix0, distCoeffs0, R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
        initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);
        
        
        // Save the rectification mappings
        rectification.K = P1;
        rectification.B = P2.at<double>(0, 3) / P2.at<double>(0, 0);
        for(int i=0; i<2; i++)
            for(int j=0; j<2; j++)
                rectification.rmap[i][j] = rmap[i][j];
        canRectify = true;
    }
}

//————————————————————————————————————————————————————————————————————
//  setupDisparityParameters
//————————————————————————————————————————————————————————————————————

void StereoPair::setupDisparityParameters() {
    string sgbmParametersFile = dataDirectory + sgbm_ParametersFileName;
    FileStorage fs(sgbmParametersFile.c_str(), FileStorage::READ);
    if (fs.isOpened()) {
        fs["SADWindowSize"]         >> semiGlobalBlobMatch.SADWindowSize;
        fs["numberOfDisparities"]   >> semiGlobalBlobMatch.numberOfDisparities;
        fs["preFilterCap"]          >> semiGlobalBlobMatch.preFilterCap;
        fs["minDisparity"]          >> semiGlobalBlobMatch.minDisparity;
        fs["uniquenessRatio"]       >> semiGlobalBlobMatch.uniquenessRatio;
        fs["speckleWindowSize"]     >> semiGlobalBlobMatch.speckleWindowSize;
        fs["speckleRange"]          >> semiGlobalBlobMatch.speckleRange;
        fs["disp12MaxDiff"]         >> semiGlobalBlobMatch.disp12MaxDiff;
        fs["fullDP"]                >> semiGlobalBlobMatch.fullDP;
        fs["P1"]                    >> semiGlobalBlobMatch.P1;
        fs["P2"]                    >> semiGlobalBlobMatch.P2;
    }
    else {
        cout << "READ ERROR: could not read from " << sgbmParametersFile << endl;
        cout << "The provided file path is: " + dataDirectory + sgbm_ParametersFileName << endl;
        cout << "using default parameters" << endl;
        semiGlobalBlobMatch.SADWindowSize = 5;
        semiGlobalBlobMatch.numberOfDisparities = 192;
        semiGlobalBlobMatch.preFilterCap = 9;
        semiGlobalBlobMatch.minDisparity = 5;
        semiGlobalBlobMatch.uniquenessRatio = 18;
        semiGlobalBlobMatch.speckleWindowSize = 83;
        semiGlobalBlobMatch.speckleRange = 95;
        semiGlobalBlobMatch.disp12MaxDiff = 25;
        semiGlobalBlobMatch.fullDP = true;
        semiGlobalBlobMatch.P1 = 240;
        semiGlobalBlobMatch.P2 = 2339;
    }

}

//————————————————————————————————————————————————————————————————————
//  displayDisparityMap
//————————————————————————————————————————————————————————————————————

void StereoPair::displayDisparityMap() {
    namedWindow("Disparity", CV_WINDOW_AUTOSIZE);
    
    //  Create the Controls window
    namedWindow("Controls", CV_WINDOW_NORMAL);
    Mat controlsBackground(1, 450, leftImage.type(), Scalar(150));
    imshow("Controls", controlsBackground);
    createTrackbar("SADWindowSize", "Controls", &semiGlobalBlobMatch.SADWindowSize, 50);
    //  createTrackbar("numberOfDisparities", "Controls", &sgbm.numberOfDisparities, 1000);
    createTrackbar("preFilterCap", "Controls", &semiGlobalBlobMatch.preFilterCap, 100);
    createTrackbar("minDisparity", "Controls", &semiGlobalBlobMatch.minDisparity, 100);
    createTrackbar("uniquenessRatio", "Controls", &semiGlobalBlobMatch.uniquenessRatio, 100);
    createTrackbar("speckleWindowSize", "Controls", &semiGlobalBlobMatch.speckleWindowSize, 300);
    createTrackbar("speckleRange", "Controls", &semiGlobalBlobMatch.speckleRange, 100);
    createTrackbar("disp12MaxDiff", "Controls", &semiGlobalBlobMatch.disp12MaxDiff, 100);
    createTrackbar("P1", "Controls", &semiGlobalBlobMatch.P1, 3000);
    createTrackbar("P2", "Controls", &semiGlobalBlobMatch.P2, 10000);
    
    //int whiteThreshold = 255;
    int frameCount = 0;
    while(1){
	    updateImages();
            //updateDisparityImg(0.4);
         Mat scaledLeftImage, scaledRightImage;
        int interpolationMethod = INTER_AREA;
        float scaleFactor = 0.4;
        //resize(leftImage, scaledLeftImage, Size(), scaleFactor, scaleFactor, interpolationMethod);
       // resize(rightImage, scaledRightImage, Size(), scaleFactor, scaleFactor, interpolationMethod);
     
          // Set common parameters
        bm.ndisp = 128;
        bp.ndisp = 16;
        csbp.ndisp = 0;
  // Prepare disparity map of specified type
        Mat disp(leftImage.size(), CV_8U);
        gpu::GpuMat d_disp(leftImage.size(), CV_8U);
	d_left.upload(leftImage);
        d_right.upload(rightImage);
          if (d_left.channels() > 1 || d_right.channels() > 1)
            {
                cout << "BM doesn't support color images\n";
               
            }
        bm(d_left, d_right, d_disp);
            //bp(d_left, d_right, d_disp);
        d_disp.download(disparityMap);
         Mat disparityMapNormalised = getDisparityImageNormalised();

                imshow("Disparity", disparityMapNormalised);
 // Exit when esc key is pressed
// Wait for key press
        int keyPressed = int(char(waitKey(20)));
        if( keyPressed== 27) break;
    }
    
   /* while(1){
        updateImages();
        updateDisparityImg(0.4);

        Mat disparityMapNormalised = getDisparityImageNormalised();
        cvtColor(disparityMapNormalised, disparityMapNormalised, COLOR_GRAY2BGR);
        
        // Draw available commands on the bottom-left corner of the window
        string message = "s: Save images                  d: Launch point cloud viewer";
        putText(disparityMapNormalised, message, Point(10, disparityMapNormalised.rows - 30), FONT_HERSHEY_COMPLEX_SMALL, 0.7, CV_RGB(255, 123, 47), 1, CV_AA);
        message = "x: Save current parameters    ESC: Close mode";
        putText(disparityMapNormalised, message, Point(10, disparityMapNormalised.rows - 10), FONT_HERSHEY_COMPLEX_SMALL, 0.7, CV_RGB(255, 123, 47), 1, CV_AA);
        
        imshow("Disparity", disparityMapNormalised);
        
        // Wait for key press
        int keyPressed = int(char(waitKey(20)));
        
        // Run point cloud visualizer if 'd' or 'D' keys are pressed
        if(keyPressed== 68 || keyPressed==100) {
                cout << "should be running point cloud biatch" << endl;
         //   updateImage3D();
         //   displayImage3D();
        }
        
        // Save the images if required (press 's' or 'S')
        else if(keyPressed== 83 || keyPressed==115) {
         //   saveImage(disparityMapNormalised, "DepthMap_" + std::to_string(frameCount), dataDirectory);
         //   frameCount++;
        }
        
        // Save disparity parameters (press 'x' or 'X')
        else if(keyPressed==120 || keyPressed==88){
         //   saveDisparityParameters();
        }
        
        // Exit when esc key is pressed
        if( keyPressed== 27) break;
    }*/

    destroyWindow("Disparity");
    destroyWindow("Controls");
    for(int i = 0; i < 10; i++) waitKey(1); // In some systems, if this is not included windows may becmome unresponsive.
}

//————————————————————————————————————————————————————————————————————
//  updateImages
//————————————————————————————————————————————————————————————————————

void StereoPair::updateImages() {
    Mat newFrameL, newFrameR;
    

    //First grab the undecoded frames, as this is a fast operation and thus the delay between captures will be lower
    assert(webcam.left.grab());
    assert(webcam.right.grab());
    assert(webcam.left.retrieve(newFrameL));
    assert(webcam.right.retrieve(newFrameR));
    if (!useColorImages) {
        cvtColor(newFrameL,newFrameL,CV_RGB2GRAY);
        cvtColor(newFrameR,newFrameR,CV_RGB2GRAY);
    }

    if (flippedUpsideDown) {
        Mat tmpL, tmpR;
        flip(newFrameL, tmpL, -1);  // Flip vertically
        flip(newFrameR, tmpR, -1);  // Flip vertically
        newFrameL = tmpR;   // Since we are upsidedown, now left is right
        newFrameR = tmpL;   // Since we are upsidedown, now right is left
        //flip(newFrameL, newFrameL, 1);  // Flip horizontally
        //flip(newFrameR, newFrameR, 1);  // Flip horizontally
    }
   
    rightImage = swapped? newFrameL : newFrameR;
    leftImage = swapped? newFrameR : newFrameL;
    
    if (rectify && canRectify) {
        remap(leftImage, leftImage,   rectification.rmap[0][0], rectification.rmap[0][1], CV_INTER_LINEAR, BORDER_TRANSPARENT);
        remap(rightImage, rightImage, rectification.rmap[1][0], rectification.rmap[1][1], CV_INTER_LINEAR, BORDER_TRANSPARENT);
    }
}

//————————————————————————————————————————————————————————————————————
//  updateDisparityImg
//————————————————————————————————————————————————————————————————————

void StereoPair::updateDisparityImg(float scaleFactor){
    const int numThreads = numberOfAvailableThreads();
    
    if(scaleFactor != 1.0){
        Mat scaledLeftImage, scaledRightImage;
        int interpolationMethod = INTER_AREA;

        resize(leftImage, scaledLeftImage, Size(), scaleFactor, scaleFactor, interpolationMethod);
        resize(rightImage, scaledRightImage, Size(), scaleFactor, scaleFactor, interpolationMethod);
     
          // Set common parameters
        bm.ndisp = 256;
        bp.ndisp = 0;
        csbp.ndisp = 0;
  // Prepare disparity map of specified type
        Mat disp(scaledLeftImage.size(), CV_8U);
        gpu::GpuMat d_disp(scaledLeftImage.size(), CV_8U);
	d_left.upload(scaledLeftImage);
        d_right.upload(scaledRightImage);
          if (d_left.channels() > 1 || d_right.channels() > 1)
            {
                cout << "BM doesn't support color images\n";
               
            }
            bm(d_left, d_right, d_disp);
        d_disp.download(disp);
                imshow("Disparity", disp);
	//cout << "M = " << endl << " " << disp << endl << endl;
       // bp = cuda::createStereoBeliefPropagation(p.ndisp);
       // csbp = cuda::createStereoConstantSpaceBP(p.ndisp);

      /*  if (numThreads > 1){
                cout << "numThreads >1" << endl;
            // Divide the disparity map computation into diferent threads by making each thread compute a strip of the disparity map
            const int extraMargin = 10; // This is for SGBM to use neighbour comparison and make the later strip fusion smooth
            Mat *dispMaps = new Mat[numThreads]; // Since numThreads is unknown, create a dynamically allocated array

            # pragma omp parallel for
            for (int i = 0; i < numThreads; i++) {
                cout << "parallel" << endl;
                int marginTop = i > 0 ? extraMargin : 0;
                int marginBottom = i < numThreads-1 ? extraMargin : 0;
                Rect roi(0, scaledLeftImage.rows*i/numThreads-marginTop, scaledLeftImage.cols, scaledLeftImage.rows/numThreads+marginBottom + marginTop);
                Mat imR_strip = scaledRightImage(roi);
                Mat imL_strip = scaledLeftImage(roi);
                semiGlobalBlobMatch(imL_strip, imR_strip, dispMaps[i]);
            }
            
            // Merge strips into a single disparity map
            disparityMap = Mat(scaledLeftImage.size(), 3);
            for (int i = 0; i < numThreads; i++) {
                cout << "merge strips" << endl;
                int marginTop = i > 0 ? extraMargin : 0;
                int marginBottom = i < numThreads-1 ? extraMargin : 0;
                Mat roiDispMap = disparityMap(Rect(0, i*(disparityMap.rows/numThreads), disparityMap.cols, disparityMap.rows/numThreads));
                Mat roiStrip = dispMaps[i](Rect(0, marginTop, dispMaps[i].cols, dispMaps[i].rows - marginBottom - marginTop));
                roiStrip.copyTo(roiDispMap);
                
            }
            delete [] dispMaps;
        }
        else {
                cout << "thread <= 1" << endl;
            semiGlobalBlobMatch(scaledLeftImage, scaledRightImage, disparityMap);
	    //Ptr<StereoBM> bm;
	    printf("gpu count is %d",gpu::getCudaEnabledDeviceCount());
        }*/
        //resize(disparityMap, disparityMap, Size(), 1/scaleFactor, 1/scaleFactor, interpolationMethod);
                 cout << "I need to take a shit\n";
    }
    
    else semiGlobalBlobMatch(leftImage, rightImage, disparityMap);
}

//————————————————————————————————————————————————————————————————————
//  getDisparityImgNormalised
//————————————————————————————————————————————————————————————————————

Mat StereoPair::getDisparityImageNormalised(){
	Mat dspn;
	normalize(disparityMap, dspn, 0, 255, CV_MINMAX, CV_8U);
	return dspn;
}

//————————————————————————————————————————————————————————————————————
//  calibrate
//————————————————————————————————————————————————————————————————————

void StereoPair::calibrate(){
	                    cout << "*** calibrate bitch" << endl;
	///////////INITIAL PARAMETERS//////////////
	Size boardSize = Size(9, 6);	//Inner board corners
	float squareSize = 0.022;       //The actual square size, in any unit (meters prefered)
	int nimages = 16;				//Number of images to take for calibration
    int maxScale = 2;

    vector<vector<Point2f> > imagePoints[2];
    vector<vector<Point3f> > objectPoints;
    Size imageSize;

    imagePoints[0].resize(nimages);
    imagePoints[1].resize(nimages);
    objectPoints.resize(nimages);
    vector<Mat> goodImages;			//Vector to store image pairs with calibration pattern correctly detected
    
    int itersSinceLastPattern = 1;

	// Create visualization windows
    float windowResize = 0.8;
    Mat calibDisplay;
	namedWindow("Live calibration view", CV_WINDOW_NORMAL);

	int i, j, k, frameID = 0;
	/*
	 * i: new image pair
	 * j: number of good image pairs
	 * k: image side. 0 = LEFT | 1 = RIGHT
	 */

    for( i = j = 0; j < nimages; i++ )
    {
    	while(1){
                printf("yo");
            this->updateImages();
    		Mat niml = leftImage, nimr = rightImage;
            Mat cimg1, cimg2;	//Corner images
            bool found = false;
            if( imageSize == Size() ) imageSize = niml.size();
            
            //////////DETECT CHESSBOARD CORNERS AND DISPLAY THEM/////////////
            for( k = 0; k < 2; k++ ) {

                Mat img = k==0 ? niml : nimr;
                if(img.empty()) break;

                vector<Point2f>& corners = imagePoints[k][j];
                
                for( int scale = 1; scale <= maxScale; scale++ ) {
                    Mat timg;
                    if( scale == 1 )
                        timg = img;
                    else
                        resize(img, timg, Size(), scale, scale);
                    
                    found = findChessboardCorners(timg, boardSize, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
                    
                    if(found) {
                        if(scale > 1) {
                            Mat cornersMat(corners);
                            cornersMat *= 1./scale;
                        }
                        //////////////Update corner view images///////////////////////
                        Mat cimg;
                        cvtColor(img, cimg, COLOR_GRAY2BGR);
                        drawChessboardCorners(cimg, boardSize, corners, found);
                        double sf = 640./MAX(img.rows, img.cols);
                        Mat cornerImg;
                        resize(cimg, cornerImg, Size(), sf, sf);
                        if(k==0)     cimg1 = cornerImg;
                        else if(k==1)cimg2 = cornerImg;
                        
                        ///////////////IMPROVE CORNER ACCURACY////////////////
                        cornerSubPix(img, corners, Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 30, 0.01));
                        break;
                    }
                }
                
                if(!found) {
                    if (itersSinceLastPattern == 0) {
                        cout << "*** Alert! Calibration pattern must be detected in both frames!" << endl;
                    }
                    itersSinceLastPattern++;
                    break;
                }
            }
            
            if(k == 2 && found){
                calibDisplay = glueTwoImagesHorizontal(cimg1, cimg2);   // Update corner images
                resize(calibDisplay, calibDisplay, Size(), windowResize, windowResize);
                if(itersSinceLastPattern > 0){
                    cout << "*** Correctly detecting pattern!" << endl;
                    itersSinceLastPattern = 0;
                }
            } else {
                calibDisplay = glueTwoImagesHorizontal(leftImage, rightImage);
                resize(calibDisplay, calibDisplay, Size(), windowResize, windowResize);
            }
            imshow("Live calibration view", calibDisplay);
            
    		// Wait for key press
    		int keyPressed = int(char(waitKey(20)));

    		// Take images if 'n' or 'N' keys have been pressed
    		if( keyPressed==78 || keyPressed==110)
            {
                if(k == 2){
                    if (leftImage.channels() > 1) cvtColor(leftImage, leftImage, CV_RGB2GRAY);
                    if (rightImage.channels() > 1) cvtColor(rightImage, rightImage, CV_RGB2GRAY);
                    goodImages.push_back(leftImage);
                    goodImages.push_back(rightImage);
                    j++;
                    cout << "Took a new image pair, remaining images: " << nimages - j << endl;
                    break;
                }
                else cout << "Alert! Calibration pattern must be detected in both frames!" << endl;
    		}
    		// Save the images if required (pressing 's' or 'S')
    		if(keyPressed== 83 || keyPressed==115)
    		{
    			cout << "Saving image pairs..." << endl;

    			if(!dataDirectory.empty()){
    				// Create the file names for saving the images
    				char fileName[256];
    				sprintf(fileName, "%sCalib_%d.png", dataDirectory.c_str(), frameID);

    				// Write the images
    				try {
    					imwrite(fileName, calibDisplay);
    				}
    				catch (runtime_error& ex) {
    					fprintf(stderr, "Exception converting image to PNG format: %s \n", ex.what());
    				}
    			}
    			frameID++;
    		}
    		else if(keyPressed==27){
    			destroyWindow("Live calibration view");
                for(int i = 0; i < 10; i++) waitKey(1);
    			return;
    		}
    	}
    }
    
    destroyWindow("Live calibration view");
    for(int i = 0; i < 10; i++) waitKey(1);

    ////////FILL "objectPoints" WITH THE COORDINATES OF THE BOARD CORNERS////////
    #pragma omp parallel for
    for( i = 0; i < nimages; i++ )
    {
        for( j = 0; j < boardSize.height; j++ )
            for( k = 0; k < boardSize.width; k++ )
                objectPoints[i].push_back(Point3f(j*squareSize, k*squareSize, 0));
    }

    /////////////RUN STEREO CALIBRATION//////////////
    cout << "******************************" << endl;
    cout << "Running stereo calibration ..." << endl;

    Mat cameraMatrix[2], distCoeffs[2];
    cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
    cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
    Mat R, T, E, F;
    
    double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
                    cameraMatrix[0], distCoeffs[0],
                    cameraMatrix[1], distCoeffs[1],
                    imageSize, R, T, E, F,
                    TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
                    CV_CALIB_FIX_ASPECT_RATIO +
                    CV_CALIB_ZERO_TANGENT_DIST +
                    CV_CALIB_SAME_FOCAL_LENGTH +
                    CV_CALIB_RATIONAL_MODEL +
                    CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);
    cout << "done with RMS error=" << rms << endl;
/*
 * /////////CALIBRATION QUALITY CHECK//////////////
 * because the output fundamental matrix implicitly
 * includes all the output information,
 * we can check the quality of calibration using the
 * epipolar geometry constraint: m2^t*F*m1=0
 */
    double err = 0;
    int npoints = 0;
    vector<Vec3f> lines[2];
    for( i = 0; i < nimages; i++ )
    {
        int npt = (int)imagePoints[0][i].size();
        Mat imgpt[2];
        #pragma omp parallel for
        for( k = 0; k < 2; k++ )
        {
            imgpt[k] = Mat(imagePoints[k][i]);
            undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
            computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
        }
        for( j = 0; j < npt; j++ )
        {
            double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
                                imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
                           fabs(imagePoints[1][i][j].x*lines[0][j][0] +
                                imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    cout << "average reprojection err = " <<  err/npoints << endl;

    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];

    stereoRectify(cameraMatrix[0], distCoeffs[0],
                  cameraMatrix[1], distCoeffs[1],
                  imageSize, R, T, R1, R2, P1, P2, Q,
                  CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

    //////////Save calibration parameters//////////////////
    string calibration_parametersFile = dataDirectory + calibration_ParametersFileName;
    FileStorage fs(calibration_parametersFile.c_str(), CV_STORAGE_WRITE);
    if( fs.isOpened() )
    {
    	cout << "Saving parameters" << endl;
    	fs <<
        "Size"          << imageSize <<
        "K1"            << cameraMatrix[0] <<
        "distCoeffs1"   << distCoeffs[0] <<
    	"K2"            << cameraMatrix[1] <<
        "distCoeffs2"   << distCoeffs[1] <<
        "R"             << R <<
        "T"             << T <<
        "Q"             << Q;
    	fs.release();
    	cout << "Parameters saved" << endl;
    }
    else cout << "CALIBRATION ERROR: Cannot save calibration results to file" << endl;

   //////////Once parameters are saved, reinitialize rectification from them////////////////
    this->setupRectification();
    this->displayImages(true /*drawLines*/);
}

//————————————————————————————————————————————————————————————————————
//  glueTwoImagesHorizontal
//————————————————————————————————————————————————————————————————————

Mat StereoPair::glueTwoImagesHorizontal(Mat Img1, Mat Img2){
    Mat LR(Img1.rows, Img1.cols+Img2.cols, Img1.type());
    
    //Place each image horizontally adjacent to each other
    Mat left_roi(LR, Rect(0, 0, Img1.cols, Img1.rows));
    Img1.copyTo(left_roi);
    Mat right_roi(LR, Rect(Img1.cols, 0, Img2.cols, Img2.rows)); // Copy constructor
    Img2.copyTo(right_roi);
    
    return LR;
}


//————————————————————————————————————————————————————————————————————
//  displayImages
//————————————————————————————————————————————————————————————————————

void StereoPair::displayImages(bool drawLines) {
    // Create visualization windows
//    if (rectified) namedWindow("Rectified stereo images", CV_WINDOW_NORMAL);
//    else namedWindow("Uncalibrated stereo images", CV_WINDOW_NORMAL);
    // Reset frame counter
    int frameCount = 0;
    
    for (;;) {
        updateImages();
        Mat LR = glueTwoImagesHorizontal(leftImage, rightImage);
        
        if (drawLines) {
            cvtColor(LR, LR, COLOR_GRAY2BGR);   // Convert to BGR (RGB) color space for drawing coloured lines.
            for(int h=0; h<LR.rows; h+=25) {
                Point pt1(0, h);
                Point pt2(LR.cols, h);
                line(LR, pt1, pt2, CV_RGB(255, 123, 47), 1);
            }
        }
        
        // Draw available commands on the bottom-left corner of the window
        string message = "s: Save images    a: Autotune exposure     ESC: Close mode";
        putText(LR, message, Point(10, LR.rows - 10), FONT_HERSHEY_COMPLEX_SMALL, 0.7, CV_RGB(255, 123, 47), 1, CV_AA);
        
        if (rectify) imshow("Rectified stereo images", LR);
        else imshow("Uncalibrated stereo images", LR);

        int keyPressed = int(char(waitKey(10)));
        
        // Save the images if 's' or 'S' key has been pressed
        if( keyPressed==83 || keyPressed==115) {
            saveImage(leftImage, (rectify? "Rectified_L_" + std::to_string(frameCount) : "Uncalibrated_L_" + std::to_string(frameCount)), dataDirectory);
            saveImage(rightImage, (rectify? "Rectified_R_" + std::to_string(frameCount) : "Uncalibrated_R_" + std::to_string(frameCount)), dataDirectory);
            if (drawLines) saveImage(LR, "StereoPair" + std::to_string(frameCount), dataDirectory);
            frameCount++;
        }
        
        // Exit if 'esc' key is pressed
        if( keyPressed==27) {
            // Close the windows
            if (rectify) destroyWindow("Rectified stereo images");
            else destroyWindow("Uncalibrated stereo images");
            for(int i = 0; i < 10; i++) waitKey(1); // In some systems, if this is not included windows may becmome unresponsive.
            return;
        }
    }
}

//————————————————————————————————————————————————————————————————————
// saveImage
//————————————————————————————————————————————————————————————————————

bool StereoPair::saveImage(Mat image, string imageName, string outputDirectory) {
    
   /* if(!outputDirectory.empty()){
        char fileName[256];
        sprintf(fileName, "%s%s.png", outputDirectory.c_str(), imageName.c_str());
        try {
            imwrite(fileName, image);
            cout << "Saved image: " << fileName << endl;
            return 1;
        }
        catch (std::runtime_error& ex){
            fprintf(stderr, "Could not save image to store: %s \n", ex.what());
        }
    }
    else fprintf(stderr, "Output directory is not set. Image could not be saved.");
    return false;*/
}

