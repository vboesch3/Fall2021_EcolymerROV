/*
 * File Sonar Example using OpenCV
 * Demonstrate opening a file, accessing a head, and retriving a ping.
 * The ping is then processed into an image and displayed using OpenCV
 * Finally, a colormap is loaded and the image is colormapped.  The
 * color image is also displayed with OpenCV
 */

#include <stdio.h>

#include <bvt_sdk.h>

#include <cv.h>
#include <highgui.h>

const char DataFile[] = "../../../data/swimmer.son";

int main( void )
{
	int ret;
	// Create a new BVTSonar Object
	BVTSonar son = BVTSonar_Create();
	if( son == NULL )
	{
		printf("BVTSonar_Create: failed\n");
		return 1;
	}

	// Open the sonar
	ret = BVTSonar_Open(son, "FILE", DataFile);
	if( ret != 0 )
	{
		printf("BVTSonar_Open: ret=%d\n", ret);
		return 1;
	}

	// Make sure we have the right number of heads
	int heads = -1;
	heads = BVTSonar_GetHeadCount(son);
	printf("BVTSonar_GetHeadCount: %d\n", heads);


	// Get the first head
	BVTHead head = NULL;
	ret = BVTSonar_GetHead(son, 0, &head);
	if( ret != 0 )
	{
		printf("BVTSonar_GetHead: ret=%d\n", ret);
		return 1;
	}
	
	// Check the ping count
	int pings = -1;
	pings = BVTHead_GetPingCount(head);
	printf("BVTHead_GetPingCount: %d\n", pings);

	// Set the range window to be 10m to 40m
	BVTHead_SetRange(head, 10, 40);

	// Now, get a ping!
	BVTPing ping = NULL;
	ret = BVTHead_GetPing(head, 0, &ping);
	if( ret != 0 )
	{
		printf("BVTHead_GetPing: ret=%d\n", ret);
		return 1;
	}
	
	// Generate an image from the ping
	BVTMagImage img;
	ret = BVTPing_GetImage(ping, &img);
	if( ret != 0 )
	{
		printf("BVTPing_GetImage: ret=%d\n", ret);
		return 1;
	}

	printf("\n");

	/////////////////////////////////////////////////////////
	// Use OpenCV to display the image
	
	int height = BVTMagImage_GetHeight(img);
	int width = BVTMagImage_GetWidth(img);

	// Create a IplImage header
	IplImage* gray_img = cvCreateImageHeader(cvSize(width,height), IPL_DEPTH_16U, 1);
	
	// And then set it's data
	// Note: This means that the IplImage points to memory 'owned'
	// by the SDK.  Make sure that you don't use gray_img after you've
	// destroyed the img.
	cvSetImageData(gray_img,  BVTMagImage_GetBits(img), width*2);
	
	// Make a new window and show the image
	char gray_wnd[] = "Gray";
	cvNamedWindow(gray_wnd, 1);
	cvShowImage(gray_wnd, gray_img);



	/////////////////////////////////////////////////////////
	// Build a color mapper
	BVTColorMapper mapper;
	mapper = BVTColorMapper_Create();
	if( mapper == NULL )
	{
		printf("BVTColorMapper_Create: failed\n");
		return 1;
	}
	
	// Load the bone colormap
	ret = BVTColorMapper_Load(mapper, "../../../colormaps/jet.cmap");
	if( ret != 0 )
	{
		printf("BVTColorMapper_Load: ret=%d\n", ret);
		return 1;
	}
	
	// Perform the colormapping
	BVTColorImage cimg;
	ret = BVTColorMapper_MapImage(mapper, img, &cimg);
	if( ret != 0 )
	{
		printf("BVTColorMapper_MapImage: ret=%d\n", ret);
		return 1;
	}
	
	printf("\n");
	
	/////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////
	// Use OpenCV to display the image
	height = BVTColorImage_GetHeight(cimg);
	width = BVTColorImage_GetWidth(cimg);

	// Create a IplImage header
	IplImage* color_img = cvCreateImageHeader(cvSize(width,height), IPL_DEPTH_8U, 4);
	
	// And set it's data
	cvSetImageData(color_img,  BVTColorImage_GetBits(cimg), width*4);
	
	IplImage* color_img2 = cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 3);
	cvCvtColor( color_img,color_img2, CV_RGB2BGR );	
	
	char color_wnd[] = "Color";
	cvNamedWindow(color_wnd, 1);
	
	cvShowImage(color_wnd, color_img);
	
	char color_wnd2[] = "Color 2";
	cvNamedWindow(color_wnd2, 1);
	
	cvShowImage(color_wnd2, color_img2);
	

	///////////////////////////////////////////////////////////
	cvWaitKey(0);
	
	cvReleaseImageHeader(&gray_img);
	cvDestroyWindow(gray_wnd);
	cvReleaseImageHeader(&color_img);
	cvDestroyWindow(color_wnd);
	cvReleaseImageHeader(&color_img2);
	cvDestroyWindow(color_wnd2);
	
	// Clean up
	BVTColorImage_Destroy(cimg);
	BVTMagImage_Destroy(img);
	BVTColorMapper_Destroy(mapper);
	BVTPing_Destroy(ping);
	BVTSonar_Destroy(son);
	return 0;
}
