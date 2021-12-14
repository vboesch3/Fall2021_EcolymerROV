/*
 * File Sonar Example
 * Demonstrate opening a file, accessing a head, and retriving a ping.
 * The ping is then processed into an image and saved to a file.
 * Finally, a colormap is loaded and the image is colormapped.
 */

#include <stdio.h>

#include <bvt_sdk.h>

char DataFile[] = "../../data/swimmer.son";

int main( int argc, char *argv[] )
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
    if ( argc == 2 )
        strcpy( DataFile, argv[1] );

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

    // Check the min and max range in this file
    printf("BVTHead_GetMinimumRange: %0.2f\n", BVTHead_GetMinimumRange(head) );
    printf("BVTHead_GetMaximumRange: %0.2f\n", BVTHead_GetMaximumRange(head) );


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
	
	// Check the image height and width out
	int height = BVTMagImage_GetHeight(img);
	printf("BVTMagImage_GetHeight: %d\n", height);
	int width = BVTMagImage_GetWidth(img);
	printf("BVTMagImage_GetWidth: %d\n", width);
	
	// Save it to a PGM (PortableGreyMap)
	ret = BVTMagImage_SavePGM(img, "img.pgm");
	if( ret != 0 )
	{
		printf("BVTMagImage_SavePGM: ret=%d\n", ret);
		return 1;
	}

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
	ret = BVTColorMapper_Load(mapper, "../../colormaps/bone.cmap");
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
	// Check the image height and width out
	height = BVTColorImage_GetHeight(cimg);
	printf("BVTColorImage_GetHeight: %d\n", height);
	width = BVTColorImage_GetWidth(cimg);
	printf("BVTColorImage_GetWidth: %d\n", width);

	
	// Save it to a PPM (PortablePixMap)
	ret = BVTColorImage_SavePPM(cimg, "cimg.ppm");
	if( ret != 0 )
	{
		printf("BVTColorImage_SavePPM: ret=%d\n", ret);
		return 1;
	}

	// Clean up
	BVTColorImage_Destroy(cimg);
	BVTMagImage_Destroy(img);
	BVTColorMapper_Destroy(mapper);
	BVTPing_Destroy(ping);
	BVTSonar_Destroy(son);
	return 0;
}
