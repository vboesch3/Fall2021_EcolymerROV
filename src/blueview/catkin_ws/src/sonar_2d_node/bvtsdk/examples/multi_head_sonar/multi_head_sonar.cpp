/*
 * Multi-Head Sonar Example
 * Connect to an ethernet connected sonar, acquire and save pings on all heads
 */

#include <stdio.h>

#include <bvt_sdk.h>

#define MAX_SONAR_HEADS  (2)

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

	// Open the live sonar
	ret = BVTSonar_Open(son, "NET", "192.168.1.45");
	if( ret != 0 )
	{
		printf("BVTSonar_Open: ret=%s\n", BVTError_GetString(ret) );
		return 1;
	}

	BVTHead son_heads[MAX_SONAR_HEADS];

    int headcount = BVTSonar_GetHeadCount(son);
    printf("The sonar has %d heads.\n", headcount);


	// Get each head, and set the range
	for( int i = 0; i < headcount; i++ )
	{
		// Get the first head
		ret = BVTSonar_GetHead(son, i, &son_heads[i]);
		if( ret != 0 )
		{
            // Some sonars will return more heads than they actually have,
            // though shouldn't be the case with multiple head sonars.
            printf("BVTSonar_GetHead for head %d: ret=%s\n", i, BVTError_GetString(ret) );
            headcount = i;
            break;
		}else
        {
            // Set the range window to be 1m to 40m
            BVTHead_SetRange(son_heads[i], 1, 40);
        }
	}
	

	////////////////////////////////////////////////
	// Create a file "sonar" to save some pings to
	BVTSonar file = BVTSonar_Create();
	if( file == NULL )
	{
		printf("BVTSonar_Create: failed\n");
		return 1;
	}

	// Create the file, which "clones" the live sonar parameters to the 
	// file "sonar." 
	ret = BVTSonar_CreateFile(file, "out.son", son, "");
	if( ret != 0 )
	{
		printf("BVTSonar_CreateFile: ret=ret=%s\n", BVTError_GetString(ret) );
		return 1;
	}

	// Get each of the file heads
	BVTHead file_heads[MAX_SONAR_HEADS];
	for ( int i = 0; i < headcount; i++ )
	{
		ret = BVTSonar_GetHead(file, i, &file_heads[i]);
		if( ret != 0 )
		{
			printf("BVTSonar_GetHead: ret=%s\n", BVTError_GetString(ret) );
			return 1;
		}
	}


	////////////////////////////////////////////////
	// Now, let's go get some pings!
	for ( int ping_num = 0; ping_num < 10; ping_num++ )
	{
		for (int head_num = 0; head_num < headcount; head_num++)
		{
			BVTPing ping = NULL;

			printf("Getting ping %d on head %d\n", ping_num, head_num);

			ret = BVTHead_GetPing (son_heads[head_num], -1, &ping );
			if( ret != 0 )
			{
				printf("BVTHead_GetPing: ret=%s\n", BVTError_GetString(ret) );
				return 1;
			}

			ret = BVTHead_PutPing( file_heads[head_num], ping );
			if( ret != 0 )
			{
				printf("BVTHead_PutPing: ret=%s\n", BVTError_GetString(ret) );
				return 1;
			}
			BVTPing_Destroy(ping);
		}
	}


	BVTSonar_Destroy( file );  // This also closes the file properly.
	BVTSonar_Destroy( son );

	return 0;
}
