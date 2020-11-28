//
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "FileReader.hpp"
#include "DebugMessage.h"
#include "ImageWarper.hpp"

// Use openmp to leverage multi-core to accelerate:
// NOTE: Need to add -fopenmp to g++ commnad
#define ENABLE_OPENMP
#ifdef  ENABLE_OPENMP
#include <omp.h>
#endif

#define VERSION_NUMBER "0.1"

typedef struct {
        float tgtX;
        float tgtY;
        float tgtZ;
        float imgIdx;
        float reserved;
        float alpha;
        float srcX;
        float srcY;
} VertexEntry;

typedef struct {
    unsigned short biTriangle[6];
} PolygonEntry;

typedef struct {
    unsigned char b;
    unsigned char g;
    unsigned char r;
} ColorRGB;


inline cv::Point2f sphericalRotation90( cv::Point2f thetaPhai )
{
    cv::Point2f result;
    float x = cos(thetaPhai.y) * cos(thetaPhai.x);
    float y = cos(thetaPhai.y) * sin(thetaPhai.x);
    float z = sin(thetaPhai.y);

    float x1 = -x;//y;
    float y1 = z;//z;
    float z1 = -y;//x;

    result.y = asin(z1);
    result.x = atan2(y1, x1);

return( result );
}


int buildSphericRotation90MappingTable( MappingTable &mappingTable )
{
    float gridX = 32.0 / 8000.0;
    float gridY = 32.0 / 4000.0;
    float overlapGap = 384.0 / 4000.0;
    cv::Point2f srcThetaPhai[4];

    mappingTable.clear();

    for( int mi = 0; mi < 4; mi++ )
    {
        for( float yt = -1.0; yt < 1.0; yt += gridY )
        {
            for( float xt = -1.0; xt < 1.0; xt += gridX )
            {
                srcThetaPhai[0] = sphericalRotation90( cv::Point2f( xt * M_PI, yt * M_PI / 2.0 ) );

                if( srcThetaPhai[0].x > -M_PI/2.0 && srcThetaPhai[0].x < M_PI/2.0 && 
                    srcThetaPhai[0].y >= -overlapGap*M_PI/2.0  && srcThetaPhai[0].y < overlapGap*M_PI/2.0 )  
                {
                    srcThetaPhai[1] = sphericalRotation90( cv::Point2f( (xt+gridX) * M_PI, (yt+0    ) * M_PI / 2.0 ) );
                    srcThetaPhai[2] = sphericalRotation90( cv::Point2f( (xt+0    ) * M_PI, (yt+gridY) * M_PI / 2.0 ) );
                    srcThetaPhai[3] = sphericalRotation90( cv::Point2f( (xt+gridX) * M_PI, (yt+gridY) * M_PI / 2.0 ) );

                    MappingBlock  vertexMapping;

                    vertexMapping.tgtBlock.x = xt + mi*0.5;
                    vertexMapping.tgtBlock.y = yt;
                    vertexMapping.tgtBlock.width  = gridX;
                    vertexMapping.tgtBlock.height = gridY;
                    
                    for( int vi = 0; vi < 4; vi++ ) {
                        vertexMapping.srcCorners[vi].x = (srcThetaPhai[vi].y + overlapGap*M_PI/2.0)/(overlapGap*M_PI*4) + mi/4.0;
                        vertexMapping.srcCorners[vi].y = (srcThetaPhai[vi].x + M_PI/2.0) / M_PI; 
                    }

                    printf("*");
                    
                    mappingTable.push_back( vertexMapping );
                }
            }
        }
    }

return(0);
}


int buildMappingTableFromBinFiles( const char *vertexBinFileName, const char *polygonBinFileName, MappingTable mappingTables[4] )
{
    DEBUG_ASSERT( vertexBinFileName != NULL && polygonBinFileName != NULL, "File name not provided.\n" );

    for( int mi = 0; mi < 4; mi++ ) {
        mappingTables[mi].clear();
    }
    
    // Read vertex bin file:
    FileReader vertexBinFile( vertexBinFileName );
    if( vertexBinFile.isEmpty() ) {
        ERROR_MESSAGE("Failed to read vertexBinFileName with name %s\n", vertexBinFileName );
        return(-2);
    }
    VertexEntry *vertexTable = (VertexEntry *)vertexBinFile.getContents();
    long vertexFileLength = vertexBinFile.fileSize();

    // Read polygon bin file:
    FileReader polygonBinFile( polygonBinFileName );
    if( polygonBinFile.isEmpty() ) {
        ERROR_MESSAGE("Failed to read polygonBinFileName with name %s\n", polygonBinFileName );
        return(-3);
    }
    PolygonEntry *polygonList = (PolygonEntry *)polygonBinFile.getContents();
    long polygonFileLength = polygonBinFile.fileSize();
   
    ////
    int numOfPolygonEntries = int(polygonFileLength/sizeof(PolygonEntry));
    printf( "polygon list[%d]:\n", numOfPolygonEntries );
   
    for( int i = 0; i < numOfPolygonEntries; i++ ) 
    {
        int mi = vertexTable[ polygonList[i].biTriangle[0] ].imgIdx;

        int cornerIdx[4] = {-1,-1,-1,-1};
        float cornerValue[4] = {3.0, -3.0, 3.0, -3.0};
        
        for( int ti = 0; ti < 6; ti++ )
        { 
            float xPlusY  = vertexTable[ polygonList[i].biTriangle[ti] ].tgtX + vertexTable[ polygonList[i].biTriangle[ti] ].tgtY;
            float xMinusY = vertexTable[ polygonList[i].biTriangle[ti] ].tgtX - vertexTable[ polygonList[i].biTriangle[ti] ].tgtY;

            if( xPlusY < cornerValue[0] ) {
                cornerValue[0] = xPlusY;
                cornerIdx[0] = ti;
            }
            if( xMinusY > cornerValue[1] ) {
                cornerValue[1] = xMinusY;
                cornerIdx[1] = ti;
            }
            if( xMinusY < cornerValue[2] ) {
                cornerValue[2] = xMinusY;
                cornerIdx[2] = ti;
            }
            if( xPlusY > cornerValue[3] ) {
                cornerValue[3] = xPlusY;
                cornerIdx[3] = ti;
            }
        }

        DEBUG_ASSERT( cornerIdx[0] != -1 && cornerIdx[1] != -1 && cornerIdx[2] != -1 && cornerIdx[3] != -1, "Failed to find block corner!\n" );

        if( ( vertexTable[ polygonList[i].biTriangle[cornerIdx[0]]].tgtY !=  vertexTable[ polygonList[i].biTriangle[cornerIdx[1]]].tgtY ) ||
            ( vertexTable[ polygonList[i].biTriangle[cornerIdx[2]]].tgtY !=  vertexTable[ polygonList[i].biTriangle[cornerIdx[3]]].tgtY ) ||
            ( vertexTable[ polygonList[i].biTriangle[cornerIdx[0]]].tgtX !=  vertexTable[ polygonList[i].biTriangle[cornerIdx[2]]].tgtX ) ||
            ( vertexTable[ polygonList[i].biTriangle[cornerIdx[1]]].tgtX !=  vertexTable[ polygonList[i].biTriangle[cornerIdx[3]]].tgtX ) || 
            ( vertexTable[ polygonList[i].biTriangle[cornerIdx[0]]].imgIdx != vertexTable[ polygonList[i].biTriangle[cornerIdx[3]]].imgIdx ) )
        {
            printf("WARNING: ################################################!!!!!!!!!!!!!!!!!\n");
        }

        VertexEntry v[4];
        MappingBlock  vertexMapping;

        v[0] = vertexTable[ polygonList[i].biTriangle[cornerIdx[0]] ];
        v[1] = vertexTable[ polygonList[i].biTriangle[cornerIdx[1]] ];
        v[2] = vertexTable[ polygonList[i].biTriangle[cornerIdx[2]] ];
        v[3] = vertexTable[ polygonList[i].biTriangle[cornerIdx[3]] ];

        vertexMapping.tgtBlock = cv::Rect2f( v[0].tgtX, v[0].tgtY, v[1].tgtX - v[0].tgtX, v[2].tgtY - v[0].tgtY );

        for( int vi = 0; vi < 4; vi++ )
        {
            // Check range: 
            /*
            if( v[vi].srcX < 0.0 ){
                v[vi].srcX = 0.0;
            }
            if( v[vi].srcX > 1.0 ){
                v[vi].srcX = 1.0;
            }
            if( v[vi].srcY < 0.0 ){
                v[vi].srcY = 0.0;
            }
            if( v[vi].srcY > 1.0 ){
                v[vi].srcY = 1.0;
            }*/

            vertexMapping.srcCorners[vi] = cv::Point2f( v[vi].srcX, v[vi].srcY );
        }

        mappingTables[mi].push_back( vertexMapping );
    }

return( 0 );
}


inline void bilinearPutPixel( uchar *pTgtPixels, uchar *pSrcPixels, int srcLineStep, int srcPixelStep, float xsf, float ysf )
{
    ColorRGB cbytes[4];

    // !check boundary condition: what happen when ( xsf >= srcImageWidth-1 || ysf >= srcImageHeight-1 ) ?...

    float fracx = int(xsf) + 1.0 - xsf;
    float fracy = int(ysf) + 1.0 - ysf;

    float w0 = fracx * fracy;
    float w1 = (1.0-fracx) * fracy;
    float w2 = fracx * (1.0-fracy);
    float w3 = (1.0-fracx) * (1.0-fracy);

    cbytes[0].b = *(pSrcPixels + 0 ) * w0;
    cbytes[0].g = *(pSrcPixels + 1 ) * w0;
    cbytes[0].r = *(pSrcPixels + 2 ) * w0;

    cbytes[1].b = *(pSrcPixels + srcPixelStep + 0 ) * w1;
    cbytes[1].g = *(pSrcPixels + srcPixelStep + 1 ) * w1;
    cbytes[1].r = *(pSrcPixels + srcPixelStep + 2 ) * w1;

    cbytes[2].b = *(pSrcPixels + srcLineStep + 0 ) * w2;
    cbytes[2].g = *(pSrcPixels + srcLineStep + 1 ) * w2;
    cbytes[2].r = *(pSrcPixels + srcLineStep + 2 ) * w2;

    cbytes[3].b = *(pSrcPixels + srcLineStep + srcPixelStep + 0 ) * w3;
    cbytes[3].g = *(pSrcPixels + srcLineStep + srcPixelStep + 1 ) * w3;
    cbytes[3].r = *(pSrcPixels + srcLineStep + srcPixelStep + 2 ) * w3;

    *(pTgtPixels + 0) = cbytes[0].b + cbytes[1].b + cbytes[2].b + cbytes[3].b;
    *(pTgtPixels + 1) = cbytes[0].g + cbytes[1].g + cbytes[2].g + cbytes[3].g;
    *(pTgtPixels + 2) = cbytes[0].r + cbytes[1].r + cbytes[2].r + cbytes[3].r;
}


int imageWarper( const cv::Mat &srcImage, cv::Mat &tgtImage, MappingTable mappingTable, bool bInterpolation )
{
    DEBUG_ENTER_FUNCTION;
    DEBUG_ASSERT( !srcImage.empty() && !tgtImage.empty(), "Input or output image can not be empty.\n" );

    int srcWidth  = srcImage.cols;
    int srcHeight = srcImage.rows;
    int tgtWidth  = tgtImage.cols;
    int tgtHeight = tgtImage.rows;
    int srcLineStep  = srcImage.step[0];
    int srcPixelStep = srcImage.step[1];
    int tgtLineStep  = tgtImage.step[0];
    int tgtPixelStep = tgtImage.step[1];

    int bi;
    // !!! Use openmp to leverage the multi-core of CPU to accelerate
    // NOTE: Need to add -fopenmp to g++ commnad
    #ifdef ENABLE_OPENMP
    int num_procs = omp_get_num_procs();
    DEBUG_MESSAGE("Openmp enabled, number of processors(cores): %d.\n", num_procs );
    #pragma omp parallel for private( bi ) num_threads( num_procs )
    #endif

    for( bi = 0; bi < mappingTable.size(); bi++ )
    {
        cv::Point2f v[4];

        for( int vi = 0;  vi < 4; vi++ ) 
        {
            v[vi].x = mappingTable[bi].srcCorners[vi].x * srcWidth;
            v[vi].y = mappingTable[bi].srcCorners[vi].y * srcHeight;
            
            // Check range:
            if( v[vi].x < 0 || v[vi].x >= srcWidth || v[vi].y < 0 || v[vi].y >= srcHeight )
            {
                ;//DEBUG_MESSAGE("source vertex out of range at blk #%d, v[vi].x=%f, v[vi].y=%f\n", bi, v[vi].x, v[vi].y );
            }
        }

        cv::Rect2i blk = cv::Rect2i( int( (mappingTable[bi].tgtBlock.x + 1.0) / 2.0 * tgtWidth  + 0.5 ),
                                     int( (mappingTable[bi].tgtBlock.y + 1.0) / 2.0 * tgtHeight + 0.5 ),
                                     int( (mappingTable[bi].tgtBlock.width)   / 2.0 * tgtWidth  + 0.5 ),
                                     int( (mappingTable[bi].tgtBlock.height)  / 2.0 * tgtHeight + 0.5 ) );
        //DEBUG_MESSAGE("Block #%d: (%d, %d, %d, %d)=>(%f,%f),(%f,%f),(%f,%f),(%f,%f) )\n", 
        //               bi, blk.x, blk.y, blk.width, blk.height, v[0].x, v[0].y, v[1].x, v[1].y, v[2].x, v[2].y, v[3].x, v[3].y );
        float yFacto = 0.0;
        float xFacto = 0.0;
        float xyFacto2 = 0.0;
        float yFactoInc = 1.0 / float(blk.height);
        float xFactoInc = 1.0 / float(blk.width);
        float xyFacto2Inc; 

        uchar *pTgtPixels = (uchar *)tgtImage.data + blk.y*tgtLineStep + blk.x*tgtPixelStep;

        for( int by = 0; by < blk.height; by++ )
        {
            xFacto = 0.0;
            xyFacto2 = 0.0;
            xyFacto2Inc = yFacto / float(blk.width);

            for( int bx = 0; bx < blk.width; bx++ ) 
            {
                float xsf = v[0].x + (v[1].x-v[0].x)*xFacto + (v[2].x-v[0].x)*yFacto + (v[3].x-v[2].x-v[1].x+v[0].x)*xyFacto2;
                float ysf = v[0].y + (v[1].y-v[0].y)*xFacto + (v[2].y-v[0].y)*yFacto + (v[3].y-v[2].y-v[1].y+v[0].y)*xyFacto2;

                int xsi = int(xsf);
                int ysi = int(ysf);

                if( xsi >= 0 && xsi < srcWidth && ysi >= 0 && ysi < srcHeight ) 
                {
                    uchar *pSrcPixels = (uchar *)srcImage.data + ysi*srcLineStep + xsi*srcPixelStep;
            
                    //if( bInterpolation ) 
                    if( bInterpolation && ysi < srcHeight-1 && xsi < srcWidth-1 )
                    {
                        bilinearPutPixel( pTgtPixels, pSrcPixels, srcLineStep, srcPixelStep, xsf, ysf );
                    } 
                    else 
                    {
                        pTgtPixels[ 0 ] = pSrcPixels[ 0 ]; 
                        pTgtPixels[ 1 ] = pSrcPixels[ 1 ]; 
                        pTgtPixels[ 2 ] = pSrcPixels[ 2 ]; 
                    }
                } 
                //else 
                //{
                //    //DEBUG_MESSAGE("source position out of range at blk#%d, xsi=%d, ysi=%d\n", bi, xsi, ysi );
                //}

                xFacto += xFactoInc;
                xyFacto2 += xyFacto2Inc;
                pTgtPixels += tgtPixelStep;
            }

            yFacto += yFactoInc;
            pTgtPixels += tgtLineStep - blk.width*tgtPixelStep;
        }
    }

return(0);
} 


cv::Mat createWarpedImage( const cv::Mat srcImages[], int numOfImages, char *vertexBinFileName, char *polygonBinFileName, int tgtWidth, int tgtHeight )
{
    int result;

    for( int mi = 0; mi < numOfImages; mi++ ) {
        DEBUG_ASSERT( !srcImages[mi].empty(), "Input images can not be empty!\n" );
    }
    
    MappingTable mappingTables[numOfImages];

    result = buildMappingTableFromBinFiles( vertexBinFileName, polygonBinFileName, mappingTables );

    for( int imgIdx = 0; imgIdx < numOfImages; imgIdx ++ ) {
        DEBUG_MESSAGE("mappingTable[%d] size = %ld.\n", imgIdx, mappingTables[imgIdx].size() );
    }

    cv::Mat tgtImage( tgtHeight, tgtWidth, CV_8UC3 );

    for( int mi = 0; mi < numOfImages; mi++ ) 
    {
        DEBUG_MESSAGE("Warping image #%d\n", mi );
        imageWarper( srcImages[mi], tgtImage, mappingTables[mi], 1 );
    }

return( tgtImage );
}


#define IWP_UNIT_TEST
#ifdef IWP_UNIT_TEST

// For processing time measurement:
#include "TimeElapsed.h"

const char *kVertexBinFileName = "vertex.bin";
const char *kPolygonBinFileName = "polygon.bin";
const char *kVertexBinFileName_2 = "vertex_overlap.bin";
const char *kPolygonBinFileName_2 = "polygon_overlap.bin";

const char *kSrcImageFileNames[4] = { "camera_0.jpg", "camera_1.jpg", "camera_2.jpg", "camera_3.jpg" };

int main()
{
    int result;
    // For processing speed measurement.
    TIMING_START_MEASUREMENT;

/*
    cv::Mat srcImages[4];

    for( int mi = 0; mi < 4; mi++ )
    {
        srcImages[mi] = cv::imread( kSrcImageFileNames[mi] );

        if( srcImages[mi].empty() )
        {
            ERROR_MESSAGE("Failed to read image from file %s.\n", kSrcImageFileNames[mi]);
            return(-1);
        }
    }
    
    MappingTable mappingTables[4];

    result = buildMappingTableFromBinFiles( kVertexBinFileName, kPolygonBinFileName, mappingTables );

    for( int imgIdx = 0; imgIdx < 4; imgIdx ++ ) {
        DEBUG_MESSAGE("mappingTable[%d] size = %ld.\n", imgIdx, mappingTables[imgIdx].size() );
    }

    TIMING_CHECK_ELAPSED;

    cv::Mat tgtImage( 4000, 8000, CV_8UC3 );

    for( int mi = 0; mi < 4; mi++ ) 
    {
        DEBUG_MESSAGE("Warping image #%d\n", mi );
        imageWarper( srcImages[mi], tgtImage, mappingTables[mi], 1 );
    }

    TIMING_CHECK_ELAPSED;

    //cv::Mat smallImage;
    //cv::resize( tgtImage, smallImage, cv::Size(0,0), 0.5, 0.5 );
    //cv::imshow( "tgtImage (1/4)", smallImage );
    //cv::waitKey(0);

    cv::imwrite( "panoimage_8k.jpg", tgtImage );

    TIMING_CHECK_ELAPSED;


    ///////
    ///////
    for( int mi = 0; mi < 4; mi++ )
    {
        mappingTables[mi].clear();
    }

    result = buildMappingTableFromBinFiles( kVertexBinFileName_2, kPolygonBinFileName_2, mappingTables );

    for( int mi = 0; mi < 4; mi ++ ) {
        DEBUG_MESSAGE("mappingTable[%d] size = %ld.\n", mi, mappingTables[mi].size() );
    }

    //for( int mi = 0; mi < 4; mi++ )
    //{   
    //    printf("\n################ IMAGE No.%d\n", mi );
    //    for( int bi = 0; bi < mappingTables[mi].size(); bi++ )
    //    {
    //         printf( "(%f, %f, %f, %f)\n", mappingTables[mi][bi].tgtBlock.x,  mappingTables[mi][bi].tgtBlock.y, mappingTables[mi][bi].tgtBlock.width, mappingTables[mi][bi].tgtBlock.height );
    //    }
    //}
    ////

    TIMING_CHECK_ELAPSED;

    cv::Mat overlapImage( 4000, 8000, CV_8UC3 );
    
    for( int mi = 0; mi < 4; mi++ ) 
    {
        DEBUG_MESSAGE("Warping image #%d\n", mi );
        imageWarper( srcImages[mi], overlapImage, mappingTables[mi], 1 );
    }

    TIMING_CHECK_ELAPSED;

    cv::imwrite( "overlapimage_8k.jpg", overlapImage );

    TIMING_CHECK_ELAPSED;

    cv::Mat smallImage;
    cv::resize( overlapImage, smallImage, cv::Size(0,0), 0.5, 0.5 );
    cv::imshow( "overlapImage (1/4)", smallImage );
    cv::waitKey(0);
*
    MappingTable mappingTables[4];

    buildSphericRotation90MappingTable( mappingTables );

    cv::Mat srcGapImage = cv::imread( "overlapimage_8k.jpg" );

    if( srcGapImage.empty() )
    {
        ERROR_MESSAGE("Failed to read gap image from file \n");
        return(-1);
    }

    cv::Mat tgtGapImage( 4000, 8000, CV_8UC3 );

    for( int mi = 0; mi < 4; mi++ ) 
    {
        DEBUG_MESSAGE("Warping gap image \n");
        imageWarper( srcGapImage, tgtGapImage, mappingTables[mi], 0 );
    }
    
    TIMING_CHECK_ELAPSED; 

    cv::Mat smallImage;
    cv::resize( tgtGapImage, smallImage, cv::Size(0,0), 0.33, 0.33 );
    cv::imshow( "target gap Image (1/4)", smallImage );
    cv::waitKey(0);
*/
   

    MappingTable mappingTable;

    buildSphericRotation90MappingTable( mappingTable );
    DEBUG_MESSAGE("mappingTable size = %ld.\n", mappingTable.size() );

    cv::Mat allGapsImage = cv::imread("allgapsimage.jpg");
    
    if( allGapsImage.empty() )
    {
        ERROR_MESSAGE("Failed to read all-gaps-image from file.\n");
        return(-1);
    }

    cv::Mat panoImage( 4000, 8000, CV_8UC3 );

    imageWarper( allGapsImage, panoImage, mappingTable, 0 );

    TIMING_CHECK_ELAPSED;

    cv::Mat smallImage;
    cv::resize( panoImage, smallImage, cv::Size(0,0), 0.33, 0.33 );
    cv::imshow( "pano Image (1/4)", smallImage );
    cv::waitKey(0);
    

return( result );
}

#endif  //IWP_UNIT_TEST