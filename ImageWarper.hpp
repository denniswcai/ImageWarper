//
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

typedef struct 
{
    cv::Rect2f  tgtBlock;
    cv::Point2f srcCorners[4]; 
} MappingBlock;

typedef std::vector<MappingBlock> MappingTable;

int buildSphericRotation90MappingTable( MappingTable &mappingTable );
int buildMappingTableFromBinFiles( const char *vertexBinFileName, const char *polygonBinFileName, MappingTable mappingTables[4] );
int imageWarper( const cv::Mat &srcImage, cv::Mat &tgtImage, MappingTable mappingTable, bool bInterpolation = 0 );
cv::Mat createWarpedImage( const cv::Mat srcImages[], int numOfImages, char *vertexBinFileName, char *polygonBinFileName, int tgtWidth, int tgtHeight );