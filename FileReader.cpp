//
#include <stdio.h>
#include <stdlib.h>
#include "FileReader.hpp"

FileReader::FileReader()
{
    mpFile = NULL;
    mpBuffer = NULL;
    mFileSize = 0;
    mbEmpty = 1;
}

FileReader::FileReader( const char* filename )
{
    int result = readFile( filename );
}

FileReader::~FileReader()
{
    int result = closeFile();
}

int FileReader::readFile( const char *filename )
{
    mpFile = fopen( filename, "rb" );

    if( mpFile == NULL ) {
        printf( "Failed to open file to read, with name:%s\n", filename );
        return(-1);
    }

    fseek( mpFile, 0L, SEEK_END );
    mFileSize = ftell( mpFile );
    fseek( mpFile, 0L, SEEK_SET );
    printf( "Size of file = %ld.\n", mFileSize );

    mpBuffer = malloc( mFileSize );

    if( mpBuffer == NULL ) {
        printf( "Failed to alloc memory for file buffer (size=%ld)\n", mFileSize );
        return(-2);
    } 

    int readLength = fread( mpBuffer, 1, mFileSize, mpFile );

    if( readLength != mFileSize ){
        printf( "!!! Only read %d bytes from vertex file!\n", readLength );
        mFileSize = readLength;
        return(-3);
    }

    mbEmpty = 0;
    
    return(0);
}

int FileReader::closeFile()
{
    if( mpBuffer != NULL )
    {
        free( mpBuffer );
        mpBuffer = NULL;
    }
    if( mpFile != NULL ) 
    {
        fclose( mpFile );
        mpFile = NULL;
    }

    mFileSize = 0;
    mbEmpty = 1;

    return(0);
}
