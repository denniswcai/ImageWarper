//
#include <stdio.h>
#include <stdlib.h>

class FileReader
{
  public:
    FileReader();
    FileReader( const char *filename );
    ~FileReader();

    int   readFile( const char *filename );
    int   closeFile();
    void *getContents() { return( mpBuffer );  };
    long  fileSize()    { return( mFileSize ); };
    bool  isEmpty()     { return( mbEmpty );   };
  private:
    FILE *mpFile;
    void *mpBuffer;
    long  mFileSize;
    bool  mbEmpty;
};
