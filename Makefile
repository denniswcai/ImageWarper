TARGET      = ImageWarper
OBJECTS     = ImageWarper.o FileReader.o

INCLUDE_DIR = /usr/local/include
LIB_DIR     = /usr/local/lib 
LIBERARIES  = -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs -lpthread -ldl

#CPP_FLAGS   =
CPP_FLAGS   = -fopenmp 
#-DENABLE_OPENMP

CC          = g++

$(TARGET): $(OBJECTS)
	$(CC) -o $(TARGET) $(OBJECTS) -I $(INCLUDE_DIR) -L $(LIB_DIR) $(LIBERARIES) $(CPP_FLAGS)

$(OBJECTS): %.o: %.cpp
	$(CC) -c -I $(INCLUDE_DIR) $(CPP_FLAGS) $< -o $@

.PHONY:clean
clean:
	-rm $(TARGET) $(OBJECTS)
	

