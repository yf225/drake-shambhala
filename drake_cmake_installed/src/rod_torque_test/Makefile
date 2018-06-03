CC = g++
CFLAGS = -I.
CXXFLAGS = -std=c++11
TARGET = feedforward_control
LIBS = -lEposCmd

all: $(TARGET)

$(TARGET): $(TARGET).cpp
	$(CC) -o $(TARGET) $(TARGET).cpp $(CFLAGS) $(CXXFLAGS) $(LIBS)

clean:
	$(RM) $(TARGET)
