CC = g++

CPPFLAGS = -g -Wall

LFLAGS = -lm

#SRCS = CostVec.cpp Grid.cpp Random.cpp GridSearch.cpp GridSearchACTR.cpp TestMain.cpp
SRCS = CostVec.cpp Grid.cpp Random.cpp GridSearch.cpp GridSearchACTR.cpp mystack.cpp mainTime.cpp

OBJS = $(SRCS:.cpp=.o)

PROGRAM = gridBOA

all: $(PROGRAM)

$(OBJS): Global.hpp CostVec.hpp Grid.hpp mystack.hpp GridSearch.hpp 

$(PROGRAM): $(OBJS)
	$(CC) $(OBJS) $(LFLAGS) -o $@
	
clean:
	rm -rf $(PROGRAM) $(OBJS) core

cb: clean $(PROGRAM)
