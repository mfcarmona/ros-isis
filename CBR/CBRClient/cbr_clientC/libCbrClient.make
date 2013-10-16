CC			= gcc
INCDIR	= 
CFLAGS	= 

CL			= gcc
LIBDIR	= 
LIBS	 	= -lrt
LFLAGS	= 

lib		= libCbrClient
lib_test = libCbrClient_Test

all		: $(lib_test)

$(lib_test)	: $(lib_test).o $(lib).o
			$(CL) -o $(lib_test) $(lib_test).o $(lib).o $(LIBDIR) $(LIBS) $(LFLAGS)

$(lib_test).o	: $(lib_test).c
			$(CC) -c $(lib_test).c $(INCDIR) $(CFLAGS)

$(lib).o : $(lib).c
			$(CC) -c $(lib).c $(INCDIR) $(CFLAGS)

clean :
			rm -f $(lib).o
			rm -f $(lib_test)
			rm -f $(lib_test).o
