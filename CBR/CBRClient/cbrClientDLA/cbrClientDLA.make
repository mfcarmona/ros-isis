#MAKEFILE FOR cbrClientDLA MODULE

#DEPENDS: DLALibrary cbr_clientC libParams 

CURRENT_DIR=$(shell pwd) -I/usr/local/include

DLALIBRARY_DIR=/usr/local/lib
CBRCLIENT_DIR = /usr/local/lib
LIBPARAMS_DIR = /usr/local/lib
LIBANGLES_DIR = /usr/local/lib
LIBUTILS_DIR = /usr/local/lib

DLALIBRARY_LIBS = -lrt

INCDIR	= -I$(CURRENT_DIR) -I$(DLALIBRARY_DIR) -L$(ROBOTLIBRARY_DIR)  -I$(LIBPARAMS_DIR) -I$(LIBANGLES_DIR) -I$(LIBUTILS_DIR)

VPATH = $(CURRENT_DIR):$(DLALIBRARY_DIR):$(ROBOTLIBRARY_DIR):$(LIBPARAMS_DIR):$(LIBANGLES_DIR):$(LIBUTILS_DIR)

DEPS	=  $(DLALIBRARY_DIR)/DLALibrary.o $(CBRCLIENT_DIR)/libCbrClient.o  $(LIBPARAMS_DIR)/libParams.o $(LIBUTILS_DIR)/libUtils.o $(LIBANGLES_DIR)/libAngles.o

EXECUTABLE = cbrClientDLA
SOURCES	= cbrClientDLA.c
# HEADERS	= cbrClientDLA.h
OBJS	= $(SOURCES:.c=.o)  

CC			= gcc
CFLAGS	= -O3 -Wall -g3 -gdwarf-2

CL			= gcc
LIBDIR	= 
LIBS	 	= -lm $(DLALIBRARY_LIBS)
LFLAGS	= 

all		: $(EXECUTABLE)


$(OBJS) : $(SOURCES) 
			$(CC) -c $(SOURCES) $(INCDIR) $(CFLAGS)

$(EXECUTABLE):  $(OBJS)
			$(CL) $(DEPS) $(OBJS) $(LIBDIR) $(LIBS)  $(INCDIR) $(LFLAGS)  -o $(EXECUTABLE)


clean		:
			rm -f $(OBJS)
			rm -f $(EXECUTABLE)

