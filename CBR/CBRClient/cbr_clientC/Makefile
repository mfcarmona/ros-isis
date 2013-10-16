CC			= gcc
INCDIR	= 
CFLAGS	= 

CL			= gcc
LIBDIR	= 
LIBS	 	= -lrt
LFLAGS	= 

lib		= libCbrClient
obj1		= libCbrClient_Test
obj2 		= server
obj3 		= client

.PHONY: all compile install clean cleanDeps cleanProj cleanTemps Tekkotsu reportTarget newstick update docs dox doc cleandoc updateTools updateLibs $(USERLIBS) checkInstallBinTimestamp test

all		: $(obj1)

$(obj1)	:  $(lib).o $(obj1).o
			$(CL) -o $(obj1) $(obj1).o $(lib).o $(LIBDIR) $(LIBS) $(LFLAGS)

$(obj1).o	: $(obj1).c
			$(CC) -c $(obj1).c $(INCDIR) $(CFLAGS)

$(lib).o : $(lib).c
			$(CC) -c $(lib).c $(INCDIR) $(CFLAGS)


clean :
			rm -f $(lib).o
			rm -f $(obj1)
			rm -f $(obj1).o
			


install : $(obj1)
	#mkdir -p /usr/local/bin
	#cp  $(EXECUTABLE) /usr/local/bin                            
	#mkdir -p /usr/local/etc
	#cp  $(CONFIG) /usr/local/etc                            

	mkdir -p libs
	cp  $(lib).o libs                            
	mkdir -p /usr/local/include
	cp $(lib).h /usr/local/include                      
	echo "libCbrClient Installed"                        

uninstall:
	#rm  /usr/local/bin/$(EXECUTABLE)                             
	rm  libs/$(lib).o                             
	rm /usr/local/include/$(lib).h
	echo "libCbrClient Uninstalled"      


