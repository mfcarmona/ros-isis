CC			= gcc
INCDIR	= 
CFLAGS	= 

CL			= gcc
LIBDIR	= 
LIBS	 	= -lrt
LFLAGS	= 

obj3 		= cbr_client
obj5		= CBRCompiler
obj6		= cbr_server

.PHONY: all compile install clean cleanDeps cleanProj cleanTemps Tekkotsu reportTarget newstick update docs dox doc cleandoc updateTools updateLibs $(USERLIBS) checkInstallBinTimestamp test

all		:  $(obj3) $(obj4) $(obj5) $(obj6)

$(obj3) : 
	javac -cp . -encoding cp1252 $(obj3).java

$(obj4) : 
	javac -cp . -encoding cp1252 $(obj4).java

$(obj5) : 
	javac -cp . -encoding cp1252 $(obj5).java

$(obj6) : 
	javac -cp . -encoding cp1252 $(obj6).java

clean :
			rm -f *.o
			rm -f $(obj1)
			rm -f *.class
			
