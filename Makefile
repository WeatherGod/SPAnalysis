SHELL = /bin/sh

.SUFFIXES:

LIB_INSTALL=$(HOME)/lib
DEV_INSTALL=$(HOME)/include

all : libSPAnalysis.so

libSPAnalysis.so : StrongPointAnalysis.o Cluster.o
	$(CXX) -shared $^ -o $@ $(LDFLAGS) -lm
#	chcon -t texrel_shlib_t $@

StrongPointAnalysis.o : StrongPointAnalysis.C StrongPointAnalysis.h Cluster.h
	$(CXX) -c $< -o $@ -O3 $(CXXFLAGS)

Cluster.o : Cluster.C Cluster.h
	$(CXX) -c $< -o $@ $(CXXFLAGS)

install : libSPAnalysis.so StrongPointAnalysis.h Cluster.h
	install -t $(LIB_INSTALL) libSPAnalysis.so
	cp StrongPointAnalysis.h Cluster.h $(DEV_INSTALL)

remove :
	-rm -f $(LIB_INSTALL)/libSPAnalysis.so $(DEV_INSTALL)/StrongPointAnalysis.h $(DEV_INSTALL)/Cluster.h

clean :
	-rm -f libSPAnalysis.so StrongPointAnalysis.o Cluster.o
