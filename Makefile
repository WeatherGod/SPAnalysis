SHELL = /bin/sh

prefix = $(HOME)/.local
exec_prefix = $(prefix)
libdir = $(exec_prefix)/lib
includedir = $(prefix)/include

.SUFFIXES:

#LIB_INSTALL=$(HOME)/.local/lib
#DEV_INSTALL=$(HOME)/.local/include



all : libSPAnalysis.so

libSPAnalysis.so : StrongPointAnalysis.o Cluster.o
	$(CXX) -shared $^ -o $@ $(LDFLAGS) -lm

StrongPointAnalysis.o : StrongPointAnalysis.C StrongPointAnalysis.h Cluster.h
	$(CXX) -c $< -o $@ -O3 -fPIC $(CXXFLAGS)

Cluster.o : Cluster.C Cluster.h
	$(CXX) -c $< -o $@ -fPIC $(CXXFLAGS)

install_lib : libSPAnalysis.so
	install -t $(libdir) libSPAnalysis.so

install_dev : StrongPointAnalysis.h Cluster.h
	cp StrongPointAnalysis.h Cluster.h $(includedir)

install : install_lib install_dev

remove :
	-rm -f $(libdir)/libSPAnalysis.so $(includedir)/StrongPointAnalysis.h $(includedir)/Cluster.h

clean :
	-rm -f libSPAnalysis.so StrongPointAnalysis.o Cluster.o

.PHONY : all install_lib install_dev install remove clean
